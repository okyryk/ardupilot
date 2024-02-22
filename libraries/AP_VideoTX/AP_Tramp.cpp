/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   Code by Andy Piper, ported from betaflight vtx_tramp
*/

#include "AP_Tramp.h"
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_SerialManager/AP_SerialManager.h>

#if AP_TRAMP_ENABLED

#define TRAMP_MODEL_DEFAULT             0
#define TRAMP_MODEL_REAPER_INFINITY_5W  1

#define AP_TRAMP_UART_BAUD            9600

// request and response size is 16 bytes
#define AP_TRAMP_UART_BUFSIZE_RX      32
#define AP_TRAMP_UART_BUFSIZE_TX      32

// Define periods between requests
#define TRAMP_MIN_REQUEST_PERIOD_US (200 * 1000) // 200ms
#define TRAMP_STATUS_REQUEST_PERIOD_US (1000 * 1000) // 1s

//#define TRAMP_DEBUG
#ifdef TRAMP_DEBUG
# define debug(fmt, args...)	do { hal.console->printf("TRAMP: " fmt "\n", ##args); } while (0)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

#ifdef VTX_TRAMP_MODELS_SUPPORT
bool is_use_receive_response_interactive = false;
#endif

extern const AP_HAL::HAL &hal;

AP_Tramp::AP_Tramp()
{
    singleton = this;
}

AP_Tramp *AP_Tramp::singleton;

// Calculate tramp protocol checksum of provided buffer
uint8_t AP_Tramp::checksum(uint8_t *buf)
{
    uint8_t cksum = 0;

    for (int i = 1 ; i < TRAMP_BUF_SIZE - 2; i++) {
        cksum += buf[i];
    }

    return cksum;
}

// Send tramp protocol frame to device
void AP_Tramp::send_command(uint8_t cmd, uint16_t param)
{
    if (port == nullptr) {
        return;
    }

    memset(request_buffer, 0, ARRAY_SIZE(request_buffer));
    request_buffer[0] = 0x0F;
    request_buffer[1] = cmd;
    request_buffer[2] = param & 0xFF;
    request_buffer[3] = (param >> 8) & 0xFF;
    request_buffer[14] = checksum(request_buffer);

    port->write(request_buffer, TRAMP_BUF_SIZE);
    port->flush();

    debug("send command '%c': %u", cmd, param);

#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: send command '%c': %u", cmd, param);
#endif
}

// Process response and return code if valid else 0
char AP_Tramp::handle_response(void)
{
    const uint8_t respCode = response_buffer[1];

    switch (respCode) {
    case 'r': {
        const uint16_t min_freq = response_buffer[2]|(response_buffer[3] << 8);
        // Check we're not reading the request (indicated by freq zero)
        if (min_freq != 0) {
            // Got response, update device limits
            device_limits.rf_freq_min = min_freq;
            device_limits.rf_freq_max = response_buffer[4]|(response_buffer[5] << 8);
            device_limits.rf_power_max = response_buffer[6]|(response_buffer[7] << 8);
            debug("device limits: min freq: %u, max freq: %u, max power %u",
                unsigned(device_limits.rf_freq_min), unsigned(device_limits.rf_freq_max), unsigned(device_limits.rf_power_max));
#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: device limits: min freq: %u, max freq: %u, max power %u",
                unsigned(device_limits.rf_freq_min), unsigned(device_limits.rf_freq_max), unsigned(device_limits.rf_power_max));
#endif
            return 'r';
        }
        break;
    }
    case 'v': {
        const uint16_t freq = response_buffer[2]|(response_buffer[3] << 8);
        // Check we're not reading the request (indicated by freq zero)
        if (freq != 0) {
            // Got response, update device status
            const uint16_t power = response_buffer[4]|(response_buffer[5] << 8);
            cur_control_mode = response_buffer[6]; // Currently only used for race lock
            const bool pit_mode = response_buffer[7];
            cur_act_power = response_buffer[8]|(response_buffer[9] << 8);

            // update the vtx
            AP_VideoTX& vtx = AP::vtx();
            bool update_pending = vtx.have_params_changed();
            vtx.set_frequency_mhz(freq);

            AP_VideoTX::VideoBand band;
            uint8_t channel;
            if (vtx.get_band_and_channel(freq, band, channel)) {
                vtx.set_band(band);
                vtx.set_channel(channel);
            }

            vtx.set_power_mw(power);
            if (pit_mode) {
                vtx.set_options(vtx.get_options() | uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
            } else {
                vtx.set_options(vtx.get_options() & ~uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
            }

            // make sure the configured values now reflect reality
            // if they do then announce if there were changes
            if (!vtx.set_defaults() && update_pending && !vtx.have_params_changed()) {
                vtx.announce_vtx_settings();
            }

            debug("device config: freq: %u, power: %u, pitmode: %u",
                unsigned(freq), unsigned(power), unsigned(pit_mode));

#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: device config: freq: %u, power: %u, pitmode: %u",
                unsigned(freq), unsigned(power), unsigned(pit_mode));
#endif
            return 'v';
        }
        break;
    }
    case 's': {
        const uint16_t temp = (int16_t)(response_buffer[6]|(response_buffer[7] << 8));
        // Check we're not reading the request (indicated by temp zero)
        if (temp != 0) {
            // Got response, update device status
            cur_temp = temp;
            return 's';
        }
        break;
    }
    }

    // Likely reading a request, return zero to indicate not accepted
    return 0;
}

// Reset receiver state machine
void AP_Tramp::reset_receiver(void)
{
    receive_state = ReceiveState::S_WAIT_LEN;
    receive_pos = 0;
}

// returns completed response code or 0
char AP_Tramp::receive_response()
{
    if (port == nullptr) {
#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "port == nullptr");
#endif
        return 0;
    }

    // wait for complete packet
    const uint16_t bytesNeeded = TRAMP_BUF_SIZE - receive_pos;
    if (port->available() < bytesNeeded) {
#ifdef VTX_DEBUG_GCS
    const uint32_t available = port->available();
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "port->available() %u < bytesNeeded %u", unsigned(available), unsigned(bytesNeeded));


    for (uint16_t i = 0; i < available; i++) {
        if (receive_pos < TRAMP_BUF_SIZE) {
            const int16_t b = port->read();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "R:  %u  %#02x", unsigned(i), unsigned(b));
        }
    }

#endif
    return 0;

    }

    // sanity check
    if (bytesNeeded == 0) {
#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "bytesNeeded == 0");
#endif
        reset_receiver();
        return 0;
    }

    for (uint16_t i = 0; i < bytesNeeded; i++) {
        const int16_t b = port->read();
        if (b < 0) {
            // uart claimed bytes available, but there were none
#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "uart claimed bytes available, but there were none");
#endif
            return 0;
        }
        const uint8_t c = uint8_t(b);
        response_buffer[receive_pos++] = c;

        switch (receive_state) {
        case ReceiveState::S_WAIT_LEN: {
            if (c == 0x0F) {
                // Found header byte, advance to wait for code
                receive_state = ReceiveState::S_WAIT_CODE;
            } else {
                // Unexpected header, reset state machine
                reset_receiver();
            }
            break;
        }
        case ReceiveState::S_WAIT_CODE: {
            if (c == 'r' || c == 'v' || c == 's') {
                // Code is for response is one we're interested in, advance to data
                receive_state = ReceiveState::S_DATA;
            } else {
                // Unexpected code, reset state machine
                reset_receiver();
            }
            break;
        }
        case ReceiveState::S_DATA: {
            if (receive_pos == TRAMP_BUF_SIZE) {
                // Buffer is full, calculate checksum
                const uint8_t cksum = checksum(response_buffer);

                // Reset state machine ready for next response
                reset_receiver();

                if ((response_buffer[TRAMP_BUF_SIZE-2] == cksum) && (response_buffer[TRAMP_BUF_SIZE-1] == 0)) {
                    // Checksum is correct, process response
                    const char r = handle_response();

                    if (r != 0) {
                        return r;
                    }
                }
#ifdef VTX_DEBUG_GCS
                else {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Checksum failed");
                }
#endif
            }
            break;
        }
        default:
            // Invalid state, reset state machine
#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Invalid state, reset state machine");
#endif
            reset_receiver();
            break;
        }
    }

#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "end receive");
#endif
    return 0;
}

#ifdef VTX_TRAMP_MODELS_SUPPORT
char AP_Tramp::receive_response_interactive()
{
    if (port == nullptr) {
#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "port == nullptr");
#endif
        return 0;
    }

    const uint16_t bytesNeeded = TRAMP_BUF_SIZE - receive_pos;

    const uint32_t available = port->available();

    if (available == 0) {
        reset_receiver();
        return 0;
    }

    // sanity check
    if (bytesNeeded == 0) {
#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "bytesNeeded == 0");
#endif
        reset_receiver();
        return 0;
    }

    for (uint16_t i = 0; i < available; i++) {
        const int16_t b = port->read();
        if (b < 0) {
            // uart claimed bytes available, but there were none
#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "uart claimed bytes available, but there were none");
#endif
            return 0;
        }
        const uint8_t c = uint8_t(b);

#ifdef VTX_DEBUG_GCS
GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "R: %u  %u  %#02x", unsigned(receive_pos), unsigned(i), unsigned(c));
#endif

        response_buffer[receive_pos++] = c;

        switch (receive_state) {
        case ReceiveState::S_WAIT_LEN: {
            if (c == 0x0F) {
                // Found header byte, advance to wait for code
                receive_state = ReceiveState::S_WAIT_CODE;
            } else {
                // Unexpected header, reset state machine
                reset_receiver();
            }
            break;
        }
        case ReceiveState::S_WAIT_CODE: {
            if (c == 'r' || c == 'v' || c == 's') {
                // Code is for response is one we're interested in, advance to data
                receive_state = ReceiveState::S_DATA;
            } else {
                // Unexpected code, reset state machine
                reset_receiver();
            }
            break;
        }
        case ReceiveState::S_DATA: {
            if (receive_pos == TRAMP_BUF_SIZE) {
                // Buffer is full, calculate checksum

               
                const uint8_t cksum = checksum(response_buffer);

                // Reset state machine ready for next response
                reset_receiver();

                if ((response_buffer[TRAMP_BUF_SIZE-2] == cksum) && (response_buffer[TRAMP_BUF_SIZE-1] == 0)) {

                    const char r = handle_response();

#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "handle_response %#02x", unsigned(r));
#endif
                    // Check response valid else keep on reading
                    if (r != 0) {
                        return r;
                    }
                }
#ifdef VTX_DEBUG_GCS
                else {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Checksum failed");
                }
#endif
            }
            break;
        }
        default:
            // Invalid state, reset state machine
#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Invalid state, reset state machine");
#endif
            reset_receiver();
            break;
        }
    }

#ifdef VTX_DEBUG_GCS
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "end receive");
#endif
    return 0;
}
#endif

void AP_Tramp::send_query(uint8_t cmd)
{
    // Reset receive buffer and issue command
    reset_receiver();
    send_command(cmd, 0);
}

void AP_Tramp::set_status(TrampStatus _status)
{
    status = _status;
#ifdef TRAMP_DEBUG
    switch (status) {
        case TrampStatus::TRAMP_STATUS_OFFLINE:
            debug("status: OFFLINE");
            break;
        case TrampStatus::TRAMP_STATUS_INIT:
            debug("status: INIT");
            break;
        case TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT:
            debug("status: ONLINE_MONITOR_FREQPWRPIT");
            break;
        case TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_TEMP:
            debug("status: ONLINE_MONITOR_TEMP");
            break;
        case TrampStatus::TRAMP_STATUS_ONLINE_CONFIG:
            debug("status: ONLINE_CONFIG");
            break;
    }
#endif

#ifdef VTX_DEBUG_GCS
    switch (status) {
        case TrampStatus::TRAMP_STATUS_OFFLINE:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: status: OFFLINE");
            break;
        case TrampStatus::TRAMP_STATUS_INIT:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: status: INIT");
            break;
        case TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: status: ONLINE_MONITOR_FREQPWRPIT");
            break;
        case TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_TEMP:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: status: ONLINE_MONITOR_TEMP");
            break;
        case TrampStatus::TRAMP_STATUS_ONLINE_CONFIG:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: status: ONLINE_CONFIG");
            break;
    }
#endif
}

void AP_Tramp::process_requests()
{
    if (port == nullptr) {
        return;
    }

    bool configUpdateRequired = false;

    // Read response from device
#ifdef VTX_TRAMP_MODELS_SUPPORT
    const char replyCode = is_use_receive_response_interactive 
    ? receive_response_interactive() 
    : receive_response();
#else
    const char replyCode = receive_response();
#endif

    
    const uint32_t now = AP_HAL::micros();

#ifdef TRAMP_DEBUG
    if (replyCode != 0) {
        debug("receive response '%c'", replyCode);
    }
#endif

#ifdef VTX_DEBUG_GCS
    if (replyCode != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: receive response '%c'", replyCode);
    }

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: status '%u'", static_cast<int>(status));
#endif

    // Act on state
    switch (status) {
    case TrampStatus::TRAMP_STATUS_OFFLINE: {
        // Offline, check for response
        if (replyCode == 'r') {
            // Device replied to reset? request, enter init
            set_status(TrampStatus::TRAMP_STATUS_INIT);
        } else if ((now - last_time_us) >= TRAMP_MIN_REQUEST_PERIOD_US) {

#ifdef VTX_DEBUG_GCS
    u_int32_t vdif = now - last_time_us;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: now: %u, last_time_us: %u, dif: %u",
            unsigned(now), unsigned(last_time_us), unsigned(vdif));
#endif
            // Min request period exceeded, issue another reset?
            send_query('r');

            // Update last time
            last_time_us = now;
        }
        break;
    }
    case TrampStatus::TRAMP_STATUS_INIT: {
        // Initializing, check for response
        if (replyCode == 'v') {
            // Device replied to freq / power / pit query, enter online
            set_status(TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT);
        } else if ((now - last_time_us) >= TRAMP_MIN_REQUEST_PERIOD_US) {

            // Min request period exceeded, issue another query
            send_query('v');

            // Update last time
            last_time_us = now;
        }
        break;
    }
    case TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT: {
        // Note after config a status update request is made, a new status
        // request is made, this request is handled above and should prevent
        // subsequent config updates if the config is now correct
        if (retry_count > 0 && ((now - last_time_us) >= TRAMP_MIN_REQUEST_PERIOD_US)) {
            AP_VideoTX& vtx = AP::vtx();
            // Config retries remain and min request period exceeded, check freq
            if (!is_race_lock_enabled() && vtx.update_frequency()) {
                // Freq can be and needs to be updated, issue request
                send_command('F', vtx.get_configured_frequency_mhz());

                // Set flag
                configUpdateRequired = true;
            } else if (!is_race_lock_enabled() && vtx.update_power()) {
                // Power can be and needs to be updated, issue request
                send_command('P', vtx.get_configured_power_mw());

                // Set flag
                configUpdateRequired = true;
            } else if (vtx.update_options()) {
                // Pit mode needs to be updated, issue request
                send_command('I', vtx.has_option(AP_VideoTX::VideoOptions::VTX_PITMODE) ? 0 : 1);

                // Set flag
                configUpdateRequired = true;
            }

            if (configUpdateRequired) {
                // Update required, decrement retry count
                retry_count--;

                // Update last time
                last_time_us = now;

                // Advance state
                set_status(TrampStatus::TRAMP_STATUS_ONLINE_CONFIG);
            } else {
                // No update required, reset retry count
                retry_count = 0;
            }
        }

        /* Was a config update made? */
        if (!configUpdateRequired) {
            /* No, look to continue monitoring */
            if ((now - last_time_us) >= TRAMP_STATUS_REQUEST_PERIOD_US) {
                // Request period exceeded, issue freq/power/pit query
                send_query('v');

                // Update last time
                last_time_us = now;
            } else if (replyCode == 'v') {
                // Got reply, issue temp query
                send_query('s');

                // Wait for reply
                set_status(TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_TEMP);

                // Update last time
                last_time_us = now;
            }
        }

        break;
    }
    case TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_TEMP: {
        // Check request time
        if (replyCode == 's') {
            // Got reply, return to request freq/power/pit
            set_status(TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_TEMP);
        } else if ((now - last_time_us) >= TRAMP_MIN_REQUEST_PERIOD_US) {
            // Timed out after min request period, return to request freq/power/pit query
            set_status(TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT);
        }
        break;
    }
    case TrampStatus::TRAMP_STATUS_ONLINE_CONFIG: {
        // Param should now be set, check time
        if ((now - last_time_us) >= TRAMP_MIN_REQUEST_PERIOD_US) {
            // Min request period exceeded, re-query
            send_query('v');

            // Advance state
            set_status(TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT);

            // Update last time
            last_time_us = now;
        }
        break;
    }
    default:
        // Invalid state, reset
        set_status(TrampStatus::TRAMP_STATUS_OFFLINE);
        break;
    }
}

bool AP_Tramp::is_device_ready()
{
    return status >= TrampStatus::TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT;
}

void AP_Tramp::set_frequency(uint16_t freq)
{
    uint8_t freqValid;

    // Check frequency valid
    if (device_limits.rf_freq_min != 0 && device_limits.rf_freq_max != 0) {
        freqValid = (freq >= device_limits.rf_freq_min && freq <= device_limits.rf_freq_max);
    } else {
        freqValid = (freq >= VTX_TRAMP_MIN_FREQUENCY_MHZ && freq <= VTX_TRAMP_MAX_FREQUENCY_MHZ);
    }

    // Is frequency valid?
    if (freqValid) {
        // Requested freq changed, reset retry count
        retry_count = VTX_TRAMP_MAX_RETRIES;
    } else {
        debug("requested frequency %u is invalid", freq);

#ifdef VTX_DEBUG_GCS
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: requested frequency %u is invalid", freq);
#endif
        // not valid reset to default
        AP::vtx().set_configured_frequency_mhz(AP::vtx().get_frequency_mhz());
    }
}

void AP_Tramp::update()
{
    if (port == nullptr) {
        return;
    }

    AP_VideoTX& vtx = AP::vtx();

    if (vtx.have_params_changed() && retry_count == 0) {
        // check changes in the order they will be processed
        if (vtx.update_frequency() || vtx.update_band() || vtx.update_channel()) {
            if (vtx.update_frequency()) {
                vtx.update_configured_channel_and_band();
            } else {
                vtx.update_configured_frequency();
            }
            set_frequency(vtx.get_configured_frequency_mhz());
        }
        else if (vtx.update_power()) {
            retry_count = VTX_TRAMP_MAX_RETRIES;
        }
        else if (vtx.update_options()) {
            retry_count = VTX_TRAMP_MAX_RETRIES;
        }
    }

    process_requests();
}

bool AP_Tramp::init(void)
{
    if (AP::vtx().get_enabled() == 0) {
        debug("protocol is not active");
#ifdef VTX_DEBUG_GCS
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: protocol is not active");
#endif
        return false;
    }

    if (AP::vtx().get_vtx_table_enabled()) {
        AP::vtx()._power_levels[0].level = AP::vtx().get_configured_vtx_table_0_power_level();
        AP::vtx()._power_levels[0].mw = AP::vtx().get_configured_vtx_table_0_power_level_mw();
        AP::vtx()._power_levels[0].dbm = AP::vtx().get_configured_vtx_table_0_power_level_dbm();
        AP::vtx()._power_levels[0].dac = 0;

        AP::vtx()._power_levels[1].level = AP::vtx().get_configured_vtx_table_1_power_level();
        AP::vtx()._power_levels[1].mw = AP::vtx().get_configured_vtx_table_1_power_level_mw();
        AP::vtx()._power_levels[1].dbm = AP::vtx().get_configured_vtx_table_1_power_level_dbm();
        AP::vtx()._power_levels[1].dac = 7;

        AP::vtx()._power_levels[2].level = AP::vtx().get_configured_vtx_table_2_power_level();
        AP::vtx()._power_levels[2].mw = AP::vtx().get_configured_vtx_table_2_power_level_mw();
        AP::vtx()._power_levels[2].dbm = AP::vtx().get_configured_vtx_table_2_power_level_dbm();
        AP::vtx()._power_levels[2].dac = 0xFF;

        AP::vtx()._power_levels[3].level = AP::vtx().get_configured_vtx_table_3_power_level();
        AP::vtx()._power_levels[3].mw = AP::vtx().get_configured_vtx_table_3_power_level_mw();
        AP::vtx()._power_levels[3].dbm = AP::vtx().get_configured_vtx_table_3_power_level_dbm();
        AP::vtx()._power_levels[3].dac = 0xFF;

        AP::vtx()._power_levels[4].level = AP::vtx().get_configured_vtx_table_4_power_level();
        AP::vtx()._power_levels[4].mw = AP::vtx().get_configured_vtx_table_4_power_level_mw();
        AP::vtx()._power_levels[4].dbm = AP::vtx().get_configured_vtx_table_4_power_level_dbm();
        AP::vtx()._power_levels[4].dac = 0xFF;

        AP::vtx()._power_levels[5].level = AP::vtx().get_configured_vtx_table_5_power_level();
        AP::vtx()._power_levels[5].mw = AP::vtx().get_configured_vtx_table_5_power_level_mw();
        AP::vtx()._power_levels[5].dbm = AP::vtx().get_configured_vtx_table_5_power_level_dbm();
        AP::vtx()._power_levels[5].dac = 0;

        AP::vtx()._power_levels[6].level = AP::vtx().get_configured_vtx_table_6_power_level();
        AP::vtx()._power_levels[6].mw = AP::vtx().get_configured_vtx_table_6_power_level_mw();
        AP::vtx()._power_levels[6].dbm = AP::vtx().get_configured_vtx_table_6_power_level_dbm();
        AP::vtx()._power_levels[6].dac = 0;

        AP::vtx()._power_levels[7].level = AP::vtx().get_configured_vtx_table_7_power_level();
        AP::vtx()._power_levels[7].mw = AP::vtx().get_configured_vtx_table_7_power_level_mw();
        AP::vtx()._power_levels[7].dbm = AP::vtx().get_configured_vtx_table_7_power_level_dbm();
        AP::vtx()._power_levels[7].dac = 0;

        AP::vtx()._power_levels[8].level = AP::vtx().get_configured_vtx_table_8_power_level();
        AP::vtx()._power_levels[8].mw = AP::vtx().get_configured_vtx_table_8_power_level_mw();
        AP::vtx()._power_levels[8].dbm = AP::vtx().get_configured_vtx_table_8_power_level_dbm();
        AP::vtx()._power_levels[8].dac = 0;
    }

#ifdef VTX_TRAMP_MODELS_SUPPORT
    u_int32_t tramp_uart_baud = AP_TRAMP_UART_BAUD;

    if (AP::vtx().get_vtx_tramp_baud() > 7000 && AP::vtx().get_vtx_tramp_baud() < 11000){
        tramp_uart_baud = AP::vtx().get_vtx_tramp_baud();
    }

    if (AP::vtx().get_vtx_tramp_transmiter_model() == TRAMP_MODEL_REAPER_INFINITY_5W){
        is_use_receive_response_interactive = true;
    }
#endif

    // init uart
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Tramp, 0);
    if (port != nullptr) {
        port->configure_parity(0);
        port->set_stop_bits(1);
        port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        port->set_options((port->get_options() & ~AP_HAL::UARTDriver::OPTION_RXINV));
#ifdef VTX_TRAMP_MODELS_SUPPORT
        port->begin(tramp_uart_baud, AP_TRAMP_UART_BUFSIZE_RX, AP_TRAMP_UART_BUFSIZE_TX);
#else
        port->begin(AP_TRAMP_UART_BAUD, AP_TRAMP_UART_BUFSIZE_RX, AP_TRAMP_UART_BUFSIZE_TX);
#endif
        debug("port opened");
#ifdef VTX_DEBUG_GCS
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRAMP: port opened");
#endif
        return true;
    }
    return false;
}

#endif // VTX_TRAMP
