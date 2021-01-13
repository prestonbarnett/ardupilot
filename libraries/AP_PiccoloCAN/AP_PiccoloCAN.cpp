/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters
 */


#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AP_PiccoloCAN.h"

#if HAL_PICCOLO_CAN_ENABLE

#include <AP_Param/AP_Param.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "PiccoloCAN", fmt, ##args); } while (0)

// table of user-configurable Piccolo CAN bus parameters
const AP_Param::GroupInfo AP_PiccoloCAN::var_info[] = {

    // @Param: ESC_BM
    // @DisplayName: ESC channels
    // @Description: Bitmask defining which ESC (motor) channels are to be transmitted over Piccolo CAN
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16
    // @User: Advanced
    AP_GROUPINFO("ESC_BM", 1, AP_PiccoloCAN, _esc_bm, 0xFFFF),

    // @Param: ESC_RT
    // @DisplayName: ESC output rate
    // @Description: Output rate of ESC command messages
    // @Units: Hz
    // @User: Advanced
    // @Range: 1 500
    AP_GROUPINFO("ESC_RT", 2, AP_PiccoloCAN, _esc_hz, PICCOLO_MSG_RATE_HZ_DEFAULT),

    AP_GROUPEND
};

AP_PiccoloCAN::AP_PiccoloCAN()
{
    AP_Param::setup_object_defaults(this, var_info);

    debug_can(AP_CANManager::LOG_INFO, "PiccoloCAN: constructed\n\r");
}

AP_PiccoloCAN *AP_PiccoloCAN::get_pcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_PiccoloCAN) {
        return nullptr;
    }

    return static_cast<AP_PiccoloCAN*>(AP::can().get_driver(driver_index));
}

bool AP_PiccoloCAN::add_interface(AP_HAL::CANIface* can_iface) {
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Cannot add event handle\n\r");
        return false;
    }
    return true;
}

// initialize PiccoloCAN bus
void AP_PiccoloCAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_DEBUG, "PiccoloCAN: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: already initialized\n\r");
        return;
    }
    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PiccoloCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    snprintf(_thread_name, sizeof(_thread_name), "PiccoloCAN_%u", driver_index);

    debug_can(AP_CANManager::LOG_DEBUG, "PiccoloCAN: init done\n\r");
}

// loop to send output to CAN devices in background thread
void AP_PiccoloCAN::loop()
{
    AP_HAL::CANFrame rxFrame;


    // CAN Frame ID components
    // uint8_t frame_id_group;     // Piccolo message group
    uint16_t frame_id_device;   // Device identifier

    uint64_t timeout;

    while (true) {

        _esc_hz = constrain_int16(_esc_hz, PICCOLO_MSG_RATE_HZ_MIN, PICCOLO_MSG_RATE_HZ_MAX);

        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        timeout = AP_HAL::micros64() + 250;

        bool result = true;

        // 1ms loop delay
        hal.scheduler->delay_microseconds(1 * 1000);

        send_esc_messages();

        // Look for any message responses on the CAN bus
        while (read_frame(rxFrame, timeout)) {

            frame_id_device = rxFrame.id & 0x7FF;

            // Only accept extended messages
            if ((rxFrame.id & AP_HAL::CANFrame::FlagEFF) == 0) {
                continue;
            }

            // Check Arbitration IDs to get the telemetry data needed

            // TELEMETRY VARIABLES                                                               UNIT CONVERSION

            if (frame_id_device == TELEM_1) {

                motor_reply_data1_t reply_data;
                memcpy(reply_data.data, rxFrame.data, sizeof(reply_data.data));

                _telemetry.rpm = reply_data.rpm;
                _telemetry.ignition_angle = (float)reply_data.ignition_angle                        / 10.0f;
                _telemetry.throttle_angle = (float)reply_data.throttle_angle                        / 10.0f;
                _telemetry.lambda = reply_data.lambda;

            } else if (frame_id_device == TELEM_2) {

                motor_reply_data2_t reply_data;
                memcpy(reply_data.data, rxFrame.data, sizeof(reply_data.data));

                _telemetry.engine_load = (float)reply_data.engine_load                              / 10.0f;
                _telemetry.target_load = (float)reply_data.target_load                              / 10.0f;
                _telemetry.injection_time = reply_data.injection_time;
                _telemetry.injection_angle = (float)reply_data.injection_angle                      / 10.0f;

            } else if (frame_id_device == TELEM_3) {

                motor_reply_data3_t reply_data;
                memcpy(reply_data.data, rxFrame.data, sizeof(reply_data.data));

                _telemetry.ignition_spacing_angle = (float)reply_data.ignition_spacing_angle        / 10.0f;
                _telemetry.lambda_correction = (float)reply_data.lambda_correction                  / 10.0f;
                _telemetry.lambda_target = reply_data.lambda_target;
                _telemetry.injector_load = reply_data.injector_load;

            } else if (frame_id_device == TELEM_4) {

                motor_reply_data4_t reply_data;
                memcpy(reply_data.data, rxFrame.data, sizeof(reply_data.data));

                _telemetry.ecu_v = (float)reply_data.ecu_v                                          / 10.0f;
                _telemetry.ecu_temp = (float)reply_data.ecu_temp                                    / 10.0f;
                _telemetry.air_pressure = reply_data.air_pressure;
                _telemetry.fuel_con = (float)reply_data.fuel_con / 1000.0f;

            } else if (frame_id_device == TELEM_5) {

                motor_reply_data5_t reply_data;
                memcpy(reply_data.data, rxFrame.data, sizeof(reply_data.data));

                _telemetry.engine_temp = (float)reply_data.engine_temp                              / 10.0f;
                _telemetry.air_temp = (float)reply_data.air_temp                                    / 10.0f;
                _telemetry.exhaust_temp = (float)reply_data.exhaust_temp                            / 10.0f;
                _telemetry.oil_temp = (float)reply_data.oil_temp                                    / 10.0f;

            } else if (frame_id_device == TELEM_6) {

                // Error Flags Transmit
                // Each error flag represents one bit on the 8-byte can frame

                // Byte 1 Error Flags
                _telemetry.engine_temp_error = rxFrame.data[0]                                  << 0 & 0x80;
                _telemetry.engine_temp_error_2 = rxFrame.data[0]                                << 1 & 0x80;
                _telemetry.exhaust_temp_error = rxFrame.data[0]                                 << 2 & 0x80;
                _telemetry.exhaust_temp_error_2 = rxFrame.data[0]                               << 3 & 0x80;
                _telemetry.air_temp_error = rxFrame.data[0]                                     << 4 & 0x80;
                _telemetry.air_pressure_air = rxFrame.data[0]                                   << 5 & 0x80;
                _telemetry.oil_temp_error = rxFrame.data[0]                                     << 6 & 0x80;

                // Byte 3 Error Flags
                _telemetry.engine_temp_restriction = rxFrame.data[2]                            << 0 & 0x80;
                _telemetry.engine_temp_2_restriction = rxFrame.data[2]                          << 1 & 0x80;
                _telemetry.oil_temp_restriction = rxFrame.data[2]                               << 2 & 0x80;

                // Byte 4 Error Flags
                _telemetry.engine_temp_sgc_restriction = rxFrame.data[3]                        << 0 & 0x80;
                _telemetry.engine_temp_2_sgc_restriction = rxFrame.data[3]                      << 1 & 0x80;
                _telemetry.sgc_engine_temp_sgc_restriction = rxFrame.data[3]                    << 2 & 0x80;
                _telemetry.sgc_controller_temp_sgc_restriction = rxFrame.data[3]                << 3 & 0x80;
                _telemetry.oil_temp_sgc_restriction = rxFrame.data[3]                           << 4 & 0x80;

            } else {
                result = false;
            }

            if (result) {
                _motor_info.last_rx_msg_timestamp = timestamp;
            }
        }
    }
}

// write frame on CAN bus, returns true on success
bool AP_PiccoloCAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout)
{
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Driver not initialized for write_frame\n\r");
        return false;
    }

    bool read_select = false;
    bool write_select = true;
    bool ret;
    do {
        ret = _can_iface->select(read_select, write_select, &out_frame, timeout);
        if (!ret || !write_select) {
            hal.scheduler->delay_microseconds(50);
        }
    } while (!ret || !write_select);

    return (_can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError) == 1);
}

// read frame on CAN bus, returns true on succses
bool AP_PiccoloCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout)
{
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Driver not initialized for read_frame\n\r");
        return false;
    }
    bool read_select = true;
    bool write_select = false;
    bool ret = _can_iface->select(read_select, write_select, nullptr, timeout);
    if (!ret || !read_select) {
        // No frame available
        return false;
    }

    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags {};

    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

// called from SRV_Channels
void AP_PiccoloCAN::update()
{
    uint64_t timestamp = AP_HAL::micros64();

    /* Read out the ESC commands from the channel mixer */
    for (uint8_t i = 0; i < PICCOLO_CAN_MAX_NUM_ESC; i++) {

        if (is_esc_channel_active(i)) {

            uint16_t output = 0;
            
            SRV_Channel::Aux_servo_function_t motor_function = SRV_Channels::get_motor_function(i);

            if (SRV_Channels::get_output_pwm(motor_function, output)) {
                _motor_info.ouput = output;
            }
        }
    }
}

// send ESC telemetry messages over MAVLink
void AP_PiccoloCAN::send_esc_telemetry_mavlink(uint8_t mav_chan)
{
   mavlink_msg_skypower_telem1_send((mavlink_channel_t) mav_chan, _telemetry.rpm, _telemetry.ignition_angle, _telemetry.throttle_angle,
                                                        _telemetry.lambda, _telemetry.engine_load, _telemetry.target_load, _telemetry.injection_time,
                                                        _telemetry.injection_angle, _telemetry.ignition_spacing_angle, _telemetry.lambda_correction,
                                                        _telemetry.lambda_target, _telemetry.injector_load);

   mavlink_msg_skypower_telem2_send((mavlink_channel_t) mav_chan, _telemetry.ecu_v, _telemetry.ecu_temp, _telemetry.air_pressure, _telemetry.fuel_con,
                                                        _telemetry.engine_temp, _telemetry.air_temp, _telemetry.exhaust_temp, _telemetry.oil_temp);
}


// send ESC messages over CAN
void AP_PiccoloCAN::send_esc_messages(void)
{
    uint64_t timeout = AP_HAL::micros64() + 1000ULL;

    // frame_id_device = rxFrame.id & 0x7FF;
    // frame_id_t id = { { .object_address = 0x500,
    //                   .source_id = 0,
    //                   .priority = 0,
    //                   .unused = 0 } };

    union test_data {
        struct PACKED {
            uint16_t engine_mode;
            int16_t target_load;
            uint16_t target_power;
            uint16_t target_torque;
        };
        uint8_t data[8];
    };

    test_data test_frame;

    test_frame.engine_mode = 0;
    test_frame.target_load = 59;
    test_frame.target_power = 30;
    test_frame.target_torque = 52;

    if (hal.util->get_soft_armed()) {

	    engine_control = { ((uint16_t)0x510 & AP_HAL::CANFrame::MaskExtID), test_frame.data, sizeof(test_frame.data) };

	    write_frame(engine_control, timeout);
	} else {
		test_frame.target_load = 0;

		engine_control = { ((uint16_t)0x510 & AP_HAL::CANFrame::MaskExtID), test_frame.data, sizeof(test_frame.data) };

	    write_frame(engine_control, timeout);
	}
}

/**
 * Check if a given ESC channel is "active" (has been configured correctly)
 */
bool AP_PiccoloCAN::is_esc_channel_active(uint8_t chan)
{
    // First check if the particular ESC channel is enabled in the channel mask
    if (((_esc_bm >> chan) & 0x01) == 0x00) {
        return false;
    }

    // Check if a motor function is assigned for this motor channel
    SRV_Channel::Aux_servo_function_t motor_function = SRV_Channels::get_motor_function(chan);

    if (SRV_Channels::function_assigned(motor_function)) {
        return true;
    }

    return false;
}


/**
 * Determine if an ESC is present on the CAN bus (has telemetry data been received)
 */
bool AP_PiccoloCAN::is_esc_present(uint8_t chan, uint64_t timeout_ms)
{
    if (chan >= PICCOLO_CAN_MAX_NUM_ESC) {
        return false;
    }

    UASGS_Info_t &motor = _motor_info;

    // No messages received from this ESC
    if (motor.last_rx_msg_timestamp == 0) {
        return false;
    }

    uint64_t now = AP_HAL::micros64();

    uint64_t timeout_us = timeout_ms * 1000;

    if (now > (motor.last_rx_msg_timestamp + timeout_us)) {
        return false;
    }

    return true;
}


/**
 * Check if a given ESC is enabled (both hardware and software enable flags)
 */
bool AP_PiccoloCAN::is_esc_enabled(uint8_t chan)
{
    // If the ESC is not present, we cannot determine if it is enabled or not
    if (!is_esc_present(chan)) {
        return false;
    }

    // ESC is present, and enabled
    return true;

}


bool AP_PiccoloCAN::pre_arm_check(char* reason, uint8_t reason_len)
{
    return true;
}

#endif // HAL_PICCOLO_CAN_ENABLE

