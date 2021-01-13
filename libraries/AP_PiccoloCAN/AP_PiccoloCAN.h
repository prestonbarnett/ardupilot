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

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/Semaphores.h>

#include <AP_Param/AP_Param.h>

#include "piccolo_protocol/ESCPackets.h"

// maximum number of ESC allowed on CAN bus simultaneously
#define PICCOLO_CAN_MAX_NUM_ESC 12
#define PICCOLO_CAN_MAX_GROUP_ESC (PICCOLO_CAN_MAX_NUM_ESC / 4)

#ifndef HAL_PICCOLO_CAN_ENABLE
#define HAL_PICCOLO_CAN_ENABLE (HAL_NUM_CAN_IFACES && !HAL_MINIMIZE_FEATURES)
#endif

#if HAL_PICCOLO_CAN_ENABLE

#define PICCOLO_MSG_RATE_HZ_MIN 1
#define PICCOLO_MSG_RATE_HZ_MAX 500
#define PICCOLO_MSG_RATE_HZ_DEFAULT 50

class AP_PiccoloCAN : public AP_CANDriver
{
public:
    AP_PiccoloCAN();
    ~AP_PiccoloCAN();

    // Piccolo message groups form part of the CAN ID of each frame
    enum class MessageGroup : uint8_t {
        SIMULATOR = 0x00,       // Simulator messages
        SENSOR = 0x04,          // External sensors
        ACTUATOR = 0x07,        // Actuators (e.g. ESC / servo)
        ECU_OUT = 0x08,         // Messages *from* an ECU
        ECU_IN = 0x09,          // Message *to* an ECU

        SYSTEM = 0x19,          // System messages (e.g. bootloader)
    };

    // Piccolo actuator types differentiate between actuator frames
    enum class ActuatorType : uint8_t {
        SERVO = 0x00,
        ESC = 0x20,
    };

    /* Do not allow copies */
    AP_PiccoloCAN(const AP_PiccoloCAN &other) = delete;
    AP_PiccoloCAN &operator=(const AP_PiccoloCAN&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // Return PiccoloCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_PiccoloCAN *get_pcan(uint8_t driver_index);

    // initialize PiccoloCAN bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // called from SRV_Channels
    void update();

    // send ESC telemetry messages over MAVLink
    void send_esc_telemetry_mavlink(uint8_t mav_chan);

    // return true if a particular ESC is 'active' on the Piccolo interface
    bool is_esc_channel_active(uint8_t chan);

    // return true if a particular ESC has been detected
    bool is_esc_present(uint8_t chan, uint64_t timeout_ms = 2000);

    // return true if a particular ESC is enabled
    bool is_esc_enabled(uint8_t chan);

    // test if the Piccolo CAN driver is ready to be armed
    bool pre_arm_check(char* reason, uint8_t reason_len);

private:

    static constexpr uint16_t TELEM_1 = 0x100;
    static constexpr uint16_t TELEM_2 = 0x101;
    static constexpr uint16_t TELEM_3 = 0x102;
    static constexpr uint16_t TELEM_4 = 0x104;
    static constexpr uint16_t TELEM_5 = 0x105;
    static constexpr uint16_t TELEM_6 = 0x11C;

    static constexpr uint16_t COMMAND_CONTROL = 0x500;
    static constexpr uint16_t COMMAND_START = 0x505;

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on succses
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);

    // send ESC commands over CAN
    void send_esc_messages(void);

    bool _initialized;
    char _thread_name[16];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_EventHandle _event_handle;
    HAL_Semaphore _telem_sem;

    struct UASGS_Info_t {

        int16_t output;    //! Raw command to send to each ESC

        uint64_t last_rx_msg_timestamp = 0;    //! Time of most recently received message

    } _motor_info;

    // Piccolo CAN parameters
    AP_Int32 _esc_bm;       //! ESC selection bitmask
    AP_Int16 _esc_hz;       //! ESC update rate (Hz)

    union frame_id_t {
        struct {
            uint16_t object_address;
            uint8_t source_id;
            uint8_t priority:5;
            uint8_t unused:3;
        };
        uint32_t value;
    };

    union motor_reply_data1_t {
        struct PACKED {
            uint16_t rpm;
            int16_t ignition_angle;
            uint16_t throttle_angle;
            uint16_t lambda;
        };
        uint8_t data[8];
    };
    union motor_reply_data2_t {
        struct PACKED {
            uint16_t engine_load;
            uint16_t target_load;
            uint16_t injection_time;
            uint16_t injection_angle;
        };
        uint8_t data[8];
    };
    union motor_reply_data3_t {
        struct PACKED {
            int16_t ignition_spacing_angle;
            int16_t lambda_correction;
            uint16_t lambda_target;
            uint16_t injector_load;
        };
        uint8_t data[8];
    };
    union motor_reply_data4_t {
        struct PACKED {
            uint16_t ecu_v;
            uint16_t ecu_temp;
            uint16_t air_pressure;
            uint16_t fuel_con;
        };
        uint8_t data[8];
    };
    union motor_reply_data5_t {
        struct PACKED {
            uint16_t engine_temp;
            int16_t air_temp;
            uint16_t exhaust_temp;
            uint16_t oil_temp;
        };
        uint8_t data[8];
    };

    struct telemetry_info {
        uint16_t rpm;
        float ignition_angle;
        float throttle_angle;
        uint16_t lambda;
        float engine_load;
        float target_load;
        uint16_t injection_time;
        float injection_angle;
        float ignition_spacing_angle;
        float lambda_correction;
        uint16_t lambda_target;
        uint16_t injector_load;
        float ecu_v;
        float ecu_temp;
        uint16_t air_pressure;
        float fuel_con;
        float engine_temp;
        float air_temp;
        float exhaust_temp;
        float oil_temp;
        bool engine_temp_error;
        bool engine_temp_error_2;
        bool exhaust_temp_error;
        bool exhaust_temp_error_2;
        bool air_temp_error;
        bool air_pressure_air;
        bool oil_temp_error;
        bool engine_temp_restriction;
        bool engine_temp_2_restriction;
        bool oil_temp_restriction;
        bool engine_temp_sgc_restriction;
        bool engine_temp_2_sgc_restriction;
        bool sgc_engine_temp_sgc_restriction;
        bool sgc_controller_temp_sgc_restriction;
        bool oil_temp_sgc_restriction;
    } _telemetry;

    AP_HAL::CANFrame engine_control;
    AP_HAL::CANFrame engine_start_stop;
};

#endif // HAL_PICCOLO_CAN_ENABLE
