/// @file	AP_MotorsCoax.h
/// @brief	Motor and Servo control class for Co-axial helicopters with two motors and two flaps
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include "AP_MotorsMulticopter.h"

// feedback direction
#define AP_MOTORS_COAX_POSITIVE      1
#define AP_MOTORS_COAX_NEGATIVE     -1

#define NUM_ACTUATORS 4

#define AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS 250 // update rate for digital servos
#define AP_MOTORS_SINGLE_SPEED_ANALOG_SERVOS 125  // update rate for analog servos

#define AP_MOTORS_COAX_SERVO_INPUT_RANGE    4500    // roll or pitch input of -4500 will cause servos to their minimum (i.e. radio_min), +4500 will move them to their maximum (i.e. radio_max)

/// @class      AP_MotorsSingle
class AP_MotorsCoax : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsCoax(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
    };

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void                set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz ) override;

    // output_to_motors - sends minimum values out to the motors
    virtual void        output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint32_t            get_motor_mask() override;

    // Run arming checks
    bool arming_checks(size_t buflen, char *buffer) const override { return AP_Motors::arming_checks(buflen, buffer); }

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing() override;
    void                output_armed_stabilizing_Eric() override;
    float               _actuator_out[NUM_ACTUATORS]; // combined roll, pitch, yaw and throttle outputs to motors in 0~1 range
    float               _thrust_yt_ccw;
    float               _thrust_yt_cw;

    const char* _get_frame_string() const override { return "COAX"; }

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;
};
