#pragma once

struct canopen_drive_profile {
	/*!
	 * 0x6007
	 * This object indicates the action to be performed when one of the following events occurs:
	 * CAN bus off
	 * Heartbeat lost
	 * Node guarding lost
	 * NMT stopped (stop remote node indication activated)
	 * Reset communication (reset communication indication activated)
	 * Reset application (reset node indication activated)
	 *
	 *  The following value definitions are valid:
	 *  0 = No action
	 *  1 = Fault signal
	 *  2 = Disable voltage command
	 *  3 = Quick Stop command
	 *  -x = Manufacturer-specific
	 */
	uint8_t abort_action;
	/*
	 * 0x603F
	 * This object indicates the error code of the last error that occurred in the drive device.
	 */
	uint16_t error;
	/*
	 * 0x6040
	 * This object controls the CiA-402 FSA, CiA-402 modes and manufacturer-specific entities.
	 * This object is organized bit-wise. The bits have the following meaning:
	 * bit 0: switch on
	 * bit 1: enable voltage
	 * bit 2: quick stop
	 * bit 3: enable operation
	 * bit 4-6: mode-specific
	 * bit 7: fault reset
	 * bit 8: halt
	 * bit 9: mode-specific
	 * bit 10: reserved
	 * bit 11: begin on time
	 * bit 12-15: manufacturer-specific
	 */
	uint16_t control;
	/*
	 * 0x6041
	 * This object indicates the current state of the FSA, the operation mode and manufacturer-specific entities.
	 * This object is organized bit-wise. The bits have the following meaning:
	 * bit 0: ready to switch on
	 * bit 1: switched on
	 * bit 2: operation enabled
	 * bit 3: fault
	 * bit 4: voltage enabled
	 * bit 5: quick stop
	 * bit 6: switch on disabled
	 * bit 7: warning
	 * bit 8: manufacturer-specific
	 * bit 9: remote
	 * bit 10: target reached
	 * bit 11: internal limit active
	 * bit 12-13: mode-specific
	 * bit 14-15: manufacturer-specific
	 */
	uint16_t status;
	/*
	 * 0x6060
	 * The object selects the operational mode. This object shows only the
	 * value of the requested operation mode. The actual operation mode of the
	 * PDS is reflected in the Modes of Operation Display object (6061h) The
	 * following value definitions are valid:
	 * 0 = no mode change / no mode assigned
	 * 1 = profile position mode
	 * 2 = velocity mode
	 * 3 = profile velocity mode
	 * 4 = profile torque mode
	 * 5 = reserved
	 * 6 = homing mode
	 * 7 = interpolated position mode
	 * 8 = cyclic synchronous position mode
	 * 9 = cyclic synchronous velocity mode
	 * 10 = cyclic synchronous torque mode
	 * -x = manufacturer-specific
	 */
	uint8_t requested_mode;
	/*
	 * 0x6061
	 * This object indicates the actual operation mode.
	 * The following value definitions are valid:
	 * 0 = no mode change / no mode assigned
	 * 1 = profile position mode
	 * 2 = velocity mode
	 * 3 = profile velocity mode
	 * 4 = profile torque mode
	 * 5 = reserved
	 * 6 = homing mode
	 * 7 = interpolated position mode
	 * 8 = cyclic synchronous position mode
	 * 9 = cyclic synchronous velocity mode
	 * 10 = cyclic synchronous torque mode
	 * -x = manufacturer-specific
	 */
	uint8_t current_mode;
	/*
	 * 0x6062
	 * This object indicates the demanded position value.
	 */
	int32_t position_demand;
	/*
	 * 0x6063
	 * This object indicates the predicted position value after kalman filtering.
	 * Note: if internal value is a float, this value is truncated to nearest integer. 
	 */
	int32_t position_actual;
	/*
	 * 0x6064
	 * This object indicates the actual value of the position measurement device.
	 */
	int32_t position_measured;
	/*
	 * 0x6065
	 * This object indicates the symmetrical range of tolerated position values
	 * relative to the target position. If the current position is out of range
	 * a following error occurs.  This object indicates the range of tolerated
	 * position values symmetrically to the position demand value (object
	 * 6062h). If the following error actual value (object 60F4h) is out of the
	 * following error window, a following error occurs. A following error may
	 * occur when a drive is blocked, or an unreachable profile velocity
	 * occurs, or due to incorrect closed-loop coefficients. If the value of
	 * the following error window is FFFFFFFFh, following control is disabled.
	 */
	uint32_t following_error_window;
	/*
	 * 0x6067
	 * This object indicates the symmetrical range of accepted positions
	 * relative to the target position. If the actual value of the position
	 * encoder is within the position window, the target position is regarded
	 * as reached. If the value of the position window is FFFFFFFFh, position
	 * window control is disabled.
	 */
	uint32_t position_window;
	/* 
	 * 0x606B
	 * This object indicates the output value of the trajectory generator.
	 */
	int32_t velocity_demand;
	/*
	 * 0x606C
	 * This object indicates the actual velocity value derived either from the
	 * velocity sensor or the position sensor.
	 */
	int32_t velocity_actual;
	/*
	 * 0x606D
	 * This object indicates the velocity window.
	 */
	uint16_t velocity_window;
	/*
	 * 0x606F
	 * This object indicates the velocity threshold.
	 */
	uint16_t velocity_threshold;
	/*
	 * 0x6070
	 * This object indicates the velocity threshold time in ms
	 */
	uint16_t velocity_threshold_time;
	/*
	 * 0x6071
	 * This object indicates the input value for the torque controller in profile torque mode in mNm
	 */
	int16_t target_torque;
	/*
	 * 0x6073
	 * This object indicates the maximum permissible torque creating current in the motor in mA
	 */
	uint16_t max_current;
	/*
	 * 0x6074
	 * This object provides the command value for the current loop in mA
	 */
	int16_t torque_demand;
	/*
	 * 0x6075
	 * This object provides the motor rated current in mA
	 */
	uint32_t motor_rated_current;
	/*
	 * 0x6078
	 * This object indicates the actual value of the current. It corresponds to
	 * the current in the motor.
	 */
	uint16_t motor_current;
	/*
	 * 0x6079
	 * This object indicates the instantaneous DC link current voltage at the drive device in mV
	 */
	uint32_t dc_voltage;
	/*
	 * 0x607A
	 * This object indicates the commanded position to which the drive will
	 * move in position profile mode or cyclic synchronous position mode. The
	 * value of this object can be interpreted as absolute or relative
	 * depending on bit 6 of the controlword.
	 */
	int32_t target_position;
	/*
	 * 0x607B
	 * This object indicates the maximum and minimum position range limits. It
	 * limits the numerical range of the input value. Upon reaching or
	 * exceeding these limits, the input value automatically wraps to the other
	 * end of the range. Wrap-around of the input value may be prevented by
	 * setting software position limits as defined in the software position
	 * limit object (607Dh).
	 */
	int32_t min_position_range_limit;
	int32_t max_position_range_limit;
	/*
	 * 0x607C
	 * This object indicates the difference between the zero position for the
	 * application and the machine home position. After the machine home
	 * position is found and homing is completed, the zero position is offset
	 * from the home position by adding the home offset value to the home
	 * position. All subsequent absolute moves are executed relative to this
	 * new zero position.  If this object is not implemented, home offset is
	 * considered to be 0. Negative values indicate the opposite direction.
	 */
	int32_t home_offset;
	/*
	 * 0x607D
	 * This object indicates the maximum and minimum software position limits.
	 * These parameters define the absolute position limits for the position
	 * demand value and the position actual value. Every new target position is
	 * checked against these limits. The limit positions are always relative to
	 * the machine home position. Before being compared to the target position,
	 * they are corrected internally by the home offset, as follows:
	 *
	 * Corrected min position limit = (min position limit - home offset)
	 * Corrected max position limit = (max position limit - home offset)
	 */
	int32_t min_software_pos_limit;
	int32_t max_software_pos_limit;
	/*
	 * 0x607E
	 * Position demand value and position actual value are multiplied by 1 or
	 * -1, depending on the value of the polarity flag.
	 */
	uint8_t polarity;
	/*
	 * 0x6081
	 * This object indicates the commanded velocity normally attained at the
	 * end of the acceleration ramp during a profiled motion. It is valid for
	 * both directions of motion.
	 *
	 * This object is used in profile position mode and interpolated position
	 * mode.
	 */
	int32_t profile_velocity;
	/*
	 * 0x6083
	 * This object indicates the commanded acceleration.
	 *
	 * This object is used in the profile position mode, profile velocity mode,
	 * and interpolated position mode.
	 */
	uint32_t profile_acceleration;
	/*
	 * 0x6084
	 * This object indicates the deceleration.
	 *
	 * This object is used in the profile position mode, profile velocity mode,
	 * and interpolated position mode.
	 */
	uint32_t profile_deceleration;
	/*
	 * 0x6085
	 * This object indicates the deceleration used to stop the motor when the
	 * quick stop function is activated and the quick stop option code is set
	 * to 2 or 6.  The quick stop deceleration is also used if the fault
	 * reaction option code is 2 and the halt option code is 2.
	 */
	uint32_t quick_stop_deceleration;
	/*
	 * 0x6086
	 * This object indicates the type of motion profile used to perform a
	 * profiled motion.  The following value definitions are valid:
	 * 0 = linear ramp (trapezoidal profile)
	 */
	int16_t motion_profile_type;
	/*
	 * 0x6089
	 * The position notation index is used to scale the objects for which it mandatory.
	 * Note: the value of this object is fixed to factor = 1.
	 */
	int8_t position_notation_index;
	/*
	 * 0x608A
	 * This object indicates position units.
	 * Note: the value of this object is fixed to steps.
	 */
	uint8_t position_dimension_index;
	/*
	 * 0x608B
	 * The velocity notation index is used to scale the objects for which it mandatory.
	 * Note: the value of this object is fixed at 0.01.
	 */
	int8_t velocity_notation_index;
	/*
	 * 0x608C
	 * This object indicates velocity units.
	 * Note: the value of this object is fixed at rpm.
	 */
	int8_t velocity_dimension_index;
	/*
	 * 0x608D
	 * The acceleration notation index is used to scale the objects for which it mandatory.
	 * Note: the value of this object is fixed tat 0.01.
	 */
	int8_t acceleration_notation_index;
	/*
	 * 0x608E
	 * This object indicates acceleration units.
	 * Note: the value of this object is fixed at rpm/second
	 */
	uint8_t acceleration_dimension_index;
	/*
	 * 0x608F
	 * This object indicates the configured encoder increments and number of
	 * motor revolutions. It is calculated by the following formula: position
	 * encoder resolution = (encoder increments/motor revolutions)
	 */
	uint32_t encoder_increments;
	uint32_t motor_revolutions;
	/*
	 * 0x6098
	 * This object indicates the homing method to be used.
	 * The following value definitions are valid:
	 * -4 = homing on hard stop in positive direction with Index
	 * -3 = homing on hard stop in negative direction with Index
	 * -2 = homing on hard stop in positive direction
	 * -1 = homing on hard stop in negative direction
	 * 0 = no homing method assigned
	 * 1 = homing method 1 to be used
	 * .
	 * .
	 * 36 = homing method 36 to used
	 */
	int8_t homing_method;
	/*
	 * 0x6099
	 * This object indicates the commanded speeds used during homing procedure.
	 */
	uint32_t fast_homing_speed;
	uint32_t slow_homing_speed;
	/*
	 * 0x609A
	 * This object indicates the acceleration and deceleration to be used during homing operation.
	 */
	uint32_t homing_acceleration;
	/*
	 * 0x60C5
	 * This object indicates the maximum acceleration. It is used to limit the
	 * acceleration to an acceptable value in order to prevent the motor and
	 * the moved mechanics from being damaged.
	 */
	uint32_t max_acceleration;
	/*
	 * 0x60C6
	 * This object indicates the maximum deceleration. It is used to limit the
	 * deceleration to an acceptable value in order to prevent the motor and
	 * the moved mechanics from being damaged.
	 */
	uint32_t max_deceleration;
	/*
	 * 0x60F4
	 * This object indicates the actual value of the following error.
	 */
	int32_t following_error_actual;
	/*
	 * 0x60FA
	 * This object indicates the control effort as the output of the position
	 * control loop. In the position control function, notation of the control
	 * effort is mode-dependent and therefore not specified.
	 */
	int32_t control_effort;
	/*
	 * 0x60FD
	 * This object provides digital inputs.
	 * This object is organized bit-wise. The bits have the following meaning:
	 * bit 0: negative limit switch
	 * bit 1: positive limit switch
	 * bit 2: home switch
	 * bit 3: reserved
	 * bit 16-31: manufacturer-specific
	 * The bit values have the following meaning:
	 * 0 = switch is off
	 * 1 = switch is on
	 */
	uint32_t digital_inputs;
	/*
	 * 0x60FF
	 * This object indicates the configured target velocity and is used as
	 * input for the trajectory generator.
	 */
	int32_t target_velocity;
	/*
	 * 0x6402
	 * This object indicates the type of motor attached to and driven by the drive device.
	 * The following value definition is valid:
	 *  
	 *  0008h = stepper motor
	 *  0009h = micro-step stepper motor
	 */
	uint16_t motor_type;
	/*
	 * 0x6502
	 * This object provides information about the supported drive modes.
	 * This object is organized bit-wise. The bits have the following meaning:
	 * bit 0: profile position mode
	 * bit 1: velocity mode
	 * bit 2: profile velocity mode
	 * bit 3: profile torque mode
	 * bit 4: reserved
	 * bit 5: homing mode
	 * bit 6: interpolated position mode
	 * bit 7: cyclic synchronous position mode
	 * bit 8: cyclic synchronous velocity mode
	 * bit 9: cyclic synchronous torque mode
	 * bit 10-15: reserved
	 * bit 16-31: manufacturer-specific
	 *
	 *  The bit values have the following meaning:
	 *  0 = mode is not supported
	 *  1 = mode is supported
	 */
	uint32_t supported_modes;
};


