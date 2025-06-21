#ifndef COMMANDS_H_
#define COMMANDS_H_

// This file defines the command values shared between the Central and Peripheral devices.
// Both sketches must include this file to ensure they agree on what each command means.
// The values are defined as single bytes (uint8_t).

#define CMD_NONE          0x00 // No command or default state
#define CMD_FLEX_CAL      0x01 // Command to start flex sensor calibration
#define CMD_FULL_CAL      0x02 // Command to start full calibration (flex + any future IMU cal)
#define CMD_RESET_TIME    0x03 // Command to reset the millis() timestamp baseline
#define CMD_RESET_DEVICE  0x04 // Command for the peripheral to restart itself

#endif // COMMANDS_H_