package com.example.motor_control_app.motors.dynamixel

/**
 * XL430-W250 control table addresses (Protocol 2.0).
 * Source: reference_repo/DynamixelSDK/control_table/xl430_w250.model
 */
object Xl430Registers {
    const val ADDR_OPERATING_MODE = 11 // 1 byte
    const val ADDR_TORQUE_ENABLE = 64 // 1 byte
    const val ADDR_STATUS_RETURN_LEVEL = 68 // 1 byte
    const val ADDR_HARDWARE_ERROR_STATUS = 70 // 1 byte

    const val ADDR_GOAL_POSITION = 116 // 4 bytes (signed in extended position mode)
    const val ADDR_PRESENT_VELOCITY = 128 // 4 bytes (signed)
    const val ADDR_PRESENT_POSITION = 132 // 4 bytes (signed)
    const val ADDR_PRESENT_INPUT_VOLTAGE = 144 // 2 bytes (0.1V units)
    const val ADDR_PRESENT_TEMPERATURE = 146 // 1 byte (degC)

    // Operating mode values (X-series)
    const val MODE_POSITION_CONTROL = 3
    const val MODE_EXTENDED_POSITION_CONTROL = 4
}

