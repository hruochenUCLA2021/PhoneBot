package com.example.phonebot_app_android.motors.dynamixel

/**
 * XL430-W250 control table addresses (Protocol 2.0).
 * Source: reference_repo/DynamixelSDK/control_table/xl430_w250.model
 */
object Xl430Registers {
    const val ADDR_OPERATING_MODE = 11 // 1 byte
    const val ADDR_TORQUE_ENABLE = 64 // 1 byte
    const val ADDR_STATUS_RETURN_LEVEL = 68 // 1 byte

    const val ADDR_POSITION_D_GAIN = 80 // 2 bytes
    const val ADDR_POSITION_I_GAIN = 82 // 2 bytes
    const val ADDR_POSITION_P_GAIN = 84 // 2 bytes

    const val ADDR_GOAL_POSITION = 116 // 4 bytes

    const val ADDR_PRESENT_VELOCITY = 128 // 4 bytes (signed)
    const val ADDR_PRESENT_POSITION = 132 // 4 bytes (signed? for extended modes; for XL430 in position mode it's 0..4095)

    // Operating mode values (common across many X-series)
    const val MODE_POSITION_CONTROL = 3
}


