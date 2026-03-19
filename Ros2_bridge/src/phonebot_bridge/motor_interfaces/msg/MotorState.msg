std_msgs/Header header

# IDs for each motor entry. Arrays below match this order.
uint8[] ids

# Converted values (not raw registers).
float32[] present_pwm_percent
float32[] present_load_percent
float32[] present_velocity_rad_s
float32[] present_position_rad
float32[] velocity_trajectory_rad_s
float32[] position_trajectory_rad
float32[] present_input_voltage_v
float32[] present_temperature_c

