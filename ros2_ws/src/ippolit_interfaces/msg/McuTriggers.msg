# Full bit-level decode of the MCU Triggers frame (type 0x00, 7 bytes), replacing the JSON String on
# /mcu/triggers. Field list ported 1:1 from mcu_node.py's _TRIGGERS_BOOL_BITS (github.com/dimik/
# dreame_mcu_protocol RE, cross-checked live 2026-07-11/12). /cliff, /bumper, /wheel_floating stay
# separate Bool topics (already-proven consumers); this message is the full diagnostic/raw view.
std_msgs/Header header

bool key1
bool key2
bool key3
bool key4
bool left_bumper
bool right_bumper
bool left_wheel_floating
bool right_wheel_floating
bool d_view_lf
bool d_view_lmf
bool d_view_rmf
bool d_view_rf
bool d_view_lb
bool d_view_rb
bool mag_signal_left
bool mag_signal_right

# 3-bit dock-IR-beacon signal strength/channel codes (MSB-first), 0-7
uint8 ir_dock_lf
uint8 ir_dock_lmf
uint8 ir_dock_rmf
uint8 ir_dock_rf
bool ir_field_lf
bool ir_field_lmf
bool ir_field_rmf
bool ir_field_rf

bool dock_sta
bool lds_button1
bool lds_button2
bool side_error
bool roll_error
bool pump_error
bool side_overcurrent
bool roll_overcurrent
bool fan_overcurrent
bool pump_overcurrent
bool left_wheel_overcurrent
bool right_wheel_overcurrent
bool lidar_error
bool fan_error
bool left_vel_error
bool right_vel_error
bool left_mag_error
bool right_mag_error
bool imu_error
bool charge_error
