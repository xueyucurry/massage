// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from fairino_msgs:msg/RobotNonrtState.idl
// generated code does not contain a copyright notice

#ifndef FAIRINO_MSGS__MSG__DETAIL__ROBOT_NONRT_STATE__STRUCT_H_
#define FAIRINO_MSGS__MSG__DETAIL__ROBOT_NONRT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'prg_name'
// Member 'version'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobotNonrtState in the package fairino_msgs.
typedef struct fairino_msgs__msg__RobotNonrtState
{
  double j1_cur_pos;
  double j2_cur_pos;
  double j3_cur_pos;
  double j4_cur_pos;
  double j5_cur_pos;
  double j6_cur_pos;
  double j1_cur_tor;
  double j2_cur_tor;
  double j3_cur_tor;
  double j4_cur_tor;
  double j5_cur_tor;
  double j6_cur_tor;
  double cart_x_cur_pos;
  double cart_y_cur_pos;
  double cart_z_cur_pos;
  double cart_a_cur_pos;
  double cart_b_cur_pos;
  double cart_c_cur_pos;
  double flange_x_cur_pos;
  double flange_y_cur_pos;
  double flange_z_cur_pos;
  double flange_a_cur_pos;
  double flange_b_cur_pos;
  double flange_c_cur_pos;
  double exaxispos1;
  double exaxispos2;
  double exaxispos3;
  double exaxispos4;
  double ft_fx_data;
  double ft_fy_data;
  double ft_fz_data;
  double ft_tx_data;
  double ft_ty_data;
  double ft_tz_data;
  uint8_t ft_actstatus;
  uint8_t robot_mode;
  uint8_t tool_num;
  uint8_t work_num;
  uint8_t prg_state;
  uint8_t abnormal_stop;
  rosidl_runtime_c__String prg_name;
  uint8_t prg_total_line;
  uint8_t prg_cur_line;
  uint8_t dgt_output_h;
  uint8_t dgt_output_l;
  uint8_t dgt_input_h;
  uint8_t dgt_input_l;
  uint8_t tl_dgt_output_l;
  uint8_t tl_dgt_input_l;
  uint8_t emg;
  /// V2.1 added
  uint8_t safetyboxsig[6];
  uint8_t robot_motion_done;
  uint8_t grip_motion_done;
  uint8_t weldbreakoffstate;
  uint8_t weldarcstate;
  double welding_voltage;
  double welding_current;
  double weldtrackspeed;
  uint32_t main_error_code;
  uint32_t sub_error_code;
  uint8_t check_sum;
  uint64_t timestamp;
  rosidl_runtime_c__String version;
  uint8_t tpd_exception;
  uint8_t alarm_reboot_robot;
  uint8_t modbusmasterconnectstate;
  uint8_t mdbsslaveconnect;
  uint8_t socket_conn_timeout;
  uint8_t socket_read_timeout;
  uint8_t btn_box_stop_signa;
  uint8_t strangeposflag;
  uint8_t drag_alarm;
  uint8_t alarm;
  uint8_t safetydoor_alarm;
  uint8_t safetyplanealarm;
  uint8_t motionalarm;
  uint8_t interferealarm;
  uint16_t endluaerrcode;
  double dr_alarm;
  uint16_t udpcmdstate;
  uint8_t aliveslavenumerror;
  uint16_t gripperfaultnum;
  uint8_t slavecomerror[8];
  uint8_t cmdpointerror;
  uint8_t ioerror;
  uint8_t grippererro;
  uint8_t fileerror;
  uint8_t paraerror;
  uint8_t exaxis_out_slimit_error;
  uint8_t dr_com_err[6];
  double dr_err;
  double out_sflimit_err;
  double collision_err;
  uint8_t weld_readystate;
  uint8_t alarm_check_emerg_stop_btn;
  uint8_t ts_web_state_com_error;
  uint8_t ts_tm_cmd_com_error;
  uint8_t ts_tm_state_com_error;
  uint16_t ctrlboxerror;
  uint8_t safety_data_state;
  uint8_t forcesensorerrstate;
  uint8_t ctrlopenluaerrcode[4];
} fairino_msgs__msg__RobotNonrtState;

// Struct for a sequence of fairino_msgs__msg__RobotNonrtState.
typedef struct fairino_msgs__msg__RobotNonrtState__Sequence
{
  fairino_msgs__msg__RobotNonrtState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} fairino_msgs__msg__RobotNonrtState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FAIRINO_MSGS__MSG__DETAIL__ROBOT_NONRT_STATE__STRUCT_H_
