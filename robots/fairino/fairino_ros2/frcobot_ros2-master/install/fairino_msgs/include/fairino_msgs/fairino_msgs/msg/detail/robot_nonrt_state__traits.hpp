// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from fairino_msgs:msg/RobotNonrtState.idl
// generated code does not contain a copyright notice

#ifndef FAIRINO_MSGS__MSG__DETAIL__ROBOT_NONRT_STATE__TRAITS_HPP_
#define FAIRINO_MSGS__MSG__DETAIL__ROBOT_NONRT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "fairino_msgs/msg/detail/robot_nonrt_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace fairino_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotNonrtState & msg,
  std::ostream & out)
{
  out << "{";
  // member: j1_cur_pos
  {
    out << "j1_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j1_cur_pos, out);
    out << ", ";
  }

  // member: j2_cur_pos
  {
    out << "j2_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j2_cur_pos, out);
    out << ", ";
  }

  // member: j3_cur_pos
  {
    out << "j3_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j3_cur_pos, out);
    out << ", ";
  }

  // member: j4_cur_pos
  {
    out << "j4_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j4_cur_pos, out);
    out << ", ";
  }

  // member: j5_cur_pos
  {
    out << "j5_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j5_cur_pos, out);
    out << ", ";
  }

  // member: j6_cur_pos
  {
    out << "j6_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j6_cur_pos, out);
    out << ", ";
  }

  // member: j1_cur_tor
  {
    out << "j1_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j1_cur_tor, out);
    out << ", ";
  }

  // member: j2_cur_tor
  {
    out << "j2_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j2_cur_tor, out);
    out << ", ";
  }

  // member: j3_cur_tor
  {
    out << "j3_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j3_cur_tor, out);
    out << ", ";
  }

  // member: j4_cur_tor
  {
    out << "j4_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j4_cur_tor, out);
    out << ", ";
  }

  // member: j5_cur_tor
  {
    out << "j5_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j5_cur_tor, out);
    out << ", ";
  }

  // member: j6_cur_tor
  {
    out << "j6_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j6_cur_tor, out);
    out << ", ";
  }

  // member: cart_x_cur_pos
  {
    out << "cart_x_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_x_cur_pos, out);
    out << ", ";
  }

  // member: cart_y_cur_pos
  {
    out << "cart_y_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_y_cur_pos, out);
    out << ", ";
  }

  // member: cart_z_cur_pos
  {
    out << "cart_z_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_z_cur_pos, out);
    out << ", ";
  }

  // member: cart_a_cur_pos
  {
    out << "cart_a_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_a_cur_pos, out);
    out << ", ";
  }

  // member: cart_b_cur_pos
  {
    out << "cart_b_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_b_cur_pos, out);
    out << ", ";
  }

  // member: cart_c_cur_pos
  {
    out << "cart_c_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_c_cur_pos, out);
    out << ", ";
  }

  // member: flange_x_cur_pos
  {
    out << "flange_x_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_x_cur_pos, out);
    out << ", ";
  }

  // member: flange_y_cur_pos
  {
    out << "flange_y_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_y_cur_pos, out);
    out << ", ";
  }

  // member: flange_z_cur_pos
  {
    out << "flange_z_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_z_cur_pos, out);
    out << ", ";
  }

  // member: flange_a_cur_pos
  {
    out << "flange_a_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_a_cur_pos, out);
    out << ", ";
  }

  // member: flange_b_cur_pos
  {
    out << "flange_b_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_b_cur_pos, out);
    out << ", ";
  }

  // member: flange_c_cur_pos
  {
    out << "flange_c_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_c_cur_pos, out);
    out << ", ";
  }

  // member: exaxispos1
  {
    out << "exaxispos1: ";
    rosidl_generator_traits::value_to_yaml(msg.exaxispos1, out);
    out << ", ";
  }

  // member: exaxispos2
  {
    out << "exaxispos2: ";
    rosidl_generator_traits::value_to_yaml(msg.exaxispos2, out);
    out << ", ";
  }

  // member: exaxispos3
  {
    out << "exaxispos3: ";
    rosidl_generator_traits::value_to_yaml(msg.exaxispos3, out);
    out << ", ";
  }

  // member: exaxispos4
  {
    out << "exaxispos4: ";
    rosidl_generator_traits::value_to_yaml(msg.exaxispos4, out);
    out << ", ";
  }

  // member: ft_fx_data
  {
    out << "ft_fx_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_fx_data, out);
    out << ", ";
  }

  // member: ft_fy_data
  {
    out << "ft_fy_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_fy_data, out);
    out << ", ";
  }

  // member: ft_fz_data
  {
    out << "ft_fz_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_fz_data, out);
    out << ", ";
  }

  // member: ft_tx_data
  {
    out << "ft_tx_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_tx_data, out);
    out << ", ";
  }

  // member: ft_ty_data
  {
    out << "ft_ty_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_ty_data, out);
    out << ", ";
  }

  // member: ft_tz_data
  {
    out << "ft_tz_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_tz_data, out);
    out << ", ";
  }

  // member: ft_actstatus
  {
    out << "ft_actstatus: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_actstatus, out);
    out << ", ";
  }

  // member: robot_mode
  {
    out << "robot_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_mode, out);
    out << ", ";
  }

  // member: tool_num
  {
    out << "tool_num: ";
    rosidl_generator_traits::value_to_yaml(msg.tool_num, out);
    out << ", ";
  }

  // member: work_num
  {
    out << "work_num: ";
    rosidl_generator_traits::value_to_yaml(msg.work_num, out);
    out << ", ";
  }

  // member: prg_state
  {
    out << "prg_state: ";
    rosidl_generator_traits::value_to_yaml(msg.prg_state, out);
    out << ", ";
  }

  // member: abnormal_stop
  {
    out << "abnormal_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.abnormal_stop, out);
    out << ", ";
  }

  // member: prg_name
  {
    out << "prg_name: ";
    rosidl_generator_traits::value_to_yaml(msg.prg_name, out);
    out << ", ";
  }

  // member: prg_total_line
  {
    out << "prg_total_line: ";
    rosidl_generator_traits::value_to_yaml(msg.prg_total_line, out);
    out << ", ";
  }

  // member: prg_cur_line
  {
    out << "prg_cur_line: ";
    rosidl_generator_traits::value_to_yaml(msg.prg_cur_line, out);
    out << ", ";
  }

  // member: dgt_output_h
  {
    out << "dgt_output_h: ";
    rosidl_generator_traits::value_to_yaml(msg.dgt_output_h, out);
    out << ", ";
  }

  // member: dgt_output_l
  {
    out << "dgt_output_l: ";
    rosidl_generator_traits::value_to_yaml(msg.dgt_output_l, out);
    out << ", ";
  }

  // member: dgt_input_h
  {
    out << "dgt_input_h: ";
    rosidl_generator_traits::value_to_yaml(msg.dgt_input_h, out);
    out << ", ";
  }

  // member: dgt_input_l
  {
    out << "dgt_input_l: ";
    rosidl_generator_traits::value_to_yaml(msg.dgt_input_l, out);
    out << ", ";
  }

  // member: tl_dgt_output_l
  {
    out << "tl_dgt_output_l: ";
    rosidl_generator_traits::value_to_yaml(msg.tl_dgt_output_l, out);
    out << ", ";
  }

  // member: tl_dgt_input_l
  {
    out << "tl_dgt_input_l: ";
    rosidl_generator_traits::value_to_yaml(msg.tl_dgt_input_l, out);
    out << ", ";
  }

  // member: emg
  {
    out << "emg: ";
    rosidl_generator_traits::value_to_yaml(msg.emg, out);
    out << ", ";
  }

  // member: safetyboxsig
  {
    if (msg.safetyboxsig.size() == 0) {
      out << "safetyboxsig: []";
    } else {
      out << "safetyboxsig: [";
      size_t pending_items = msg.safetyboxsig.size();
      for (auto item : msg.safetyboxsig) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: robot_motion_done
  {
    out << "robot_motion_done: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_motion_done, out);
    out << ", ";
  }

  // member: grip_motion_done
  {
    out << "grip_motion_done: ";
    rosidl_generator_traits::value_to_yaml(msg.grip_motion_done, out);
    out << ", ";
  }

  // member: weldbreakoffstate
  {
    out << "weldbreakoffstate: ";
    rosidl_generator_traits::value_to_yaml(msg.weldbreakoffstate, out);
    out << ", ";
  }

  // member: weldarcstate
  {
    out << "weldarcstate: ";
    rosidl_generator_traits::value_to_yaml(msg.weldarcstate, out);
    out << ", ";
  }

  // member: welding_voltage
  {
    out << "welding_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.welding_voltage, out);
    out << ", ";
  }

  // member: welding_current
  {
    out << "welding_current: ";
    rosidl_generator_traits::value_to_yaml(msg.welding_current, out);
    out << ", ";
  }

  // member: weldtrackspeed
  {
    out << "weldtrackspeed: ";
    rosidl_generator_traits::value_to_yaml(msg.weldtrackspeed, out);
    out << ", ";
  }

  // member: main_error_code
  {
    out << "main_error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.main_error_code, out);
    out << ", ";
  }

  // member: sub_error_code
  {
    out << "sub_error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.sub_error_code, out);
    out << ", ";
  }

  // member: check_sum
  {
    out << "check_sum: ";
    rosidl_generator_traits::value_to_yaml(msg.check_sum, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: version
  {
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << ", ";
  }

  // member: tpd_exception
  {
    out << "tpd_exception: ";
    rosidl_generator_traits::value_to_yaml(msg.tpd_exception, out);
    out << ", ";
  }

  // member: alarm_reboot_robot
  {
    out << "alarm_reboot_robot: ";
    rosidl_generator_traits::value_to_yaml(msg.alarm_reboot_robot, out);
    out << ", ";
  }

  // member: modbusmasterconnectstate
  {
    out << "modbusmasterconnectstate: ";
    rosidl_generator_traits::value_to_yaml(msg.modbusmasterconnectstate, out);
    out << ", ";
  }

  // member: mdbsslaveconnect
  {
    out << "mdbsslaveconnect: ";
    rosidl_generator_traits::value_to_yaml(msg.mdbsslaveconnect, out);
    out << ", ";
  }

  // member: socket_conn_timeout
  {
    out << "socket_conn_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.socket_conn_timeout, out);
    out << ", ";
  }

  // member: socket_read_timeout
  {
    out << "socket_read_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.socket_read_timeout, out);
    out << ", ";
  }

  // member: btn_box_stop_signa
  {
    out << "btn_box_stop_signa: ";
    rosidl_generator_traits::value_to_yaml(msg.btn_box_stop_signa, out);
    out << ", ";
  }

  // member: strangeposflag
  {
    out << "strangeposflag: ";
    rosidl_generator_traits::value_to_yaml(msg.strangeposflag, out);
    out << ", ";
  }

  // member: drag_alarm
  {
    out << "drag_alarm: ";
    rosidl_generator_traits::value_to_yaml(msg.drag_alarm, out);
    out << ", ";
  }

  // member: alarm
  {
    out << "alarm: ";
    rosidl_generator_traits::value_to_yaml(msg.alarm, out);
    out << ", ";
  }

  // member: safetydoor_alarm
  {
    out << "safetydoor_alarm: ";
    rosidl_generator_traits::value_to_yaml(msg.safetydoor_alarm, out);
    out << ", ";
  }

  // member: safetyplanealarm
  {
    out << "safetyplanealarm: ";
    rosidl_generator_traits::value_to_yaml(msg.safetyplanealarm, out);
    out << ", ";
  }

  // member: motionalarm
  {
    out << "motionalarm: ";
    rosidl_generator_traits::value_to_yaml(msg.motionalarm, out);
    out << ", ";
  }

  // member: interferealarm
  {
    out << "interferealarm: ";
    rosidl_generator_traits::value_to_yaml(msg.interferealarm, out);
    out << ", ";
  }

  // member: endluaerrcode
  {
    out << "endluaerrcode: ";
    rosidl_generator_traits::value_to_yaml(msg.endluaerrcode, out);
    out << ", ";
  }

  // member: dr_alarm
  {
    out << "dr_alarm: ";
    rosidl_generator_traits::value_to_yaml(msg.dr_alarm, out);
    out << ", ";
  }

  // member: udpcmdstate
  {
    out << "udpcmdstate: ";
    rosidl_generator_traits::value_to_yaml(msg.udpcmdstate, out);
    out << ", ";
  }

  // member: aliveslavenumerror
  {
    out << "aliveslavenumerror: ";
    rosidl_generator_traits::value_to_yaml(msg.aliveslavenumerror, out);
    out << ", ";
  }

  // member: gripperfaultnum
  {
    out << "gripperfaultnum: ";
    rosidl_generator_traits::value_to_yaml(msg.gripperfaultnum, out);
    out << ", ";
  }

  // member: slavecomerror
  {
    if (msg.slavecomerror.size() == 0) {
      out << "slavecomerror: []";
    } else {
      out << "slavecomerror: [";
      size_t pending_items = msg.slavecomerror.size();
      for (auto item : msg.slavecomerror) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cmdpointerror
  {
    out << "cmdpointerror: ";
    rosidl_generator_traits::value_to_yaml(msg.cmdpointerror, out);
    out << ", ";
  }

  // member: ioerror
  {
    out << "ioerror: ";
    rosidl_generator_traits::value_to_yaml(msg.ioerror, out);
    out << ", ";
  }

  // member: grippererro
  {
    out << "grippererro: ";
    rosidl_generator_traits::value_to_yaml(msg.grippererro, out);
    out << ", ";
  }

  // member: fileerror
  {
    out << "fileerror: ";
    rosidl_generator_traits::value_to_yaml(msg.fileerror, out);
    out << ", ";
  }

  // member: paraerror
  {
    out << "paraerror: ";
    rosidl_generator_traits::value_to_yaml(msg.paraerror, out);
    out << ", ";
  }

  // member: exaxis_out_slimit_error
  {
    out << "exaxis_out_slimit_error: ";
    rosidl_generator_traits::value_to_yaml(msg.exaxis_out_slimit_error, out);
    out << ", ";
  }

  // member: dr_com_err
  {
    if (msg.dr_com_err.size() == 0) {
      out << "dr_com_err: []";
    } else {
      out << "dr_com_err: [";
      size_t pending_items = msg.dr_com_err.size();
      for (auto item : msg.dr_com_err) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: dr_err
  {
    out << "dr_err: ";
    rosidl_generator_traits::value_to_yaml(msg.dr_err, out);
    out << ", ";
  }

  // member: out_sflimit_err
  {
    out << "out_sflimit_err: ";
    rosidl_generator_traits::value_to_yaml(msg.out_sflimit_err, out);
    out << ", ";
  }

  // member: collision_err
  {
    out << "collision_err: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_err, out);
    out << ", ";
  }

  // member: weld_readystate
  {
    out << "weld_readystate: ";
    rosidl_generator_traits::value_to_yaml(msg.weld_readystate, out);
    out << ", ";
  }

  // member: alarm_check_emerg_stop_btn
  {
    out << "alarm_check_emerg_stop_btn: ";
    rosidl_generator_traits::value_to_yaml(msg.alarm_check_emerg_stop_btn, out);
    out << ", ";
  }

  // member: ts_web_state_com_error
  {
    out << "ts_web_state_com_error: ";
    rosidl_generator_traits::value_to_yaml(msg.ts_web_state_com_error, out);
    out << ", ";
  }

  // member: ts_tm_cmd_com_error
  {
    out << "ts_tm_cmd_com_error: ";
    rosidl_generator_traits::value_to_yaml(msg.ts_tm_cmd_com_error, out);
    out << ", ";
  }

  // member: ts_tm_state_com_error
  {
    out << "ts_tm_state_com_error: ";
    rosidl_generator_traits::value_to_yaml(msg.ts_tm_state_com_error, out);
    out << ", ";
  }

  // member: ctrlboxerror
  {
    out << "ctrlboxerror: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrlboxerror, out);
    out << ", ";
  }

  // member: safety_data_state
  {
    out << "safety_data_state: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_data_state, out);
    out << ", ";
  }

  // member: forcesensorerrstate
  {
    out << "forcesensorerrstate: ";
    rosidl_generator_traits::value_to_yaml(msg.forcesensorerrstate, out);
    out << ", ";
  }

  // member: ctrlopenluaerrcode
  {
    if (msg.ctrlopenluaerrcode.size() == 0) {
      out << "ctrlopenluaerrcode: []";
    } else {
      out << "ctrlopenluaerrcode: [";
      size_t pending_items = msg.ctrlopenluaerrcode.size();
      for (auto item : msg.ctrlopenluaerrcode) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotNonrtState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: j1_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j1_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j1_cur_pos, out);
    out << "\n";
  }

  // member: j2_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j2_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j2_cur_pos, out);
    out << "\n";
  }

  // member: j3_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j3_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j3_cur_pos, out);
    out << "\n";
  }

  // member: j4_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j4_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j4_cur_pos, out);
    out << "\n";
  }

  // member: j5_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j5_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j5_cur_pos, out);
    out << "\n";
  }

  // member: j6_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j6_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.j6_cur_pos, out);
    out << "\n";
  }

  // member: j1_cur_tor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j1_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j1_cur_tor, out);
    out << "\n";
  }

  // member: j2_cur_tor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j2_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j2_cur_tor, out);
    out << "\n";
  }

  // member: j3_cur_tor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j3_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j3_cur_tor, out);
    out << "\n";
  }

  // member: j4_cur_tor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j4_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j4_cur_tor, out);
    out << "\n";
  }

  // member: j5_cur_tor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j5_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j5_cur_tor, out);
    out << "\n";
  }

  // member: j6_cur_tor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j6_cur_tor: ";
    rosidl_generator_traits::value_to_yaml(msg.j6_cur_tor, out);
    out << "\n";
  }

  // member: cart_x_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cart_x_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_x_cur_pos, out);
    out << "\n";
  }

  // member: cart_y_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cart_y_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_y_cur_pos, out);
    out << "\n";
  }

  // member: cart_z_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cart_z_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_z_cur_pos, out);
    out << "\n";
  }

  // member: cart_a_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cart_a_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_a_cur_pos, out);
    out << "\n";
  }

  // member: cart_b_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cart_b_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_b_cur_pos, out);
    out << "\n";
  }

  // member: cart_c_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cart_c_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_c_cur_pos, out);
    out << "\n";
  }

  // member: flange_x_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flange_x_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_x_cur_pos, out);
    out << "\n";
  }

  // member: flange_y_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flange_y_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_y_cur_pos, out);
    out << "\n";
  }

  // member: flange_z_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flange_z_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_z_cur_pos, out);
    out << "\n";
  }

  // member: flange_a_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flange_a_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_a_cur_pos, out);
    out << "\n";
  }

  // member: flange_b_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flange_b_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_b_cur_pos, out);
    out << "\n";
  }

  // member: flange_c_cur_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flange_c_cur_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.flange_c_cur_pos, out);
    out << "\n";
  }

  // member: exaxispos1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "exaxispos1: ";
    rosidl_generator_traits::value_to_yaml(msg.exaxispos1, out);
    out << "\n";
  }

  // member: exaxispos2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "exaxispos2: ";
    rosidl_generator_traits::value_to_yaml(msg.exaxispos2, out);
    out << "\n";
  }

  // member: exaxispos3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "exaxispos3: ";
    rosidl_generator_traits::value_to_yaml(msg.exaxispos3, out);
    out << "\n";
  }

  // member: exaxispos4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "exaxispos4: ";
    rosidl_generator_traits::value_to_yaml(msg.exaxispos4, out);
    out << "\n";
  }

  // member: ft_fx_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ft_fx_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_fx_data, out);
    out << "\n";
  }

  // member: ft_fy_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ft_fy_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_fy_data, out);
    out << "\n";
  }

  // member: ft_fz_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ft_fz_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_fz_data, out);
    out << "\n";
  }

  // member: ft_tx_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ft_tx_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_tx_data, out);
    out << "\n";
  }

  // member: ft_ty_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ft_ty_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_ty_data, out);
    out << "\n";
  }

  // member: ft_tz_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ft_tz_data: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_tz_data, out);
    out << "\n";
  }

  // member: ft_actstatus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ft_actstatus: ";
    rosidl_generator_traits::value_to_yaml(msg.ft_actstatus, out);
    out << "\n";
  }

  // member: robot_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_mode, out);
    out << "\n";
  }

  // member: tool_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tool_num: ";
    rosidl_generator_traits::value_to_yaml(msg.tool_num, out);
    out << "\n";
  }

  // member: work_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "work_num: ";
    rosidl_generator_traits::value_to_yaml(msg.work_num, out);
    out << "\n";
  }

  // member: prg_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "prg_state: ";
    rosidl_generator_traits::value_to_yaml(msg.prg_state, out);
    out << "\n";
  }

  // member: abnormal_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "abnormal_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.abnormal_stop, out);
    out << "\n";
  }

  // member: prg_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "prg_name: ";
    rosidl_generator_traits::value_to_yaml(msg.prg_name, out);
    out << "\n";
  }

  // member: prg_total_line
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "prg_total_line: ";
    rosidl_generator_traits::value_to_yaml(msg.prg_total_line, out);
    out << "\n";
  }

  // member: prg_cur_line
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "prg_cur_line: ";
    rosidl_generator_traits::value_to_yaml(msg.prg_cur_line, out);
    out << "\n";
  }

  // member: dgt_output_h
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dgt_output_h: ";
    rosidl_generator_traits::value_to_yaml(msg.dgt_output_h, out);
    out << "\n";
  }

  // member: dgt_output_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dgt_output_l: ";
    rosidl_generator_traits::value_to_yaml(msg.dgt_output_l, out);
    out << "\n";
  }

  // member: dgt_input_h
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dgt_input_h: ";
    rosidl_generator_traits::value_to_yaml(msg.dgt_input_h, out);
    out << "\n";
  }

  // member: dgt_input_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dgt_input_l: ";
    rosidl_generator_traits::value_to_yaml(msg.dgt_input_l, out);
    out << "\n";
  }

  // member: tl_dgt_output_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tl_dgt_output_l: ";
    rosidl_generator_traits::value_to_yaml(msg.tl_dgt_output_l, out);
    out << "\n";
  }

  // member: tl_dgt_input_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tl_dgt_input_l: ";
    rosidl_generator_traits::value_to_yaml(msg.tl_dgt_input_l, out);
    out << "\n";
  }

  // member: emg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emg: ";
    rosidl_generator_traits::value_to_yaml(msg.emg, out);
    out << "\n";
  }

  // member: safetyboxsig
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.safetyboxsig.size() == 0) {
      out << "safetyboxsig: []\n";
    } else {
      out << "safetyboxsig:\n";
      for (auto item : msg.safetyboxsig) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: robot_motion_done
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_motion_done: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_motion_done, out);
    out << "\n";
  }

  // member: grip_motion_done
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grip_motion_done: ";
    rosidl_generator_traits::value_to_yaml(msg.grip_motion_done, out);
    out << "\n";
  }

  // member: weldbreakoffstate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "weldbreakoffstate: ";
    rosidl_generator_traits::value_to_yaml(msg.weldbreakoffstate, out);
    out << "\n";
  }

  // member: weldarcstate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "weldarcstate: ";
    rosidl_generator_traits::value_to_yaml(msg.weldarcstate, out);
    out << "\n";
  }

  // member: welding_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "welding_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.welding_voltage, out);
    out << "\n";
  }

  // member: welding_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "welding_current: ";
    rosidl_generator_traits::value_to_yaml(msg.welding_current, out);
    out << "\n";
  }

  // member: weldtrackspeed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "weldtrackspeed: ";
    rosidl_generator_traits::value_to_yaml(msg.weldtrackspeed, out);
    out << "\n";
  }

  // member: main_error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "main_error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.main_error_code, out);
    out << "\n";
  }

  // member: sub_error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sub_error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.sub_error_code, out);
    out << "\n";
  }

  // member: check_sum
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "check_sum: ";
    rosidl_generator_traits::value_to_yaml(msg.check_sum, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << "\n";
  }

  // member: tpd_exception
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tpd_exception: ";
    rosidl_generator_traits::value_to_yaml(msg.tpd_exception, out);
    out << "\n";
  }

  // member: alarm_reboot_robot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alarm_reboot_robot: ";
    rosidl_generator_traits::value_to_yaml(msg.alarm_reboot_robot, out);
    out << "\n";
  }

  // member: modbusmasterconnectstate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "modbusmasterconnectstate: ";
    rosidl_generator_traits::value_to_yaml(msg.modbusmasterconnectstate, out);
    out << "\n";
  }

  // member: mdbsslaveconnect
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mdbsslaveconnect: ";
    rosidl_generator_traits::value_to_yaml(msg.mdbsslaveconnect, out);
    out << "\n";
  }

  // member: socket_conn_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "socket_conn_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.socket_conn_timeout, out);
    out << "\n";
  }

  // member: socket_read_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "socket_read_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.socket_read_timeout, out);
    out << "\n";
  }

  // member: btn_box_stop_signa
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "btn_box_stop_signa: ";
    rosidl_generator_traits::value_to_yaml(msg.btn_box_stop_signa, out);
    out << "\n";
  }

  // member: strangeposflag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "strangeposflag: ";
    rosidl_generator_traits::value_to_yaml(msg.strangeposflag, out);
    out << "\n";
  }

  // member: drag_alarm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drag_alarm: ";
    rosidl_generator_traits::value_to_yaml(msg.drag_alarm, out);
    out << "\n";
  }

  // member: alarm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alarm: ";
    rosidl_generator_traits::value_to_yaml(msg.alarm, out);
    out << "\n";
  }

  // member: safetydoor_alarm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safetydoor_alarm: ";
    rosidl_generator_traits::value_to_yaml(msg.safetydoor_alarm, out);
    out << "\n";
  }

  // member: safetyplanealarm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safetyplanealarm: ";
    rosidl_generator_traits::value_to_yaml(msg.safetyplanealarm, out);
    out << "\n";
  }

  // member: motionalarm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motionalarm: ";
    rosidl_generator_traits::value_to_yaml(msg.motionalarm, out);
    out << "\n";
  }

  // member: interferealarm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "interferealarm: ";
    rosidl_generator_traits::value_to_yaml(msg.interferealarm, out);
    out << "\n";
  }

  // member: endluaerrcode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "endluaerrcode: ";
    rosidl_generator_traits::value_to_yaml(msg.endluaerrcode, out);
    out << "\n";
  }

  // member: dr_alarm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dr_alarm: ";
    rosidl_generator_traits::value_to_yaml(msg.dr_alarm, out);
    out << "\n";
  }

  // member: udpcmdstate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "udpcmdstate: ";
    rosidl_generator_traits::value_to_yaml(msg.udpcmdstate, out);
    out << "\n";
  }

  // member: aliveslavenumerror
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aliveslavenumerror: ";
    rosidl_generator_traits::value_to_yaml(msg.aliveslavenumerror, out);
    out << "\n";
  }

  // member: gripperfaultnum
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripperfaultnum: ";
    rosidl_generator_traits::value_to_yaml(msg.gripperfaultnum, out);
    out << "\n";
  }

  // member: slavecomerror
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.slavecomerror.size() == 0) {
      out << "slavecomerror: []\n";
    } else {
      out << "slavecomerror:\n";
      for (auto item : msg.slavecomerror) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: cmdpointerror
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmdpointerror: ";
    rosidl_generator_traits::value_to_yaml(msg.cmdpointerror, out);
    out << "\n";
  }

  // member: ioerror
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ioerror: ";
    rosidl_generator_traits::value_to_yaml(msg.ioerror, out);
    out << "\n";
  }

  // member: grippererro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grippererro: ";
    rosidl_generator_traits::value_to_yaml(msg.grippererro, out);
    out << "\n";
  }

  // member: fileerror
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fileerror: ";
    rosidl_generator_traits::value_to_yaml(msg.fileerror, out);
    out << "\n";
  }

  // member: paraerror
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "paraerror: ";
    rosidl_generator_traits::value_to_yaml(msg.paraerror, out);
    out << "\n";
  }

  // member: exaxis_out_slimit_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "exaxis_out_slimit_error: ";
    rosidl_generator_traits::value_to_yaml(msg.exaxis_out_slimit_error, out);
    out << "\n";
  }

  // member: dr_com_err
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.dr_com_err.size() == 0) {
      out << "dr_com_err: []\n";
    } else {
      out << "dr_com_err:\n";
      for (auto item : msg.dr_com_err) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: dr_err
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dr_err: ";
    rosidl_generator_traits::value_to_yaml(msg.dr_err, out);
    out << "\n";
  }

  // member: out_sflimit_err
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "out_sflimit_err: ";
    rosidl_generator_traits::value_to_yaml(msg.out_sflimit_err, out);
    out << "\n";
  }

  // member: collision_err
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision_err: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_err, out);
    out << "\n";
  }

  // member: weld_readystate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "weld_readystate: ";
    rosidl_generator_traits::value_to_yaml(msg.weld_readystate, out);
    out << "\n";
  }

  // member: alarm_check_emerg_stop_btn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alarm_check_emerg_stop_btn: ";
    rosidl_generator_traits::value_to_yaml(msg.alarm_check_emerg_stop_btn, out);
    out << "\n";
  }

  // member: ts_web_state_com_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ts_web_state_com_error: ";
    rosidl_generator_traits::value_to_yaml(msg.ts_web_state_com_error, out);
    out << "\n";
  }

  // member: ts_tm_cmd_com_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ts_tm_cmd_com_error: ";
    rosidl_generator_traits::value_to_yaml(msg.ts_tm_cmd_com_error, out);
    out << "\n";
  }

  // member: ts_tm_state_com_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ts_tm_state_com_error: ";
    rosidl_generator_traits::value_to_yaml(msg.ts_tm_state_com_error, out);
    out << "\n";
  }

  // member: ctrlboxerror
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ctrlboxerror: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrlboxerror, out);
    out << "\n";
  }

  // member: safety_data_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_data_state: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_data_state, out);
    out << "\n";
  }

  // member: forcesensorerrstate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "forcesensorerrstate: ";
    rosidl_generator_traits::value_to_yaml(msg.forcesensorerrstate, out);
    out << "\n";
  }

  // member: ctrlopenluaerrcode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ctrlopenluaerrcode.size() == 0) {
      out << "ctrlopenluaerrcode: []\n";
    } else {
      out << "ctrlopenluaerrcode:\n";
      for (auto item : msg.ctrlopenluaerrcode) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotNonrtState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace fairino_msgs

namespace rosidl_generator_traits
{

[[deprecated("use fairino_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const fairino_msgs::msg::RobotNonrtState & msg,
  std::ostream & out, size_t indentation = 0)
{
  fairino_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use fairino_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const fairino_msgs::msg::RobotNonrtState & msg)
{
  return fairino_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<fairino_msgs::msg::RobotNonrtState>()
{
  return "fairino_msgs::msg::RobotNonrtState";
}

template<>
inline const char * name<fairino_msgs::msg::RobotNonrtState>()
{
  return "fairino_msgs/msg/RobotNonrtState";
}

template<>
struct has_fixed_size<fairino_msgs::msg::RobotNonrtState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<fairino_msgs::msg::RobotNonrtState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<fairino_msgs::msg::RobotNonrtState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FAIRINO_MSGS__MSG__DETAIL__ROBOT_NONRT_STATE__TRAITS_HPP_
