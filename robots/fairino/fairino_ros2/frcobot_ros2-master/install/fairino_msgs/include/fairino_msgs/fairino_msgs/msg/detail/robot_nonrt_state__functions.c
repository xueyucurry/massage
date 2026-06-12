// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from fairino_msgs:msg/RobotNonrtState.idl
// generated code does not contain a copyright notice
#include "fairino_msgs/msg/detail/robot_nonrt_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `prg_name`
// Member `version`
#include "rosidl_runtime_c/string_functions.h"

bool
fairino_msgs__msg__RobotNonrtState__init(fairino_msgs__msg__RobotNonrtState * msg)
{
  if (!msg) {
    return false;
  }
  // j1_cur_pos
  // j2_cur_pos
  // j3_cur_pos
  // j4_cur_pos
  // j5_cur_pos
  // j6_cur_pos
  // j1_cur_tor
  // j2_cur_tor
  // j3_cur_tor
  // j4_cur_tor
  // j5_cur_tor
  // j6_cur_tor
  // cart_x_cur_pos
  // cart_y_cur_pos
  // cart_z_cur_pos
  // cart_a_cur_pos
  // cart_b_cur_pos
  // cart_c_cur_pos
  // flange_x_cur_pos
  // flange_y_cur_pos
  // flange_z_cur_pos
  // flange_a_cur_pos
  // flange_b_cur_pos
  // flange_c_cur_pos
  // exaxispos1
  // exaxispos2
  // exaxispos3
  // exaxispos4
  // ft_fx_data
  // ft_fy_data
  // ft_fz_data
  // ft_tx_data
  // ft_ty_data
  // ft_tz_data
  // ft_actstatus
  // robot_mode
  // tool_num
  // work_num
  // prg_state
  // abnormal_stop
  // prg_name
  if (!rosidl_runtime_c__String__init(&msg->prg_name)) {
    fairino_msgs__msg__RobotNonrtState__fini(msg);
    return false;
  }
  // prg_total_line
  // prg_cur_line
  // dgt_output_h
  // dgt_output_l
  // dgt_input_h
  // dgt_input_l
  // tl_dgt_output_l
  // tl_dgt_input_l
  // emg
  // safetyboxsig
  // robot_motion_done
  // grip_motion_done
  // weldbreakoffstate
  // weldarcstate
  // welding_voltage
  // welding_current
  // weldtrackspeed
  // main_error_code
  // sub_error_code
  // check_sum
  // timestamp
  // version
  if (!rosidl_runtime_c__String__init(&msg->version)) {
    fairino_msgs__msg__RobotNonrtState__fini(msg);
    return false;
  }
  // tpd_exception
  // alarm_reboot_robot
  // modbusmasterconnectstate
  // mdbsslaveconnect
  // socket_conn_timeout
  // socket_read_timeout
  // btn_box_stop_signa
  // strangeposflag
  // drag_alarm
  // alarm
  // safetydoor_alarm
  // safetyplanealarm
  // motionalarm
  // interferealarm
  // endluaerrcode
  // dr_alarm
  // udpcmdstate
  // aliveslavenumerror
  // gripperfaultnum
  // slavecomerror
  // cmdpointerror
  // ioerror
  // grippererro
  // fileerror
  // paraerror
  // exaxis_out_slimit_error
  // dr_com_err
  // dr_err
  // out_sflimit_err
  // collision_err
  // weld_readystate
  // alarm_check_emerg_stop_btn
  // ts_web_state_com_error
  // ts_tm_cmd_com_error
  // ts_tm_state_com_error
  // ctrlboxerror
  // safety_data_state
  // forcesensorerrstate
  // ctrlopenluaerrcode
  return true;
}

void
fairino_msgs__msg__RobotNonrtState__fini(fairino_msgs__msg__RobotNonrtState * msg)
{
  if (!msg) {
    return;
  }
  // j1_cur_pos
  // j2_cur_pos
  // j3_cur_pos
  // j4_cur_pos
  // j5_cur_pos
  // j6_cur_pos
  // j1_cur_tor
  // j2_cur_tor
  // j3_cur_tor
  // j4_cur_tor
  // j5_cur_tor
  // j6_cur_tor
  // cart_x_cur_pos
  // cart_y_cur_pos
  // cart_z_cur_pos
  // cart_a_cur_pos
  // cart_b_cur_pos
  // cart_c_cur_pos
  // flange_x_cur_pos
  // flange_y_cur_pos
  // flange_z_cur_pos
  // flange_a_cur_pos
  // flange_b_cur_pos
  // flange_c_cur_pos
  // exaxispos1
  // exaxispos2
  // exaxispos3
  // exaxispos4
  // ft_fx_data
  // ft_fy_data
  // ft_fz_data
  // ft_tx_data
  // ft_ty_data
  // ft_tz_data
  // ft_actstatus
  // robot_mode
  // tool_num
  // work_num
  // prg_state
  // abnormal_stop
  // prg_name
  rosidl_runtime_c__String__fini(&msg->prg_name);
  // prg_total_line
  // prg_cur_line
  // dgt_output_h
  // dgt_output_l
  // dgt_input_h
  // dgt_input_l
  // tl_dgt_output_l
  // tl_dgt_input_l
  // emg
  // safetyboxsig
  // robot_motion_done
  // grip_motion_done
  // weldbreakoffstate
  // weldarcstate
  // welding_voltage
  // welding_current
  // weldtrackspeed
  // main_error_code
  // sub_error_code
  // check_sum
  // timestamp
  // version
  rosidl_runtime_c__String__fini(&msg->version);
  // tpd_exception
  // alarm_reboot_robot
  // modbusmasterconnectstate
  // mdbsslaveconnect
  // socket_conn_timeout
  // socket_read_timeout
  // btn_box_stop_signa
  // strangeposflag
  // drag_alarm
  // alarm
  // safetydoor_alarm
  // safetyplanealarm
  // motionalarm
  // interferealarm
  // endluaerrcode
  // dr_alarm
  // udpcmdstate
  // aliveslavenumerror
  // gripperfaultnum
  // slavecomerror
  // cmdpointerror
  // ioerror
  // grippererro
  // fileerror
  // paraerror
  // exaxis_out_slimit_error
  // dr_com_err
  // dr_err
  // out_sflimit_err
  // collision_err
  // weld_readystate
  // alarm_check_emerg_stop_btn
  // ts_web_state_com_error
  // ts_tm_cmd_com_error
  // ts_tm_state_com_error
  // ctrlboxerror
  // safety_data_state
  // forcesensorerrstate
  // ctrlopenluaerrcode
}

bool
fairino_msgs__msg__RobotNonrtState__are_equal(const fairino_msgs__msg__RobotNonrtState * lhs, const fairino_msgs__msg__RobotNonrtState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // j1_cur_pos
  if (lhs->j1_cur_pos != rhs->j1_cur_pos) {
    return false;
  }
  // j2_cur_pos
  if (lhs->j2_cur_pos != rhs->j2_cur_pos) {
    return false;
  }
  // j3_cur_pos
  if (lhs->j3_cur_pos != rhs->j3_cur_pos) {
    return false;
  }
  // j4_cur_pos
  if (lhs->j4_cur_pos != rhs->j4_cur_pos) {
    return false;
  }
  // j5_cur_pos
  if (lhs->j5_cur_pos != rhs->j5_cur_pos) {
    return false;
  }
  // j6_cur_pos
  if (lhs->j6_cur_pos != rhs->j6_cur_pos) {
    return false;
  }
  // j1_cur_tor
  if (lhs->j1_cur_tor != rhs->j1_cur_tor) {
    return false;
  }
  // j2_cur_tor
  if (lhs->j2_cur_tor != rhs->j2_cur_tor) {
    return false;
  }
  // j3_cur_tor
  if (lhs->j3_cur_tor != rhs->j3_cur_tor) {
    return false;
  }
  // j4_cur_tor
  if (lhs->j4_cur_tor != rhs->j4_cur_tor) {
    return false;
  }
  // j5_cur_tor
  if (lhs->j5_cur_tor != rhs->j5_cur_tor) {
    return false;
  }
  // j6_cur_tor
  if (lhs->j6_cur_tor != rhs->j6_cur_tor) {
    return false;
  }
  // cart_x_cur_pos
  if (lhs->cart_x_cur_pos != rhs->cart_x_cur_pos) {
    return false;
  }
  // cart_y_cur_pos
  if (lhs->cart_y_cur_pos != rhs->cart_y_cur_pos) {
    return false;
  }
  // cart_z_cur_pos
  if (lhs->cart_z_cur_pos != rhs->cart_z_cur_pos) {
    return false;
  }
  // cart_a_cur_pos
  if (lhs->cart_a_cur_pos != rhs->cart_a_cur_pos) {
    return false;
  }
  // cart_b_cur_pos
  if (lhs->cart_b_cur_pos != rhs->cart_b_cur_pos) {
    return false;
  }
  // cart_c_cur_pos
  if (lhs->cart_c_cur_pos != rhs->cart_c_cur_pos) {
    return false;
  }
  // flange_x_cur_pos
  if (lhs->flange_x_cur_pos != rhs->flange_x_cur_pos) {
    return false;
  }
  // flange_y_cur_pos
  if (lhs->flange_y_cur_pos != rhs->flange_y_cur_pos) {
    return false;
  }
  // flange_z_cur_pos
  if (lhs->flange_z_cur_pos != rhs->flange_z_cur_pos) {
    return false;
  }
  // flange_a_cur_pos
  if (lhs->flange_a_cur_pos != rhs->flange_a_cur_pos) {
    return false;
  }
  // flange_b_cur_pos
  if (lhs->flange_b_cur_pos != rhs->flange_b_cur_pos) {
    return false;
  }
  // flange_c_cur_pos
  if (lhs->flange_c_cur_pos != rhs->flange_c_cur_pos) {
    return false;
  }
  // exaxispos1
  if (lhs->exaxispos1 != rhs->exaxispos1) {
    return false;
  }
  // exaxispos2
  if (lhs->exaxispos2 != rhs->exaxispos2) {
    return false;
  }
  // exaxispos3
  if (lhs->exaxispos3 != rhs->exaxispos3) {
    return false;
  }
  // exaxispos4
  if (lhs->exaxispos4 != rhs->exaxispos4) {
    return false;
  }
  // ft_fx_data
  if (lhs->ft_fx_data != rhs->ft_fx_data) {
    return false;
  }
  // ft_fy_data
  if (lhs->ft_fy_data != rhs->ft_fy_data) {
    return false;
  }
  // ft_fz_data
  if (lhs->ft_fz_data != rhs->ft_fz_data) {
    return false;
  }
  // ft_tx_data
  if (lhs->ft_tx_data != rhs->ft_tx_data) {
    return false;
  }
  // ft_ty_data
  if (lhs->ft_ty_data != rhs->ft_ty_data) {
    return false;
  }
  // ft_tz_data
  if (lhs->ft_tz_data != rhs->ft_tz_data) {
    return false;
  }
  // ft_actstatus
  if (lhs->ft_actstatus != rhs->ft_actstatus) {
    return false;
  }
  // robot_mode
  if (lhs->robot_mode != rhs->robot_mode) {
    return false;
  }
  // tool_num
  if (lhs->tool_num != rhs->tool_num) {
    return false;
  }
  // work_num
  if (lhs->work_num != rhs->work_num) {
    return false;
  }
  // prg_state
  if (lhs->prg_state != rhs->prg_state) {
    return false;
  }
  // abnormal_stop
  if (lhs->abnormal_stop != rhs->abnormal_stop) {
    return false;
  }
  // prg_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->prg_name), &(rhs->prg_name)))
  {
    return false;
  }
  // prg_total_line
  if (lhs->prg_total_line != rhs->prg_total_line) {
    return false;
  }
  // prg_cur_line
  if (lhs->prg_cur_line != rhs->prg_cur_line) {
    return false;
  }
  // dgt_output_h
  if (lhs->dgt_output_h != rhs->dgt_output_h) {
    return false;
  }
  // dgt_output_l
  if (lhs->dgt_output_l != rhs->dgt_output_l) {
    return false;
  }
  // dgt_input_h
  if (lhs->dgt_input_h != rhs->dgt_input_h) {
    return false;
  }
  // dgt_input_l
  if (lhs->dgt_input_l != rhs->dgt_input_l) {
    return false;
  }
  // tl_dgt_output_l
  if (lhs->tl_dgt_output_l != rhs->tl_dgt_output_l) {
    return false;
  }
  // tl_dgt_input_l
  if (lhs->tl_dgt_input_l != rhs->tl_dgt_input_l) {
    return false;
  }
  // emg
  if (lhs->emg != rhs->emg) {
    return false;
  }
  // safetyboxsig
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->safetyboxsig[i] != rhs->safetyboxsig[i]) {
      return false;
    }
  }
  // robot_motion_done
  if (lhs->robot_motion_done != rhs->robot_motion_done) {
    return false;
  }
  // grip_motion_done
  if (lhs->grip_motion_done != rhs->grip_motion_done) {
    return false;
  }
  // weldbreakoffstate
  if (lhs->weldbreakoffstate != rhs->weldbreakoffstate) {
    return false;
  }
  // weldarcstate
  if (lhs->weldarcstate != rhs->weldarcstate) {
    return false;
  }
  // welding_voltage
  if (lhs->welding_voltage != rhs->welding_voltage) {
    return false;
  }
  // welding_current
  if (lhs->welding_current != rhs->welding_current) {
    return false;
  }
  // weldtrackspeed
  if (lhs->weldtrackspeed != rhs->weldtrackspeed) {
    return false;
  }
  // main_error_code
  if (lhs->main_error_code != rhs->main_error_code) {
    return false;
  }
  // sub_error_code
  if (lhs->sub_error_code != rhs->sub_error_code) {
    return false;
  }
  // check_sum
  if (lhs->check_sum != rhs->check_sum) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->version), &(rhs->version)))
  {
    return false;
  }
  // tpd_exception
  if (lhs->tpd_exception != rhs->tpd_exception) {
    return false;
  }
  // alarm_reboot_robot
  if (lhs->alarm_reboot_robot != rhs->alarm_reboot_robot) {
    return false;
  }
  // modbusmasterconnectstate
  if (lhs->modbusmasterconnectstate != rhs->modbusmasterconnectstate) {
    return false;
  }
  // mdbsslaveconnect
  if (lhs->mdbsslaveconnect != rhs->mdbsslaveconnect) {
    return false;
  }
  // socket_conn_timeout
  if (lhs->socket_conn_timeout != rhs->socket_conn_timeout) {
    return false;
  }
  // socket_read_timeout
  if (lhs->socket_read_timeout != rhs->socket_read_timeout) {
    return false;
  }
  // btn_box_stop_signa
  if (lhs->btn_box_stop_signa != rhs->btn_box_stop_signa) {
    return false;
  }
  // strangeposflag
  if (lhs->strangeposflag != rhs->strangeposflag) {
    return false;
  }
  // drag_alarm
  if (lhs->drag_alarm != rhs->drag_alarm) {
    return false;
  }
  // alarm
  if (lhs->alarm != rhs->alarm) {
    return false;
  }
  // safetydoor_alarm
  if (lhs->safetydoor_alarm != rhs->safetydoor_alarm) {
    return false;
  }
  // safetyplanealarm
  if (lhs->safetyplanealarm != rhs->safetyplanealarm) {
    return false;
  }
  // motionalarm
  if (lhs->motionalarm != rhs->motionalarm) {
    return false;
  }
  // interferealarm
  if (lhs->interferealarm != rhs->interferealarm) {
    return false;
  }
  // endluaerrcode
  if (lhs->endluaerrcode != rhs->endluaerrcode) {
    return false;
  }
  // dr_alarm
  if (lhs->dr_alarm != rhs->dr_alarm) {
    return false;
  }
  // udpcmdstate
  if (lhs->udpcmdstate != rhs->udpcmdstate) {
    return false;
  }
  // aliveslavenumerror
  if (lhs->aliveslavenumerror != rhs->aliveslavenumerror) {
    return false;
  }
  // gripperfaultnum
  if (lhs->gripperfaultnum != rhs->gripperfaultnum) {
    return false;
  }
  // slavecomerror
  for (size_t i = 0; i < 8; ++i) {
    if (lhs->slavecomerror[i] != rhs->slavecomerror[i]) {
      return false;
    }
  }
  // cmdpointerror
  if (lhs->cmdpointerror != rhs->cmdpointerror) {
    return false;
  }
  // ioerror
  if (lhs->ioerror != rhs->ioerror) {
    return false;
  }
  // grippererro
  if (lhs->grippererro != rhs->grippererro) {
    return false;
  }
  // fileerror
  if (lhs->fileerror != rhs->fileerror) {
    return false;
  }
  // paraerror
  if (lhs->paraerror != rhs->paraerror) {
    return false;
  }
  // exaxis_out_slimit_error
  if (lhs->exaxis_out_slimit_error != rhs->exaxis_out_slimit_error) {
    return false;
  }
  // dr_com_err
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->dr_com_err[i] != rhs->dr_com_err[i]) {
      return false;
    }
  }
  // dr_err
  if (lhs->dr_err != rhs->dr_err) {
    return false;
  }
  // out_sflimit_err
  if (lhs->out_sflimit_err != rhs->out_sflimit_err) {
    return false;
  }
  // collision_err
  if (lhs->collision_err != rhs->collision_err) {
    return false;
  }
  // weld_readystate
  if (lhs->weld_readystate != rhs->weld_readystate) {
    return false;
  }
  // alarm_check_emerg_stop_btn
  if (lhs->alarm_check_emerg_stop_btn != rhs->alarm_check_emerg_stop_btn) {
    return false;
  }
  // ts_web_state_com_error
  if (lhs->ts_web_state_com_error != rhs->ts_web_state_com_error) {
    return false;
  }
  // ts_tm_cmd_com_error
  if (lhs->ts_tm_cmd_com_error != rhs->ts_tm_cmd_com_error) {
    return false;
  }
  // ts_tm_state_com_error
  if (lhs->ts_tm_state_com_error != rhs->ts_tm_state_com_error) {
    return false;
  }
  // ctrlboxerror
  if (lhs->ctrlboxerror != rhs->ctrlboxerror) {
    return false;
  }
  // safety_data_state
  if (lhs->safety_data_state != rhs->safety_data_state) {
    return false;
  }
  // forcesensorerrstate
  if (lhs->forcesensorerrstate != rhs->forcesensorerrstate) {
    return false;
  }
  // ctrlopenluaerrcode
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->ctrlopenluaerrcode[i] != rhs->ctrlopenluaerrcode[i]) {
      return false;
    }
  }
  return true;
}

bool
fairino_msgs__msg__RobotNonrtState__copy(
  const fairino_msgs__msg__RobotNonrtState * input,
  fairino_msgs__msg__RobotNonrtState * output)
{
  if (!input || !output) {
    return false;
  }
  // j1_cur_pos
  output->j1_cur_pos = input->j1_cur_pos;
  // j2_cur_pos
  output->j2_cur_pos = input->j2_cur_pos;
  // j3_cur_pos
  output->j3_cur_pos = input->j3_cur_pos;
  // j4_cur_pos
  output->j4_cur_pos = input->j4_cur_pos;
  // j5_cur_pos
  output->j5_cur_pos = input->j5_cur_pos;
  // j6_cur_pos
  output->j6_cur_pos = input->j6_cur_pos;
  // j1_cur_tor
  output->j1_cur_tor = input->j1_cur_tor;
  // j2_cur_tor
  output->j2_cur_tor = input->j2_cur_tor;
  // j3_cur_tor
  output->j3_cur_tor = input->j3_cur_tor;
  // j4_cur_tor
  output->j4_cur_tor = input->j4_cur_tor;
  // j5_cur_tor
  output->j5_cur_tor = input->j5_cur_tor;
  // j6_cur_tor
  output->j6_cur_tor = input->j6_cur_tor;
  // cart_x_cur_pos
  output->cart_x_cur_pos = input->cart_x_cur_pos;
  // cart_y_cur_pos
  output->cart_y_cur_pos = input->cart_y_cur_pos;
  // cart_z_cur_pos
  output->cart_z_cur_pos = input->cart_z_cur_pos;
  // cart_a_cur_pos
  output->cart_a_cur_pos = input->cart_a_cur_pos;
  // cart_b_cur_pos
  output->cart_b_cur_pos = input->cart_b_cur_pos;
  // cart_c_cur_pos
  output->cart_c_cur_pos = input->cart_c_cur_pos;
  // flange_x_cur_pos
  output->flange_x_cur_pos = input->flange_x_cur_pos;
  // flange_y_cur_pos
  output->flange_y_cur_pos = input->flange_y_cur_pos;
  // flange_z_cur_pos
  output->flange_z_cur_pos = input->flange_z_cur_pos;
  // flange_a_cur_pos
  output->flange_a_cur_pos = input->flange_a_cur_pos;
  // flange_b_cur_pos
  output->flange_b_cur_pos = input->flange_b_cur_pos;
  // flange_c_cur_pos
  output->flange_c_cur_pos = input->flange_c_cur_pos;
  // exaxispos1
  output->exaxispos1 = input->exaxispos1;
  // exaxispos2
  output->exaxispos2 = input->exaxispos2;
  // exaxispos3
  output->exaxispos3 = input->exaxispos3;
  // exaxispos4
  output->exaxispos4 = input->exaxispos4;
  // ft_fx_data
  output->ft_fx_data = input->ft_fx_data;
  // ft_fy_data
  output->ft_fy_data = input->ft_fy_data;
  // ft_fz_data
  output->ft_fz_data = input->ft_fz_data;
  // ft_tx_data
  output->ft_tx_data = input->ft_tx_data;
  // ft_ty_data
  output->ft_ty_data = input->ft_ty_data;
  // ft_tz_data
  output->ft_tz_data = input->ft_tz_data;
  // ft_actstatus
  output->ft_actstatus = input->ft_actstatus;
  // robot_mode
  output->robot_mode = input->robot_mode;
  // tool_num
  output->tool_num = input->tool_num;
  // work_num
  output->work_num = input->work_num;
  // prg_state
  output->prg_state = input->prg_state;
  // abnormal_stop
  output->abnormal_stop = input->abnormal_stop;
  // prg_name
  if (!rosidl_runtime_c__String__copy(
      &(input->prg_name), &(output->prg_name)))
  {
    return false;
  }
  // prg_total_line
  output->prg_total_line = input->prg_total_line;
  // prg_cur_line
  output->prg_cur_line = input->prg_cur_line;
  // dgt_output_h
  output->dgt_output_h = input->dgt_output_h;
  // dgt_output_l
  output->dgt_output_l = input->dgt_output_l;
  // dgt_input_h
  output->dgt_input_h = input->dgt_input_h;
  // dgt_input_l
  output->dgt_input_l = input->dgt_input_l;
  // tl_dgt_output_l
  output->tl_dgt_output_l = input->tl_dgt_output_l;
  // tl_dgt_input_l
  output->tl_dgt_input_l = input->tl_dgt_input_l;
  // emg
  output->emg = input->emg;
  // safetyboxsig
  for (size_t i = 0; i < 6; ++i) {
    output->safetyboxsig[i] = input->safetyboxsig[i];
  }
  // robot_motion_done
  output->robot_motion_done = input->robot_motion_done;
  // grip_motion_done
  output->grip_motion_done = input->grip_motion_done;
  // weldbreakoffstate
  output->weldbreakoffstate = input->weldbreakoffstate;
  // weldarcstate
  output->weldarcstate = input->weldarcstate;
  // welding_voltage
  output->welding_voltage = input->welding_voltage;
  // welding_current
  output->welding_current = input->welding_current;
  // weldtrackspeed
  output->weldtrackspeed = input->weldtrackspeed;
  // main_error_code
  output->main_error_code = input->main_error_code;
  // sub_error_code
  output->sub_error_code = input->sub_error_code;
  // check_sum
  output->check_sum = input->check_sum;
  // timestamp
  output->timestamp = input->timestamp;
  // version
  if (!rosidl_runtime_c__String__copy(
      &(input->version), &(output->version)))
  {
    return false;
  }
  // tpd_exception
  output->tpd_exception = input->tpd_exception;
  // alarm_reboot_robot
  output->alarm_reboot_robot = input->alarm_reboot_robot;
  // modbusmasterconnectstate
  output->modbusmasterconnectstate = input->modbusmasterconnectstate;
  // mdbsslaveconnect
  output->mdbsslaveconnect = input->mdbsslaveconnect;
  // socket_conn_timeout
  output->socket_conn_timeout = input->socket_conn_timeout;
  // socket_read_timeout
  output->socket_read_timeout = input->socket_read_timeout;
  // btn_box_stop_signa
  output->btn_box_stop_signa = input->btn_box_stop_signa;
  // strangeposflag
  output->strangeposflag = input->strangeposflag;
  // drag_alarm
  output->drag_alarm = input->drag_alarm;
  // alarm
  output->alarm = input->alarm;
  // safetydoor_alarm
  output->safetydoor_alarm = input->safetydoor_alarm;
  // safetyplanealarm
  output->safetyplanealarm = input->safetyplanealarm;
  // motionalarm
  output->motionalarm = input->motionalarm;
  // interferealarm
  output->interferealarm = input->interferealarm;
  // endluaerrcode
  output->endluaerrcode = input->endluaerrcode;
  // dr_alarm
  output->dr_alarm = input->dr_alarm;
  // udpcmdstate
  output->udpcmdstate = input->udpcmdstate;
  // aliveslavenumerror
  output->aliveslavenumerror = input->aliveslavenumerror;
  // gripperfaultnum
  output->gripperfaultnum = input->gripperfaultnum;
  // slavecomerror
  for (size_t i = 0; i < 8; ++i) {
    output->slavecomerror[i] = input->slavecomerror[i];
  }
  // cmdpointerror
  output->cmdpointerror = input->cmdpointerror;
  // ioerror
  output->ioerror = input->ioerror;
  // grippererro
  output->grippererro = input->grippererro;
  // fileerror
  output->fileerror = input->fileerror;
  // paraerror
  output->paraerror = input->paraerror;
  // exaxis_out_slimit_error
  output->exaxis_out_slimit_error = input->exaxis_out_slimit_error;
  // dr_com_err
  for (size_t i = 0; i < 6; ++i) {
    output->dr_com_err[i] = input->dr_com_err[i];
  }
  // dr_err
  output->dr_err = input->dr_err;
  // out_sflimit_err
  output->out_sflimit_err = input->out_sflimit_err;
  // collision_err
  output->collision_err = input->collision_err;
  // weld_readystate
  output->weld_readystate = input->weld_readystate;
  // alarm_check_emerg_stop_btn
  output->alarm_check_emerg_stop_btn = input->alarm_check_emerg_stop_btn;
  // ts_web_state_com_error
  output->ts_web_state_com_error = input->ts_web_state_com_error;
  // ts_tm_cmd_com_error
  output->ts_tm_cmd_com_error = input->ts_tm_cmd_com_error;
  // ts_tm_state_com_error
  output->ts_tm_state_com_error = input->ts_tm_state_com_error;
  // ctrlboxerror
  output->ctrlboxerror = input->ctrlboxerror;
  // safety_data_state
  output->safety_data_state = input->safety_data_state;
  // forcesensorerrstate
  output->forcesensorerrstate = input->forcesensorerrstate;
  // ctrlopenluaerrcode
  for (size_t i = 0; i < 4; ++i) {
    output->ctrlopenluaerrcode[i] = input->ctrlopenluaerrcode[i];
  }
  return true;
}

fairino_msgs__msg__RobotNonrtState *
fairino_msgs__msg__RobotNonrtState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fairino_msgs__msg__RobotNonrtState * msg = (fairino_msgs__msg__RobotNonrtState *)allocator.allocate(sizeof(fairino_msgs__msg__RobotNonrtState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(fairino_msgs__msg__RobotNonrtState));
  bool success = fairino_msgs__msg__RobotNonrtState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
fairino_msgs__msg__RobotNonrtState__destroy(fairino_msgs__msg__RobotNonrtState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    fairino_msgs__msg__RobotNonrtState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
fairino_msgs__msg__RobotNonrtState__Sequence__init(fairino_msgs__msg__RobotNonrtState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fairino_msgs__msg__RobotNonrtState * data = NULL;

  if (size) {
    data = (fairino_msgs__msg__RobotNonrtState *)allocator.zero_allocate(size, sizeof(fairino_msgs__msg__RobotNonrtState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = fairino_msgs__msg__RobotNonrtState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        fairino_msgs__msg__RobotNonrtState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
fairino_msgs__msg__RobotNonrtState__Sequence__fini(fairino_msgs__msg__RobotNonrtState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      fairino_msgs__msg__RobotNonrtState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

fairino_msgs__msg__RobotNonrtState__Sequence *
fairino_msgs__msg__RobotNonrtState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fairino_msgs__msg__RobotNonrtState__Sequence * array = (fairino_msgs__msg__RobotNonrtState__Sequence *)allocator.allocate(sizeof(fairino_msgs__msg__RobotNonrtState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = fairino_msgs__msg__RobotNonrtState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
fairino_msgs__msg__RobotNonrtState__Sequence__destroy(fairino_msgs__msg__RobotNonrtState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    fairino_msgs__msg__RobotNonrtState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
fairino_msgs__msg__RobotNonrtState__Sequence__are_equal(const fairino_msgs__msg__RobotNonrtState__Sequence * lhs, const fairino_msgs__msg__RobotNonrtState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!fairino_msgs__msg__RobotNonrtState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
fairino_msgs__msg__RobotNonrtState__Sequence__copy(
  const fairino_msgs__msg__RobotNonrtState__Sequence * input,
  fairino_msgs__msg__RobotNonrtState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(fairino_msgs__msg__RobotNonrtState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    fairino_msgs__msg__RobotNonrtState * data =
      (fairino_msgs__msg__RobotNonrtState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!fairino_msgs__msg__RobotNonrtState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          fairino_msgs__msg__RobotNonrtState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!fairino_msgs__msg__RobotNonrtState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
