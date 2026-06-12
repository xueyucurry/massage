// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from fairino_msgs:msg/RobotNonrtState.idl
// generated code does not contain a copyright notice

#ifndef FAIRINO_MSGS__MSG__DETAIL__ROBOT_NONRT_STATE__STRUCT_HPP_
#define FAIRINO_MSGS__MSG__DETAIL__ROBOT_NONRT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__fairino_msgs__msg__RobotNonrtState __attribute__((deprecated))
#else
# define DEPRECATED__fairino_msgs__msg__RobotNonrtState __declspec(deprecated)
#endif

namespace fairino_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotNonrtState_
{
  using Type = RobotNonrtState_<ContainerAllocator>;

  explicit RobotNonrtState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->j1_cur_pos = 0.0;
      this->j2_cur_pos = 0.0;
      this->j3_cur_pos = 0.0;
      this->j4_cur_pos = 0.0;
      this->j5_cur_pos = 0.0;
      this->j6_cur_pos = 0.0;
      this->j1_cur_tor = 0.0;
      this->j2_cur_tor = 0.0;
      this->j3_cur_tor = 0.0;
      this->j4_cur_tor = 0.0;
      this->j5_cur_tor = 0.0;
      this->j6_cur_tor = 0.0;
      this->cart_x_cur_pos = 0.0;
      this->cart_y_cur_pos = 0.0;
      this->cart_z_cur_pos = 0.0;
      this->cart_a_cur_pos = 0.0;
      this->cart_b_cur_pos = 0.0;
      this->cart_c_cur_pos = 0.0;
      this->flange_x_cur_pos = 0.0;
      this->flange_y_cur_pos = 0.0;
      this->flange_z_cur_pos = 0.0;
      this->flange_a_cur_pos = 0.0;
      this->flange_b_cur_pos = 0.0;
      this->flange_c_cur_pos = 0.0;
      this->exaxispos1 = 0.0;
      this->exaxispos2 = 0.0;
      this->exaxispos3 = 0.0;
      this->exaxispos4 = 0.0;
      this->ft_fx_data = 0.0;
      this->ft_fy_data = 0.0;
      this->ft_fz_data = 0.0;
      this->ft_tx_data = 0.0;
      this->ft_ty_data = 0.0;
      this->ft_tz_data = 0.0;
      this->ft_actstatus = 0;
      this->robot_mode = 0;
      this->tool_num = 0;
      this->work_num = 0;
      this->prg_state = 0;
      this->abnormal_stop = 0;
      this->prg_name = "";
      this->prg_total_line = 0;
      this->prg_cur_line = 0;
      this->dgt_output_h = 0;
      this->dgt_output_l = 0;
      this->dgt_input_h = 0;
      this->dgt_input_l = 0;
      this->tl_dgt_output_l = 0;
      this->tl_dgt_input_l = 0;
      this->emg = 0;
      std::fill<typename std::array<uint8_t, 6>::iterator, uint8_t>(this->safetyboxsig.begin(), this->safetyboxsig.end(), 0);
      this->robot_motion_done = 0;
      this->grip_motion_done = 0;
      this->weldbreakoffstate = 0;
      this->weldarcstate = 0;
      this->welding_voltage = 0.0;
      this->welding_current = 0.0;
      this->weldtrackspeed = 0.0;
      this->main_error_code = 0ul;
      this->sub_error_code = 0ul;
      this->check_sum = 0;
      this->timestamp = 0ull;
      this->version = "";
      this->tpd_exception = 0;
      this->alarm_reboot_robot = 0;
      this->modbusmasterconnectstate = 0;
      this->mdbsslaveconnect = 0;
      this->socket_conn_timeout = 0;
      this->socket_read_timeout = 0;
      this->btn_box_stop_signa = 0;
      this->strangeposflag = 0;
      this->drag_alarm = 0;
      this->alarm = 0;
      this->safetydoor_alarm = 0;
      this->safetyplanealarm = 0;
      this->motionalarm = 0;
      this->interferealarm = 0;
      this->endluaerrcode = 0;
      this->dr_alarm = 0.0;
      this->udpcmdstate = 0;
      this->aliveslavenumerror = 0;
      this->gripperfaultnum = 0;
      std::fill<typename std::array<uint8_t, 8>::iterator, uint8_t>(this->slavecomerror.begin(), this->slavecomerror.end(), 0);
      this->cmdpointerror = 0;
      this->ioerror = 0;
      this->grippererro = 0;
      this->fileerror = 0;
      this->paraerror = 0;
      this->exaxis_out_slimit_error = 0;
      std::fill<typename std::array<uint8_t, 6>::iterator, uint8_t>(this->dr_com_err.begin(), this->dr_com_err.end(), 0);
      this->dr_err = 0.0;
      this->out_sflimit_err = 0.0;
      this->collision_err = 0.0;
      this->weld_readystate = 0;
      this->alarm_check_emerg_stop_btn = 0;
      this->ts_web_state_com_error = 0;
      this->ts_tm_cmd_com_error = 0;
      this->ts_tm_state_com_error = 0;
      this->ctrlboxerror = 0;
      this->safety_data_state = 0;
      this->forcesensorerrstate = 0;
      std::fill<typename std::array<uint8_t, 4>::iterator, uint8_t>(this->ctrlopenluaerrcode.begin(), this->ctrlopenluaerrcode.end(), 0);
    }
  }

  explicit RobotNonrtState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : prg_name(_alloc),
    safetyboxsig(_alloc),
    version(_alloc),
    slavecomerror(_alloc),
    dr_com_err(_alloc),
    ctrlopenluaerrcode(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->j1_cur_pos = 0.0;
      this->j2_cur_pos = 0.0;
      this->j3_cur_pos = 0.0;
      this->j4_cur_pos = 0.0;
      this->j5_cur_pos = 0.0;
      this->j6_cur_pos = 0.0;
      this->j1_cur_tor = 0.0;
      this->j2_cur_tor = 0.0;
      this->j3_cur_tor = 0.0;
      this->j4_cur_tor = 0.0;
      this->j5_cur_tor = 0.0;
      this->j6_cur_tor = 0.0;
      this->cart_x_cur_pos = 0.0;
      this->cart_y_cur_pos = 0.0;
      this->cart_z_cur_pos = 0.0;
      this->cart_a_cur_pos = 0.0;
      this->cart_b_cur_pos = 0.0;
      this->cart_c_cur_pos = 0.0;
      this->flange_x_cur_pos = 0.0;
      this->flange_y_cur_pos = 0.0;
      this->flange_z_cur_pos = 0.0;
      this->flange_a_cur_pos = 0.0;
      this->flange_b_cur_pos = 0.0;
      this->flange_c_cur_pos = 0.0;
      this->exaxispos1 = 0.0;
      this->exaxispos2 = 0.0;
      this->exaxispos3 = 0.0;
      this->exaxispos4 = 0.0;
      this->ft_fx_data = 0.0;
      this->ft_fy_data = 0.0;
      this->ft_fz_data = 0.0;
      this->ft_tx_data = 0.0;
      this->ft_ty_data = 0.0;
      this->ft_tz_data = 0.0;
      this->ft_actstatus = 0;
      this->robot_mode = 0;
      this->tool_num = 0;
      this->work_num = 0;
      this->prg_state = 0;
      this->abnormal_stop = 0;
      this->prg_name = "";
      this->prg_total_line = 0;
      this->prg_cur_line = 0;
      this->dgt_output_h = 0;
      this->dgt_output_l = 0;
      this->dgt_input_h = 0;
      this->dgt_input_l = 0;
      this->tl_dgt_output_l = 0;
      this->tl_dgt_input_l = 0;
      this->emg = 0;
      std::fill<typename std::array<uint8_t, 6>::iterator, uint8_t>(this->safetyboxsig.begin(), this->safetyboxsig.end(), 0);
      this->robot_motion_done = 0;
      this->grip_motion_done = 0;
      this->weldbreakoffstate = 0;
      this->weldarcstate = 0;
      this->welding_voltage = 0.0;
      this->welding_current = 0.0;
      this->weldtrackspeed = 0.0;
      this->main_error_code = 0ul;
      this->sub_error_code = 0ul;
      this->check_sum = 0;
      this->timestamp = 0ull;
      this->version = "";
      this->tpd_exception = 0;
      this->alarm_reboot_robot = 0;
      this->modbusmasterconnectstate = 0;
      this->mdbsslaveconnect = 0;
      this->socket_conn_timeout = 0;
      this->socket_read_timeout = 0;
      this->btn_box_stop_signa = 0;
      this->strangeposflag = 0;
      this->drag_alarm = 0;
      this->alarm = 0;
      this->safetydoor_alarm = 0;
      this->safetyplanealarm = 0;
      this->motionalarm = 0;
      this->interferealarm = 0;
      this->endluaerrcode = 0;
      this->dr_alarm = 0.0;
      this->udpcmdstate = 0;
      this->aliveslavenumerror = 0;
      this->gripperfaultnum = 0;
      std::fill<typename std::array<uint8_t, 8>::iterator, uint8_t>(this->slavecomerror.begin(), this->slavecomerror.end(), 0);
      this->cmdpointerror = 0;
      this->ioerror = 0;
      this->grippererro = 0;
      this->fileerror = 0;
      this->paraerror = 0;
      this->exaxis_out_slimit_error = 0;
      std::fill<typename std::array<uint8_t, 6>::iterator, uint8_t>(this->dr_com_err.begin(), this->dr_com_err.end(), 0);
      this->dr_err = 0.0;
      this->out_sflimit_err = 0.0;
      this->collision_err = 0.0;
      this->weld_readystate = 0;
      this->alarm_check_emerg_stop_btn = 0;
      this->ts_web_state_com_error = 0;
      this->ts_tm_cmd_com_error = 0;
      this->ts_tm_state_com_error = 0;
      this->ctrlboxerror = 0;
      this->safety_data_state = 0;
      this->forcesensorerrstate = 0;
      std::fill<typename std::array<uint8_t, 4>::iterator, uint8_t>(this->ctrlopenluaerrcode.begin(), this->ctrlopenluaerrcode.end(), 0);
    }
  }

  // field types and members
  using _j1_cur_pos_type =
    double;
  _j1_cur_pos_type j1_cur_pos;
  using _j2_cur_pos_type =
    double;
  _j2_cur_pos_type j2_cur_pos;
  using _j3_cur_pos_type =
    double;
  _j3_cur_pos_type j3_cur_pos;
  using _j4_cur_pos_type =
    double;
  _j4_cur_pos_type j4_cur_pos;
  using _j5_cur_pos_type =
    double;
  _j5_cur_pos_type j5_cur_pos;
  using _j6_cur_pos_type =
    double;
  _j6_cur_pos_type j6_cur_pos;
  using _j1_cur_tor_type =
    double;
  _j1_cur_tor_type j1_cur_tor;
  using _j2_cur_tor_type =
    double;
  _j2_cur_tor_type j2_cur_tor;
  using _j3_cur_tor_type =
    double;
  _j3_cur_tor_type j3_cur_tor;
  using _j4_cur_tor_type =
    double;
  _j4_cur_tor_type j4_cur_tor;
  using _j5_cur_tor_type =
    double;
  _j5_cur_tor_type j5_cur_tor;
  using _j6_cur_tor_type =
    double;
  _j6_cur_tor_type j6_cur_tor;
  using _cart_x_cur_pos_type =
    double;
  _cart_x_cur_pos_type cart_x_cur_pos;
  using _cart_y_cur_pos_type =
    double;
  _cart_y_cur_pos_type cart_y_cur_pos;
  using _cart_z_cur_pos_type =
    double;
  _cart_z_cur_pos_type cart_z_cur_pos;
  using _cart_a_cur_pos_type =
    double;
  _cart_a_cur_pos_type cart_a_cur_pos;
  using _cart_b_cur_pos_type =
    double;
  _cart_b_cur_pos_type cart_b_cur_pos;
  using _cart_c_cur_pos_type =
    double;
  _cart_c_cur_pos_type cart_c_cur_pos;
  using _flange_x_cur_pos_type =
    double;
  _flange_x_cur_pos_type flange_x_cur_pos;
  using _flange_y_cur_pos_type =
    double;
  _flange_y_cur_pos_type flange_y_cur_pos;
  using _flange_z_cur_pos_type =
    double;
  _flange_z_cur_pos_type flange_z_cur_pos;
  using _flange_a_cur_pos_type =
    double;
  _flange_a_cur_pos_type flange_a_cur_pos;
  using _flange_b_cur_pos_type =
    double;
  _flange_b_cur_pos_type flange_b_cur_pos;
  using _flange_c_cur_pos_type =
    double;
  _flange_c_cur_pos_type flange_c_cur_pos;
  using _exaxispos1_type =
    double;
  _exaxispos1_type exaxispos1;
  using _exaxispos2_type =
    double;
  _exaxispos2_type exaxispos2;
  using _exaxispos3_type =
    double;
  _exaxispos3_type exaxispos3;
  using _exaxispos4_type =
    double;
  _exaxispos4_type exaxispos4;
  using _ft_fx_data_type =
    double;
  _ft_fx_data_type ft_fx_data;
  using _ft_fy_data_type =
    double;
  _ft_fy_data_type ft_fy_data;
  using _ft_fz_data_type =
    double;
  _ft_fz_data_type ft_fz_data;
  using _ft_tx_data_type =
    double;
  _ft_tx_data_type ft_tx_data;
  using _ft_ty_data_type =
    double;
  _ft_ty_data_type ft_ty_data;
  using _ft_tz_data_type =
    double;
  _ft_tz_data_type ft_tz_data;
  using _ft_actstatus_type =
    uint8_t;
  _ft_actstatus_type ft_actstatus;
  using _robot_mode_type =
    uint8_t;
  _robot_mode_type robot_mode;
  using _tool_num_type =
    uint8_t;
  _tool_num_type tool_num;
  using _work_num_type =
    uint8_t;
  _work_num_type work_num;
  using _prg_state_type =
    uint8_t;
  _prg_state_type prg_state;
  using _abnormal_stop_type =
    uint8_t;
  _abnormal_stop_type abnormal_stop;
  using _prg_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _prg_name_type prg_name;
  using _prg_total_line_type =
    uint8_t;
  _prg_total_line_type prg_total_line;
  using _prg_cur_line_type =
    uint8_t;
  _prg_cur_line_type prg_cur_line;
  using _dgt_output_h_type =
    uint8_t;
  _dgt_output_h_type dgt_output_h;
  using _dgt_output_l_type =
    uint8_t;
  _dgt_output_l_type dgt_output_l;
  using _dgt_input_h_type =
    uint8_t;
  _dgt_input_h_type dgt_input_h;
  using _dgt_input_l_type =
    uint8_t;
  _dgt_input_l_type dgt_input_l;
  using _tl_dgt_output_l_type =
    uint8_t;
  _tl_dgt_output_l_type tl_dgt_output_l;
  using _tl_dgt_input_l_type =
    uint8_t;
  _tl_dgt_input_l_type tl_dgt_input_l;
  using _emg_type =
    uint8_t;
  _emg_type emg;
  using _safetyboxsig_type =
    std::array<uint8_t, 6>;
  _safetyboxsig_type safetyboxsig;
  using _robot_motion_done_type =
    uint8_t;
  _robot_motion_done_type robot_motion_done;
  using _grip_motion_done_type =
    uint8_t;
  _grip_motion_done_type grip_motion_done;
  using _weldbreakoffstate_type =
    uint8_t;
  _weldbreakoffstate_type weldbreakoffstate;
  using _weldarcstate_type =
    uint8_t;
  _weldarcstate_type weldarcstate;
  using _welding_voltage_type =
    double;
  _welding_voltage_type welding_voltage;
  using _welding_current_type =
    double;
  _welding_current_type welding_current;
  using _weldtrackspeed_type =
    double;
  _weldtrackspeed_type weldtrackspeed;
  using _main_error_code_type =
    uint32_t;
  _main_error_code_type main_error_code;
  using _sub_error_code_type =
    uint32_t;
  _sub_error_code_type sub_error_code;
  using _check_sum_type =
    uint8_t;
  _check_sum_type check_sum;
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _version_type version;
  using _tpd_exception_type =
    uint8_t;
  _tpd_exception_type tpd_exception;
  using _alarm_reboot_robot_type =
    uint8_t;
  _alarm_reboot_robot_type alarm_reboot_robot;
  using _modbusmasterconnectstate_type =
    uint8_t;
  _modbusmasterconnectstate_type modbusmasterconnectstate;
  using _mdbsslaveconnect_type =
    uint8_t;
  _mdbsslaveconnect_type mdbsslaveconnect;
  using _socket_conn_timeout_type =
    uint8_t;
  _socket_conn_timeout_type socket_conn_timeout;
  using _socket_read_timeout_type =
    uint8_t;
  _socket_read_timeout_type socket_read_timeout;
  using _btn_box_stop_signa_type =
    uint8_t;
  _btn_box_stop_signa_type btn_box_stop_signa;
  using _strangeposflag_type =
    uint8_t;
  _strangeposflag_type strangeposflag;
  using _drag_alarm_type =
    uint8_t;
  _drag_alarm_type drag_alarm;
  using _alarm_type =
    uint8_t;
  _alarm_type alarm;
  using _safetydoor_alarm_type =
    uint8_t;
  _safetydoor_alarm_type safetydoor_alarm;
  using _safetyplanealarm_type =
    uint8_t;
  _safetyplanealarm_type safetyplanealarm;
  using _motionalarm_type =
    uint8_t;
  _motionalarm_type motionalarm;
  using _interferealarm_type =
    uint8_t;
  _interferealarm_type interferealarm;
  using _endluaerrcode_type =
    uint16_t;
  _endluaerrcode_type endluaerrcode;
  using _dr_alarm_type =
    double;
  _dr_alarm_type dr_alarm;
  using _udpcmdstate_type =
    uint16_t;
  _udpcmdstate_type udpcmdstate;
  using _aliveslavenumerror_type =
    uint8_t;
  _aliveslavenumerror_type aliveslavenumerror;
  using _gripperfaultnum_type =
    uint16_t;
  _gripperfaultnum_type gripperfaultnum;
  using _slavecomerror_type =
    std::array<uint8_t, 8>;
  _slavecomerror_type slavecomerror;
  using _cmdpointerror_type =
    uint8_t;
  _cmdpointerror_type cmdpointerror;
  using _ioerror_type =
    uint8_t;
  _ioerror_type ioerror;
  using _grippererro_type =
    uint8_t;
  _grippererro_type grippererro;
  using _fileerror_type =
    uint8_t;
  _fileerror_type fileerror;
  using _paraerror_type =
    uint8_t;
  _paraerror_type paraerror;
  using _exaxis_out_slimit_error_type =
    uint8_t;
  _exaxis_out_slimit_error_type exaxis_out_slimit_error;
  using _dr_com_err_type =
    std::array<uint8_t, 6>;
  _dr_com_err_type dr_com_err;
  using _dr_err_type =
    double;
  _dr_err_type dr_err;
  using _out_sflimit_err_type =
    double;
  _out_sflimit_err_type out_sflimit_err;
  using _collision_err_type =
    double;
  _collision_err_type collision_err;
  using _weld_readystate_type =
    uint8_t;
  _weld_readystate_type weld_readystate;
  using _alarm_check_emerg_stop_btn_type =
    uint8_t;
  _alarm_check_emerg_stop_btn_type alarm_check_emerg_stop_btn;
  using _ts_web_state_com_error_type =
    uint8_t;
  _ts_web_state_com_error_type ts_web_state_com_error;
  using _ts_tm_cmd_com_error_type =
    uint8_t;
  _ts_tm_cmd_com_error_type ts_tm_cmd_com_error;
  using _ts_tm_state_com_error_type =
    uint8_t;
  _ts_tm_state_com_error_type ts_tm_state_com_error;
  using _ctrlboxerror_type =
    uint16_t;
  _ctrlboxerror_type ctrlboxerror;
  using _safety_data_state_type =
    uint8_t;
  _safety_data_state_type safety_data_state;
  using _forcesensorerrstate_type =
    uint8_t;
  _forcesensorerrstate_type forcesensorerrstate;
  using _ctrlopenluaerrcode_type =
    std::array<uint8_t, 4>;
  _ctrlopenluaerrcode_type ctrlopenluaerrcode;

  // setters for named parameter idiom
  Type & set__j1_cur_pos(
    const double & _arg)
  {
    this->j1_cur_pos = _arg;
    return *this;
  }
  Type & set__j2_cur_pos(
    const double & _arg)
  {
    this->j2_cur_pos = _arg;
    return *this;
  }
  Type & set__j3_cur_pos(
    const double & _arg)
  {
    this->j3_cur_pos = _arg;
    return *this;
  }
  Type & set__j4_cur_pos(
    const double & _arg)
  {
    this->j4_cur_pos = _arg;
    return *this;
  }
  Type & set__j5_cur_pos(
    const double & _arg)
  {
    this->j5_cur_pos = _arg;
    return *this;
  }
  Type & set__j6_cur_pos(
    const double & _arg)
  {
    this->j6_cur_pos = _arg;
    return *this;
  }
  Type & set__j1_cur_tor(
    const double & _arg)
  {
    this->j1_cur_tor = _arg;
    return *this;
  }
  Type & set__j2_cur_tor(
    const double & _arg)
  {
    this->j2_cur_tor = _arg;
    return *this;
  }
  Type & set__j3_cur_tor(
    const double & _arg)
  {
    this->j3_cur_tor = _arg;
    return *this;
  }
  Type & set__j4_cur_tor(
    const double & _arg)
  {
    this->j4_cur_tor = _arg;
    return *this;
  }
  Type & set__j5_cur_tor(
    const double & _arg)
  {
    this->j5_cur_tor = _arg;
    return *this;
  }
  Type & set__j6_cur_tor(
    const double & _arg)
  {
    this->j6_cur_tor = _arg;
    return *this;
  }
  Type & set__cart_x_cur_pos(
    const double & _arg)
  {
    this->cart_x_cur_pos = _arg;
    return *this;
  }
  Type & set__cart_y_cur_pos(
    const double & _arg)
  {
    this->cart_y_cur_pos = _arg;
    return *this;
  }
  Type & set__cart_z_cur_pos(
    const double & _arg)
  {
    this->cart_z_cur_pos = _arg;
    return *this;
  }
  Type & set__cart_a_cur_pos(
    const double & _arg)
  {
    this->cart_a_cur_pos = _arg;
    return *this;
  }
  Type & set__cart_b_cur_pos(
    const double & _arg)
  {
    this->cart_b_cur_pos = _arg;
    return *this;
  }
  Type & set__cart_c_cur_pos(
    const double & _arg)
  {
    this->cart_c_cur_pos = _arg;
    return *this;
  }
  Type & set__flange_x_cur_pos(
    const double & _arg)
  {
    this->flange_x_cur_pos = _arg;
    return *this;
  }
  Type & set__flange_y_cur_pos(
    const double & _arg)
  {
    this->flange_y_cur_pos = _arg;
    return *this;
  }
  Type & set__flange_z_cur_pos(
    const double & _arg)
  {
    this->flange_z_cur_pos = _arg;
    return *this;
  }
  Type & set__flange_a_cur_pos(
    const double & _arg)
  {
    this->flange_a_cur_pos = _arg;
    return *this;
  }
  Type & set__flange_b_cur_pos(
    const double & _arg)
  {
    this->flange_b_cur_pos = _arg;
    return *this;
  }
  Type & set__flange_c_cur_pos(
    const double & _arg)
  {
    this->flange_c_cur_pos = _arg;
    return *this;
  }
  Type & set__exaxispos1(
    const double & _arg)
  {
    this->exaxispos1 = _arg;
    return *this;
  }
  Type & set__exaxispos2(
    const double & _arg)
  {
    this->exaxispos2 = _arg;
    return *this;
  }
  Type & set__exaxispos3(
    const double & _arg)
  {
    this->exaxispos3 = _arg;
    return *this;
  }
  Type & set__exaxispos4(
    const double & _arg)
  {
    this->exaxispos4 = _arg;
    return *this;
  }
  Type & set__ft_fx_data(
    const double & _arg)
  {
    this->ft_fx_data = _arg;
    return *this;
  }
  Type & set__ft_fy_data(
    const double & _arg)
  {
    this->ft_fy_data = _arg;
    return *this;
  }
  Type & set__ft_fz_data(
    const double & _arg)
  {
    this->ft_fz_data = _arg;
    return *this;
  }
  Type & set__ft_tx_data(
    const double & _arg)
  {
    this->ft_tx_data = _arg;
    return *this;
  }
  Type & set__ft_ty_data(
    const double & _arg)
  {
    this->ft_ty_data = _arg;
    return *this;
  }
  Type & set__ft_tz_data(
    const double & _arg)
  {
    this->ft_tz_data = _arg;
    return *this;
  }
  Type & set__ft_actstatus(
    const uint8_t & _arg)
  {
    this->ft_actstatus = _arg;
    return *this;
  }
  Type & set__robot_mode(
    const uint8_t & _arg)
  {
    this->robot_mode = _arg;
    return *this;
  }
  Type & set__tool_num(
    const uint8_t & _arg)
  {
    this->tool_num = _arg;
    return *this;
  }
  Type & set__work_num(
    const uint8_t & _arg)
  {
    this->work_num = _arg;
    return *this;
  }
  Type & set__prg_state(
    const uint8_t & _arg)
  {
    this->prg_state = _arg;
    return *this;
  }
  Type & set__abnormal_stop(
    const uint8_t & _arg)
  {
    this->abnormal_stop = _arg;
    return *this;
  }
  Type & set__prg_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->prg_name = _arg;
    return *this;
  }
  Type & set__prg_total_line(
    const uint8_t & _arg)
  {
    this->prg_total_line = _arg;
    return *this;
  }
  Type & set__prg_cur_line(
    const uint8_t & _arg)
  {
    this->prg_cur_line = _arg;
    return *this;
  }
  Type & set__dgt_output_h(
    const uint8_t & _arg)
  {
    this->dgt_output_h = _arg;
    return *this;
  }
  Type & set__dgt_output_l(
    const uint8_t & _arg)
  {
    this->dgt_output_l = _arg;
    return *this;
  }
  Type & set__dgt_input_h(
    const uint8_t & _arg)
  {
    this->dgt_input_h = _arg;
    return *this;
  }
  Type & set__dgt_input_l(
    const uint8_t & _arg)
  {
    this->dgt_input_l = _arg;
    return *this;
  }
  Type & set__tl_dgt_output_l(
    const uint8_t & _arg)
  {
    this->tl_dgt_output_l = _arg;
    return *this;
  }
  Type & set__tl_dgt_input_l(
    const uint8_t & _arg)
  {
    this->tl_dgt_input_l = _arg;
    return *this;
  }
  Type & set__emg(
    const uint8_t & _arg)
  {
    this->emg = _arg;
    return *this;
  }
  Type & set__safetyboxsig(
    const std::array<uint8_t, 6> & _arg)
  {
    this->safetyboxsig = _arg;
    return *this;
  }
  Type & set__robot_motion_done(
    const uint8_t & _arg)
  {
    this->robot_motion_done = _arg;
    return *this;
  }
  Type & set__grip_motion_done(
    const uint8_t & _arg)
  {
    this->grip_motion_done = _arg;
    return *this;
  }
  Type & set__weldbreakoffstate(
    const uint8_t & _arg)
  {
    this->weldbreakoffstate = _arg;
    return *this;
  }
  Type & set__weldarcstate(
    const uint8_t & _arg)
  {
    this->weldarcstate = _arg;
    return *this;
  }
  Type & set__welding_voltage(
    const double & _arg)
  {
    this->welding_voltage = _arg;
    return *this;
  }
  Type & set__welding_current(
    const double & _arg)
  {
    this->welding_current = _arg;
    return *this;
  }
  Type & set__weldtrackspeed(
    const double & _arg)
  {
    this->weldtrackspeed = _arg;
    return *this;
  }
  Type & set__main_error_code(
    const uint32_t & _arg)
  {
    this->main_error_code = _arg;
    return *this;
  }
  Type & set__sub_error_code(
    const uint32_t & _arg)
  {
    this->sub_error_code = _arg;
    return *this;
  }
  Type & set__check_sum(
    const uint8_t & _arg)
  {
    this->check_sum = _arg;
    return *this;
  }
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__tpd_exception(
    const uint8_t & _arg)
  {
    this->tpd_exception = _arg;
    return *this;
  }
  Type & set__alarm_reboot_robot(
    const uint8_t & _arg)
  {
    this->alarm_reboot_robot = _arg;
    return *this;
  }
  Type & set__modbusmasterconnectstate(
    const uint8_t & _arg)
  {
    this->modbusmasterconnectstate = _arg;
    return *this;
  }
  Type & set__mdbsslaveconnect(
    const uint8_t & _arg)
  {
    this->mdbsslaveconnect = _arg;
    return *this;
  }
  Type & set__socket_conn_timeout(
    const uint8_t & _arg)
  {
    this->socket_conn_timeout = _arg;
    return *this;
  }
  Type & set__socket_read_timeout(
    const uint8_t & _arg)
  {
    this->socket_read_timeout = _arg;
    return *this;
  }
  Type & set__btn_box_stop_signa(
    const uint8_t & _arg)
  {
    this->btn_box_stop_signa = _arg;
    return *this;
  }
  Type & set__strangeposflag(
    const uint8_t & _arg)
  {
    this->strangeposflag = _arg;
    return *this;
  }
  Type & set__drag_alarm(
    const uint8_t & _arg)
  {
    this->drag_alarm = _arg;
    return *this;
  }
  Type & set__alarm(
    const uint8_t & _arg)
  {
    this->alarm = _arg;
    return *this;
  }
  Type & set__safetydoor_alarm(
    const uint8_t & _arg)
  {
    this->safetydoor_alarm = _arg;
    return *this;
  }
  Type & set__safetyplanealarm(
    const uint8_t & _arg)
  {
    this->safetyplanealarm = _arg;
    return *this;
  }
  Type & set__motionalarm(
    const uint8_t & _arg)
  {
    this->motionalarm = _arg;
    return *this;
  }
  Type & set__interferealarm(
    const uint8_t & _arg)
  {
    this->interferealarm = _arg;
    return *this;
  }
  Type & set__endluaerrcode(
    const uint16_t & _arg)
  {
    this->endluaerrcode = _arg;
    return *this;
  }
  Type & set__dr_alarm(
    const double & _arg)
  {
    this->dr_alarm = _arg;
    return *this;
  }
  Type & set__udpcmdstate(
    const uint16_t & _arg)
  {
    this->udpcmdstate = _arg;
    return *this;
  }
  Type & set__aliveslavenumerror(
    const uint8_t & _arg)
  {
    this->aliveslavenumerror = _arg;
    return *this;
  }
  Type & set__gripperfaultnum(
    const uint16_t & _arg)
  {
    this->gripperfaultnum = _arg;
    return *this;
  }
  Type & set__slavecomerror(
    const std::array<uint8_t, 8> & _arg)
  {
    this->slavecomerror = _arg;
    return *this;
  }
  Type & set__cmdpointerror(
    const uint8_t & _arg)
  {
    this->cmdpointerror = _arg;
    return *this;
  }
  Type & set__ioerror(
    const uint8_t & _arg)
  {
    this->ioerror = _arg;
    return *this;
  }
  Type & set__grippererro(
    const uint8_t & _arg)
  {
    this->grippererro = _arg;
    return *this;
  }
  Type & set__fileerror(
    const uint8_t & _arg)
  {
    this->fileerror = _arg;
    return *this;
  }
  Type & set__paraerror(
    const uint8_t & _arg)
  {
    this->paraerror = _arg;
    return *this;
  }
  Type & set__exaxis_out_slimit_error(
    const uint8_t & _arg)
  {
    this->exaxis_out_slimit_error = _arg;
    return *this;
  }
  Type & set__dr_com_err(
    const std::array<uint8_t, 6> & _arg)
  {
    this->dr_com_err = _arg;
    return *this;
  }
  Type & set__dr_err(
    const double & _arg)
  {
    this->dr_err = _arg;
    return *this;
  }
  Type & set__out_sflimit_err(
    const double & _arg)
  {
    this->out_sflimit_err = _arg;
    return *this;
  }
  Type & set__collision_err(
    const double & _arg)
  {
    this->collision_err = _arg;
    return *this;
  }
  Type & set__weld_readystate(
    const uint8_t & _arg)
  {
    this->weld_readystate = _arg;
    return *this;
  }
  Type & set__alarm_check_emerg_stop_btn(
    const uint8_t & _arg)
  {
    this->alarm_check_emerg_stop_btn = _arg;
    return *this;
  }
  Type & set__ts_web_state_com_error(
    const uint8_t & _arg)
  {
    this->ts_web_state_com_error = _arg;
    return *this;
  }
  Type & set__ts_tm_cmd_com_error(
    const uint8_t & _arg)
  {
    this->ts_tm_cmd_com_error = _arg;
    return *this;
  }
  Type & set__ts_tm_state_com_error(
    const uint8_t & _arg)
  {
    this->ts_tm_state_com_error = _arg;
    return *this;
  }
  Type & set__ctrlboxerror(
    const uint16_t & _arg)
  {
    this->ctrlboxerror = _arg;
    return *this;
  }
  Type & set__safety_data_state(
    const uint8_t & _arg)
  {
    this->safety_data_state = _arg;
    return *this;
  }
  Type & set__forcesensorerrstate(
    const uint8_t & _arg)
  {
    this->forcesensorerrstate = _arg;
    return *this;
  }
  Type & set__ctrlopenluaerrcode(
    const std::array<uint8_t, 4> & _arg)
  {
    this->ctrlopenluaerrcode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    fairino_msgs::msg::RobotNonrtState_<ContainerAllocator> *;
  using ConstRawPtr =
    const fairino_msgs::msg::RobotNonrtState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<fairino_msgs::msg::RobotNonrtState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<fairino_msgs::msg::RobotNonrtState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      fairino_msgs::msg::RobotNonrtState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<fairino_msgs::msg::RobotNonrtState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      fairino_msgs::msg::RobotNonrtState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<fairino_msgs::msg::RobotNonrtState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<fairino_msgs::msg::RobotNonrtState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<fairino_msgs::msg::RobotNonrtState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__fairino_msgs__msg__RobotNonrtState
    std::shared_ptr<fairino_msgs::msg::RobotNonrtState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__fairino_msgs__msg__RobotNonrtState
    std::shared_ptr<fairino_msgs::msg::RobotNonrtState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotNonrtState_ & other) const
  {
    if (this->j1_cur_pos != other.j1_cur_pos) {
      return false;
    }
    if (this->j2_cur_pos != other.j2_cur_pos) {
      return false;
    }
    if (this->j3_cur_pos != other.j3_cur_pos) {
      return false;
    }
    if (this->j4_cur_pos != other.j4_cur_pos) {
      return false;
    }
    if (this->j5_cur_pos != other.j5_cur_pos) {
      return false;
    }
    if (this->j6_cur_pos != other.j6_cur_pos) {
      return false;
    }
    if (this->j1_cur_tor != other.j1_cur_tor) {
      return false;
    }
    if (this->j2_cur_tor != other.j2_cur_tor) {
      return false;
    }
    if (this->j3_cur_tor != other.j3_cur_tor) {
      return false;
    }
    if (this->j4_cur_tor != other.j4_cur_tor) {
      return false;
    }
    if (this->j5_cur_tor != other.j5_cur_tor) {
      return false;
    }
    if (this->j6_cur_tor != other.j6_cur_tor) {
      return false;
    }
    if (this->cart_x_cur_pos != other.cart_x_cur_pos) {
      return false;
    }
    if (this->cart_y_cur_pos != other.cart_y_cur_pos) {
      return false;
    }
    if (this->cart_z_cur_pos != other.cart_z_cur_pos) {
      return false;
    }
    if (this->cart_a_cur_pos != other.cart_a_cur_pos) {
      return false;
    }
    if (this->cart_b_cur_pos != other.cart_b_cur_pos) {
      return false;
    }
    if (this->cart_c_cur_pos != other.cart_c_cur_pos) {
      return false;
    }
    if (this->flange_x_cur_pos != other.flange_x_cur_pos) {
      return false;
    }
    if (this->flange_y_cur_pos != other.flange_y_cur_pos) {
      return false;
    }
    if (this->flange_z_cur_pos != other.flange_z_cur_pos) {
      return false;
    }
    if (this->flange_a_cur_pos != other.flange_a_cur_pos) {
      return false;
    }
    if (this->flange_b_cur_pos != other.flange_b_cur_pos) {
      return false;
    }
    if (this->flange_c_cur_pos != other.flange_c_cur_pos) {
      return false;
    }
    if (this->exaxispos1 != other.exaxispos1) {
      return false;
    }
    if (this->exaxispos2 != other.exaxispos2) {
      return false;
    }
    if (this->exaxispos3 != other.exaxispos3) {
      return false;
    }
    if (this->exaxispos4 != other.exaxispos4) {
      return false;
    }
    if (this->ft_fx_data != other.ft_fx_data) {
      return false;
    }
    if (this->ft_fy_data != other.ft_fy_data) {
      return false;
    }
    if (this->ft_fz_data != other.ft_fz_data) {
      return false;
    }
    if (this->ft_tx_data != other.ft_tx_data) {
      return false;
    }
    if (this->ft_ty_data != other.ft_ty_data) {
      return false;
    }
    if (this->ft_tz_data != other.ft_tz_data) {
      return false;
    }
    if (this->ft_actstatus != other.ft_actstatus) {
      return false;
    }
    if (this->robot_mode != other.robot_mode) {
      return false;
    }
    if (this->tool_num != other.tool_num) {
      return false;
    }
    if (this->work_num != other.work_num) {
      return false;
    }
    if (this->prg_state != other.prg_state) {
      return false;
    }
    if (this->abnormal_stop != other.abnormal_stop) {
      return false;
    }
    if (this->prg_name != other.prg_name) {
      return false;
    }
    if (this->prg_total_line != other.prg_total_line) {
      return false;
    }
    if (this->prg_cur_line != other.prg_cur_line) {
      return false;
    }
    if (this->dgt_output_h != other.dgt_output_h) {
      return false;
    }
    if (this->dgt_output_l != other.dgt_output_l) {
      return false;
    }
    if (this->dgt_input_h != other.dgt_input_h) {
      return false;
    }
    if (this->dgt_input_l != other.dgt_input_l) {
      return false;
    }
    if (this->tl_dgt_output_l != other.tl_dgt_output_l) {
      return false;
    }
    if (this->tl_dgt_input_l != other.tl_dgt_input_l) {
      return false;
    }
    if (this->emg != other.emg) {
      return false;
    }
    if (this->safetyboxsig != other.safetyboxsig) {
      return false;
    }
    if (this->robot_motion_done != other.robot_motion_done) {
      return false;
    }
    if (this->grip_motion_done != other.grip_motion_done) {
      return false;
    }
    if (this->weldbreakoffstate != other.weldbreakoffstate) {
      return false;
    }
    if (this->weldarcstate != other.weldarcstate) {
      return false;
    }
    if (this->welding_voltage != other.welding_voltage) {
      return false;
    }
    if (this->welding_current != other.welding_current) {
      return false;
    }
    if (this->weldtrackspeed != other.weldtrackspeed) {
      return false;
    }
    if (this->main_error_code != other.main_error_code) {
      return false;
    }
    if (this->sub_error_code != other.sub_error_code) {
      return false;
    }
    if (this->check_sum != other.check_sum) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->tpd_exception != other.tpd_exception) {
      return false;
    }
    if (this->alarm_reboot_robot != other.alarm_reboot_robot) {
      return false;
    }
    if (this->modbusmasterconnectstate != other.modbusmasterconnectstate) {
      return false;
    }
    if (this->mdbsslaveconnect != other.mdbsslaveconnect) {
      return false;
    }
    if (this->socket_conn_timeout != other.socket_conn_timeout) {
      return false;
    }
    if (this->socket_read_timeout != other.socket_read_timeout) {
      return false;
    }
    if (this->btn_box_stop_signa != other.btn_box_stop_signa) {
      return false;
    }
    if (this->strangeposflag != other.strangeposflag) {
      return false;
    }
    if (this->drag_alarm != other.drag_alarm) {
      return false;
    }
    if (this->alarm != other.alarm) {
      return false;
    }
    if (this->safetydoor_alarm != other.safetydoor_alarm) {
      return false;
    }
    if (this->safetyplanealarm != other.safetyplanealarm) {
      return false;
    }
    if (this->motionalarm != other.motionalarm) {
      return false;
    }
    if (this->interferealarm != other.interferealarm) {
      return false;
    }
    if (this->endluaerrcode != other.endluaerrcode) {
      return false;
    }
    if (this->dr_alarm != other.dr_alarm) {
      return false;
    }
    if (this->udpcmdstate != other.udpcmdstate) {
      return false;
    }
    if (this->aliveslavenumerror != other.aliveslavenumerror) {
      return false;
    }
    if (this->gripperfaultnum != other.gripperfaultnum) {
      return false;
    }
    if (this->slavecomerror != other.slavecomerror) {
      return false;
    }
    if (this->cmdpointerror != other.cmdpointerror) {
      return false;
    }
    if (this->ioerror != other.ioerror) {
      return false;
    }
    if (this->grippererro != other.grippererro) {
      return false;
    }
    if (this->fileerror != other.fileerror) {
      return false;
    }
    if (this->paraerror != other.paraerror) {
      return false;
    }
    if (this->exaxis_out_slimit_error != other.exaxis_out_slimit_error) {
      return false;
    }
    if (this->dr_com_err != other.dr_com_err) {
      return false;
    }
    if (this->dr_err != other.dr_err) {
      return false;
    }
    if (this->out_sflimit_err != other.out_sflimit_err) {
      return false;
    }
    if (this->collision_err != other.collision_err) {
      return false;
    }
    if (this->weld_readystate != other.weld_readystate) {
      return false;
    }
    if (this->alarm_check_emerg_stop_btn != other.alarm_check_emerg_stop_btn) {
      return false;
    }
    if (this->ts_web_state_com_error != other.ts_web_state_com_error) {
      return false;
    }
    if (this->ts_tm_cmd_com_error != other.ts_tm_cmd_com_error) {
      return false;
    }
    if (this->ts_tm_state_com_error != other.ts_tm_state_com_error) {
      return false;
    }
    if (this->ctrlboxerror != other.ctrlboxerror) {
      return false;
    }
    if (this->safety_data_state != other.safety_data_state) {
      return false;
    }
    if (this->forcesensorerrstate != other.forcesensorerrstate) {
      return false;
    }
    if (this->ctrlopenluaerrcode != other.ctrlopenluaerrcode) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotNonrtState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotNonrtState_

// alias to use template instance with default allocator
using RobotNonrtState =
  fairino_msgs::msg::RobotNonrtState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace fairino_msgs

#endif  // FAIRINO_MSGS__MSG__DETAIL__ROBOT_NONRT_STATE__STRUCT_HPP_
