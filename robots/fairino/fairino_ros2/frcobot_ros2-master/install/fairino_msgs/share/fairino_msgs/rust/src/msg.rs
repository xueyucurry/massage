#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to fairino_msgs__msg__RobotNonrtState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotNonrtState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub j1_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub j2_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub j3_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub j4_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub j5_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub j6_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub j1_cur_tor: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub j2_cur_tor: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub j3_cur_tor: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub j4_cur_tor: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub j5_cur_tor: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub j6_cur_tor: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cart_x_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cart_y_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cart_z_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cart_a_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cart_b_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cart_c_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub flange_x_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub flange_y_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub flange_z_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub flange_a_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub flange_b_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub flange_c_cur_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub exaxispos1: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub exaxispos2: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub exaxispos3: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub exaxispos4: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ft_fx_data: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ft_fy_data: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ft_fz_data: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ft_tx_data: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ft_ty_data: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ft_tz_data: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ft_actstatus: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_mode: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tool_num: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub work_num: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub prg_state: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub abnormal_stop: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub prg_name: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub prg_total_line: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub prg_cur_line: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dgt_output_h: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dgt_output_l: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dgt_input_h: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dgt_input_l: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tl_dgt_output_l: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tl_dgt_input_l: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub emg: u8,

    /// V2.1 added
    pub safetyboxsig: [u8; 6],


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_motion_done: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub grip_motion_done: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub weldbreakoffstate: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub weldarcstate: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub welding_voltage: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub welding_current: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub weldtrackspeed: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub main_error_code: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sub_error_code: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub check_sum: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub timestamp: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub version: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tpd_exception: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub alarm_reboot_robot: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub modbusmasterconnectstate: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mdbsslaveconnect: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub socket_conn_timeout: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub socket_read_timeout: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub btn_box_stop_signa: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub strangeposflag: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub drag_alarm: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub alarm: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub safetydoor_alarm: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub safetyplanealarm: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub motionalarm: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub interferealarm: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub endluaerrcode: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dr_alarm: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub udpcmdstate: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub aliveslavenumerror: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub gripperfaultnum: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub slavecomerror: [u8; 8],


    // This member is not documented.
    #[allow(missing_docs)]
    pub cmdpointerror: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ioerror: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub grippererro: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fileerror: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub paraerror: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub exaxis_out_slimit_error: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dr_com_err: [u8; 6],


    // This member is not documented.
    #[allow(missing_docs)]
    pub dr_err: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub out_sflimit_err: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub collision_err: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub weld_readystate: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub alarm_check_emerg_stop_btn: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ts_web_state_com_error: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ts_tm_cmd_com_error: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ts_tm_state_com_error: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ctrlboxerror: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub safety_data_state: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub forcesensorerrstate: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ctrlopenluaerrcode: [u8; 4],

}



impl Default for RobotNonrtState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RobotNonrtState::default())
  }
}

impl rosidl_runtime_rs::Message for RobotNonrtState {
  type RmwMsg = super::msg::rmw::RobotNonrtState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        j1_cur_pos: msg.j1_cur_pos,
        j2_cur_pos: msg.j2_cur_pos,
        j3_cur_pos: msg.j3_cur_pos,
        j4_cur_pos: msg.j4_cur_pos,
        j5_cur_pos: msg.j5_cur_pos,
        j6_cur_pos: msg.j6_cur_pos,
        j1_cur_tor: msg.j1_cur_tor,
        j2_cur_tor: msg.j2_cur_tor,
        j3_cur_tor: msg.j3_cur_tor,
        j4_cur_tor: msg.j4_cur_tor,
        j5_cur_tor: msg.j5_cur_tor,
        j6_cur_tor: msg.j6_cur_tor,
        cart_x_cur_pos: msg.cart_x_cur_pos,
        cart_y_cur_pos: msg.cart_y_cur_pos,
        cart_z_cur_pos: msg.cart_z_cur_pos,
        cart_a_cur_pos: msg.cart_a_cur_pos,
        cart_b_cur_pos: msg.cart_b_cur_pos,
        cart_c_cur_pos: msg.cart_c_cur_pos,
        flange_x_cur_pos: msg.flange_x_cur_pos,
        flange_y_cur_pos: msg.flange_y_cur_pos,
        flange_z_cur_pos: msg.flange_z_cur_pos,
        flange_a_cur_pos: msg.flange_a_cur_pos,
        flange_b_cur_pos: msg.flange_b_cur_pos,
        flange_c_cur_pos: msg.flange_c_cur_pos,
        exaxispos1: msg.exaxispos1,
        exaxispos2: msg.exaxispos2,
        exaxispos3: msg.exaxispos3,
        exaxispos4: msg.exaxispos4,
        ft_fx_data: msg.ft_fx_data,
        ft_fy_data: msg.ft_fy_data,
        ft_fz_data: msg.ft_fz_data,
        ft_tx_data: msg.ft_tx_data,
        ft_ty_data: msg.ft_ty_data,
        ft_tz_data: msg.ft_tz_data,
        ft_actstatus: msg.ft_actstatus,
        robot_mode: msg.robot_mode,
        tool_num: msg.tool_num,
        work_num: msg.work_num,
        prg_state: msg.prg_state,
        abnormal_stop: msg.abnormal_stop,
        prg_name: msg.prg_name.as_str().into(),
        prg_total_line: msg.prg_total_line,
        prg_cur_line: msg.prg_cur_line,
        dgt_output_h: msg.dgt_output_h,
        dgt_output_l: msg.dgt_output_l,
        dgt_input_h: msg.dgt_input_h,
        dgt_input_l: msg.dgt_input_l,
        tl_dgt_output_l: msg.tl_dgt_output_l,
        tl_dgt_input_l: msg.tl_dgt_input_l,
        emg: msg.emg,
        safetyboxsig: msg.safetyboxsig,
        robot_motion_done: msg.robot_motion_done,
        grip_motion_done: msg.grip_motion_done,
        weldbreakoffstate: msg.weldbreakoffstate,
        weldarcstate: msg.weldarcstate,
        welding_voltage: msg.welding_voltage,
        welding_current: msg.welding_current,
        weldtrackspeed: msg.weldtrackspeed,
        main_error_code: msg.main_error_code,
        sub_error_code: msg.sub_error_code,
        check_sum: msg.check_sum,
        timestamp: msg.timestamp,
        version: msg.version.as_str().into(),
        tpd_exception: msg.tpd_exception,
        alarm_reboot_robot: msg.alarm_reboot_robot,
        modbusmasterconnectstate: msg.modbusmasterconnectstate,
        mdbsslaveconnect: msg.mdbsslaveconnect,
        socket_conn_timeout: msg.socket_conn_timeout,
        socket_read_timeout: msg.socket_read_timeout,
        btn_box_stop_signa: msg.btn_box_stop_signa,
        strangeposflag: msg.strangeposflag,
        drag_alarm: msg.drag_alarm,
        alarm: msg.alarm,
        safetydoor_alarm: msg.safetydoor_alarm,
        safetyplanealarm: msg.safetyplanealarm,
        motionalarm: msg.motionalarm,
        interferealarm: msg.interferealarm,
        endluaerrcode: msg.endluaerrcode,
        dr_alarm: msg.dr_alarm,
        udpcmdstate: msg.udpcmdstate,
        aliveslavenumerror: msg.aliveslavenumerror,
        gripperfaultnum: msg.gripperfaultnum,
        slavecomerror: msg.slavecomerror,
        cmdpointerror: msg.cmdpointerror,
        ioerror: msg.ioerror,
        grippererro: msg.grippererro,
        fileerror: msg.fileerror,
        paraerror: msg.paraerror,
        exaxis_out_slimit_error: msg.exaxis_out_slimit_error,
        dr_com_err: msg.dr_com_err,
        dr_err: msg.dr_err,
        out_sflimit_err: msg.out_sflimit_err,
        collision_err: msg.collision_err,
        weld_readystate: msg.weld_readystate,
        alarm_check_emerg_stop_btn: msg.alarm_check_emerg_stop_btn,
        ts_web_state_com_error: msg.ts_web_state_com_error,
        ts_tm_cmd_com_error: msg.ts_tm_cmd_com_error,
        ts_tm_state_com_error: msg.ts_tm_state_com_error,
        ctrlboxerror: msg.ctrlboxerror,
        safety_data_state: msg.safety_data_state,
        forcesensorerrstate: msg.forcesensorerrstate,
        ctrlopenluaerrcode: msg.ctrlopenluaerrcode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      j1_cur_pos: msg.j1_cur_pos,
      j2_cur_pos: msg.j2_cur_pos,
      j3_cur_pos: msg.j3_cur_pos,
      j4_cur_pos: msg.j4_cur_pos,
      j5_cur_pos: msg.j5_cur_pos,
      j6_cur_pos: msg.j6_cur_pos,
      j1_cur_tor: msg.j1_cur_tor,
      j2_cur_tor: msg.j2_cur_tor,
      j3_cur_tor: msg.j3_cur_tor,
      j4_cur_tor: msg.j4_cur_tor,
      j5_cur_tor: msg.j5_cur_tor,
      j6_cur_tor: msg.j6_cur_tor,
      cart_x_cur_pos: msg.cart_x_cur_pos,
      cart_y_cur_pos: msg.cart_y_cur_pos,
      cart_z_cur_pos: msg.cart_z_cur_pos,
      cart_a_cur_pos: msg.cart_a_cur_pos,
      cart_b_cur_pos: msg.cart_b_cur_pos,
      cart_c_cur_pos: msg.cart_c_cur_pos,
      flange_x_cur_pos: msg.flange_x_cur_pos,
      flange_y_cur_pos: msg.flange_y_cur_pos,
      flange_z_cur_pos: msg.flange_z_cur_pos,
      flange_a_cur_pos: msg.flange_a_cur_pos,
      flange_b_cur_pos: msg.flange_b_cur_pos,
      flange_c_cur_pos: msg.flange_c_cur_pos,
      exaxispos1: msg.exaxispos1,
      exaxispos2: msg.exaxispos2,
      exaxispos3: msg.exaxispos3,
      exaxispos4: msg.exaxispos4,
      ft_fx_data: msg.ft_fx_data,
      ft_fy_data: msg.ft_fy_data,
      ft_fz_data: msg.ft_fz_data,
      ft_tx_data: msg.ft_tx_data,
      ft_ty_data: msg.ft_ty_data,
      ft_tz_data: msg.ft_tz_data,
      ft_actstatus: msg.ft_actstatus,
      robot_mode: msg.robot_mode,
      tool_num: msg.tool_num,
      work_num: msg.work_num,
      prg_state: msg.prg_state,
      abnormal_stop: msg.abnormal_stop,
        prg_name: msg.prg_name.as_str().into(),
      prg_total_line: msg.prg_total_line,
      prg_cur_line: msg.prg_cur_line,
      dgt_output_h: msg.dgt_output_h,
      dgt_output_l: msg.dgt_output_l,
      dgt_input_h: msg.dgt_input_h,
      dgt_input_l: msg.dgt_input_l,
      tl_dgt_output_l: msg.tl_dgt_output_l,
      tl_dgt_input_l: msg.tl_dgt_input_l,
      emg: msg.emg,
        safetyboxsig: msg.safetyboxsig,
      robot_motion_done: msg.robot_motion_done,
      grip_motion_done: msg.grip_motion_done,
      weldbreakoffstate: msg.weldbreakoffstate,
      weldarcstate: msg.weldarcstate,
      welding_voltage: msg.welding_voltage,
      welding_current: msg.welding_current,
      weldtrackspeed: msg.weldtrackspeed,
      main_error_code: msg.main_error_code,
      sub_error_code: msg.sub_error_code,
      check_sum: msg.check_sum,
      timestamp: msg.timestamp,
        version: msg.version.as_str().into(),
      tpd_exception: msg.tpd_exception,
      alarm_reboot_robot: msg.alarm_reboot_robot,
      modbusmasterconnectstate: msg.modbusmasterconnectstate,
      mdbsslaveconnect: msg.mdbsslaveconnect,
      socket_conn_timeout: msg.socket_conn_timeout,
      socket_read_timeout: msg.socket_read_timeout,
      btn_box_stop_signa: msg.btn_box_stop_signa,
      strangeposflag: msg.strangeposflag,
      drag_alarm: msg.drag_alarm,
      alarm: msg.alarm,
      safetydoor_alarm: msg.safetydoor_alarm,
      safetyplanealarm: msg.safetyplanealarm,
      motionalarm: msg.motionalarm,
      interferealarm: msg.interferealarm,
      endluaerrcode: msg.endluaerrcode,
      dr_alarm: msg.dr_alarm,
      udpcmdstate: msg.udpcmdstate,
      aliveslavenumerror: msg.aliveslavenumerror,
      gripperfaultnum: msg.gripperfaultnum,
        slavecomerror: msg.slavecomerror,
      cmdpointerror: msg.cmdpointerror,
      ioerror: msg.ioerror,
      grippererro: msg.grippererro,
      fileerror: msg.fileerror,
      paraerror: msg.paraerror,
      exaxis_out_slimit_error: msg.exaxis_out_slimit_error,
        dr_com_err: msg.dr_com_err,
      dr_err: msg.dr_err,
      out_sflimit_err: msg.out_sflimit_err,
      collision_err: msg.collision_err,
      weld_readystate: msg.weld_readystate,
      alarm_check_emerg_stop_btn: msg.alarm_check_emerg_stop_btn,
      ts_web_state_com_error: msg.ts_web_state_com_error,
      ts_tm_cmd_com_error: msg.ts_tm_cmd_com_error,
      ts_tm_state_com_error: msg.ts_tm_state_com_error,
      ctrlboxerror: msg.ctrlboxerror,
      safety_data_state: msg.safety_data_state,
      forcesensorerrstate: msg.forcesensorerrstate,
        ctrlopenluaerrcode: msg.ctrlopenluaerrcode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      j1_cur_pos: msg.j1_cur_pos,
      j2_cur_pos: msg.j2_cur_pos,
      j3_cur_pos: msg.j3_cur_pos,
      j4_cur_pos: msg.j4_cur_pos,
      j5_cur_pos: msg.j5_cur_pos,
      j6_cur_pos: msg.j6_cur_pos,
      j1_cur_tor: msg.j1_cur_tor,
      j2_cur_tor: msg.j2_cur_tor,
      j3_cur_tor: msg.j3_cur_tor,
      j4_cur_tor: msg.j4_cur_tor,
      j5_cur_tor: msg.j5_cur_tor,
      j6_cur_tor: msg.j6_cur_tor,
      cart_x_cur_pos: msg.cart_x_cur_pos,
      cart_y_cur_pos: msg.cart_y_cur_pos,
      cart_z_cur_pos: msg.cart_z_cur_pos,
      cart_a_cur_pos: msg.cart_a_cur_pos,
      cart_b_cur_pos: msg.cart_b_cur_pos,
      cart_c_cur_pos: msg.cart_c_cur_pos,
      flange_x_cur_pos: msg.flange_x_cur_pos,
      flange_y_cur_pos: msg.flange_y_cur_pos,
      flange_z_cur_pos: msg.flange_z_cur_pos,
      flange_a_cur_pos: msg.flange_a_cur_pos,
      flange_b_cur_pos: msg.flange_b_cur_pos,
      flange_c_cur_pos: msg.flange_c_cur_pos,
      exaxispos1: msg.exaxispos1,
      exaxispos2: msg.exaxispos2,
      exaxispos3: msg.exaxispos3,
      exaxispos4: msg.exaxispos4,
      ft_fx_data: msg.ft_fx_data,
      ft_fy_data: msg.ft_fy_data,
      ft_fz_data: msg.ft_fz_data,
      ft_tx_data: msg.ft_tx_data,
      ft_ty_data: msg.ft_ty_data,
      ft_tz_data: msg.ft_tz_data,
      ft_actstatus: msg.ft_actstatus,
      robot_mode: msg.robot_mode,
      tool_num: msg.tool_num,
      work_num: msg.work_num,
      prg_state: msg.prg_state,
      abnormal_stop: msg.abnormal_stop,
      prg_name: msg.prg_name.to_string(),
      prg_total_line: msg.prg_total_line,
      prg_cur_line: msg.prg_cur_line,
      dgt_output_h: msg.dgt_output_h,
      dgt_output_l: msg.dgt_output_l,
      dgt_input_h: msg.dgt_input_h,
      dgt_input_l: msg.dgt_input_l,
      tl_dgt_output_l: msg.tl_dgt_output_l,
      tl_dgt_input_l: msg.tl_dgt_input_l,
      emg: msg.emg,
      safetyboxsig: msg.safetyboxsig,
      robot_motion_done: msg.robot_motion_done,
      grip_motion_done: msg.grip_motion_done,
      weldbreakoffstate: msg.weldbreakoffstate,
      weldarcstate: msg.weldarcstate,
      welding_voltage: msg.welding_voltage,
      welding_current: msg.welding_current,
      weldtrackspeed: msg.weldtrackspeed,
      main_error_code: msg.main_error_code,
      sub_error_code: msg.sub_error_code,
      check_sum: msg.check_sum,
      timestamp: msg.timestamp,
      version: msg.version.to_string(),
      tpd_exception: msg.tpd_exception,
      alarm_reboot_robot: msg.alarm_reboot_robot,
      modbusmasterconnectstate: msg.modbusmasterconnectstate,
      mdbsslaveconnect: msg.mdbsslaveconnect,
      socket_conn_timeout: msg.socket_conn_timeout,
      socket_read_timeout: msg.socket_read_timeout,
      btn_box_stop_signa: msg.btn_box_stop_signa,
      strangeposflag: msg.strangeposflag,
      drag_alarm: msg.drag_alarm,
      alarm: msg.alarm,
      safetydoor_alarm: msg.safetydoor_alarm,
      safetyplanealarm: msg.safetyplanealarm,
      motionalarm: msg.motionalarm,
      interferealarm: msg.interferealarm,
      endluaerrcode: msg.endluaerrcode,
      dr_alarm: msg.dr_alarm,
      udpcmdstate: msg.udpcmdstate,
      aliveslavenumerror: msg.aliveslavenumerror,
      gripperfaultnum: msg.gripperfaultnum,
      slavecomerror: msg.slavecomerror,
      cmdpointerror: msg.cmdpointerror,
      ioerror: msg.ioerror,
      grippererro: msg.grippererro,
      fileerror: msg.fileerror,
      paraerror: msg.paraerror,
      exaxis_out_slimit_error: msg.exaxis_out_slimit_error,
      dr_com_err: msg.dr_com_err,
      dr_err: msg.dr_err,
      out_sflimit_err: msg.out_sflimit_err,
      collision_err: msg.collision_err,
      weld_readystate: msg.weld_readystate,
      alarm_check_emerg_stop_btn: msg.alarm_check_emerg_stop_btn,
      ts_web_state_com_error: msg.ts_web_state_com_error,
      ts_tm_cmd_com_error: msg.ts_tm_cmd_com_error,
      ts_tm_state_com_error: msg.ts_tm_state_com_error,
      ctrlboxerror: msg.ctrlboxerror,
      safety_data_state: msg.safety_data_state,
      forcesensorerrstate: msg.forcesensorerrstate,
      ctrlopenluaerrcode: msg.ctrlopenluaerrcode,
    }
  }
}


