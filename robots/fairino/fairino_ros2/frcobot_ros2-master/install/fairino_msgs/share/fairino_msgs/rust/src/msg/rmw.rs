#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "fairino_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fairino_msgs__msg__RobotNonrtState() -> *const std::ffi::c_void;
}

#[link(name = "fairino_msgs__rosidl_generator_c")]
extern "C" {
    fn fairino_msgs__msg__RobotNonrtState__init(msg: *mut RobotNonrtState) -> bool;
    fn fairino_msgs__msg__RobotNonrtState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RobotNonrtState>, size: usize) -> bool;
    fn fairino_msgs__msg__RobotNonrtState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RobotNonrtState>);
    fn fairino_msgs__msg__RobotNonrtState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RobotNonrtState>, out_seq: *mut rosidl_runtime_rs::Sequence<RobotNonrtState>) -> bool;
}

// Corresponds to fairino_msgs__msg__RobotNonrtState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
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
    pub prg_name: rosidl_runtime_rs::String,


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
    pub version: rosidl_runtime_rs::String,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fairino_msgs__msg__RobotNonrtState__init(&mut msg as *mut _) {
        panic!("Call to fairino_msgs__msg__RobotNonrtState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RobotNonrtState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__msg__RobotNonrtState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__msg__RobotNonrtState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__msg__RobotNonrtState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RobotNonrtState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RobotNonrtState where Self: Sized {
  const TYPE_NAME: &'static str = "fairino_msgs/msg/RobotNonrtState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fairino_msgs__msg__RobotNonrtState() }
  }
}


