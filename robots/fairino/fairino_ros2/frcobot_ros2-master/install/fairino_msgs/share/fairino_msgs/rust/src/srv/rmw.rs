#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "fairino_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fairino_msgs__srv__RemoteCmdInterface_Request() -> *const std::ffi::c_void;
}

#[link(name = "fairino_msgs__rosidl_generator_c")]
extern "C" {
    fn fairino_msgs__srv__RemoteCmdInterface_Request__init(msg: *mut RemoteCmdInterface_Request) -> bool;
    fn fairino_msgs__srv__RemoteCmdInterface_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RemoteCmdInterface_Request>, size: usize) -> bool;
    fn fairino_msgs__srv__RemoteCmdInterface_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RemoteCmdInterface_Request>);
    fn fairino_msgs__srv__RemoteCmdInterface_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RemoteCmdInterface_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<RemoteCmdInterface_Request>) -> bool;
}

// Corresponds to fairino_msgs__srv__RemoteCmdInterface_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RemoteCmdInterface_Request {
    /// ros用户输入的字符串指令，比如movej(p1,100)
    pub cmd_str: rosidl_runtime_rs::String,

}



impl Default for RemoteCmdInterface_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fairino_msgs__srv__RemoteCmdInterface_Request__init(&mut msg as *mut _) {
        panic!("Call to fairino_msgs__srv__RemoteCmdInterface_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RemoteCmdInterface_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteCmdInterface_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteCmdInterface_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteCmdInterface_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RemoteCmdInterface_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RemoteCmdInterface_Request where Self: Sized {
  const TYPE_NAME: &'static str = "fairino_msgs/srv/RemoteCmdInterface_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fairino_msgs__srv__RemoteCmdInterface_Request() }
  }
}


#[link(name = "fairino_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fairino_msgs__srv__RemoteCmdInterface_Response() -> *const std::ffi::c_void;
}

#[link(name = "fairino_msgs__rosidl_generator_c")]
extern "C" {
    fn fairino_msgs__srv__RemoteCmdInterface_Response__init(msg: *mut RemoteCmdInterface_Response) -> bool;
    fn fairino_msgs__srv__RemoteCmdInterface_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RemoteCmdInterface_Response>, size: usize) -> bool;
    fn fairino_msgs__srv__RemoteCmdInterface_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RemoteCmdInterface_Response>);
    fn fairino_msgs__srv__RemoteCmdInterface_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RemoteCmdInterface_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<RemoteCmdInterface_Response>) -> bool;
}

// Corresponds to fairino_msgs__srv__RemoteCmdInterface_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RemoteCmdInterface_Response {
    /// 创建结果反馈，0-成功，-1-失败
    pub cmd_res: rosidl_runtime_rs::String,

}



impl Default for RemoteCmdInterface_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fairino_msgs__srv__RemoteCmdInterface_Response__init(&mut msg as *mut _) {
        panic!("Call to fairino_msgs__srv__RemoteCmdInterface_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RemoteCmdInterface_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteCmdInterface_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteCmdInterface_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteCmdInterface_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RemoteCmdInterface_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RemoteCmdInterface_Response where Self: Sized {
  const TYPE_NAME: &'static str = "fairino_msgs/srv/RemoteCmdInterface_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fairino_msgs__srv__RemoteCmdInterface_Response() }
  }
}


#[link(name = "fairino_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fairino_msgs__srv__RemoteScriptContent_Request() -> *const std::ffi::c_void;
}

#[link(name = "fairino_msgs__rosidl_generator_c")]
extern "C" {
    fn fairino_msgs__srv__RemoteScriptContent_Request__init(msg: *mut RemoteScriptContent_Request) -> bool;
    fn fairino_msgs__srv__RemoteScriptContent_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RemoteScriptContent_Request>, size: usize) -> bool;
    fn fairino_msgs__srv__RemoteScriptContent_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RemoteScriptContent_Request>);
    fn fairino_msgs__srv__RemoteScriptContent_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RemoteScriptContent_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<RemoteScriptContent_Request>) -> bool;
}

// Corresponds to fairino_msgs__srv__RemoteScriptContent_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RemoteScriptContent_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub line_str: rosidl_runtime_rs::String,

}



impl Default for RemoteScriptContent_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fairino_msgs__srv__RemoteScriptContent_Request__init(&mut msg as *mut _) {
        panic!("Call to fairino_msgs__srv__RemoteScriptContent_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RemoteScriptContent_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteScriptContent_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteScriptContent_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteScriptContent_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RemoteScriptContent_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RemoteScriptContent_Request where Self: Sized {
  const TYPE_NAME: &'static str = "fairino_msgs/srv/RemoteScriptContent_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fairino_msgs__srv__RemoteScriptContent_Request() }
  }
}


#[link(name = "fairino_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fairino_msgs__srv__RemoteScriptContent_Response() -> *const std::ffi::c_void;
}

#[link(name = "fairino_msgs__rosidl_generator_c")]
extern "C" {
    fn fairino_msgs__srv__RemoteScriptContent_Response__init(msg: *mut RemoteScriptContent_Response) -> bool;
    fn fairino_msgs__srv__RemoteScriptContent_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RemoteScriptContent_Response>, size: usize) -> bool;
    fn fairino_msgs__srv__RemoteScriptContent_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RemoteScriptContent_Response>);
    fn fairino_msgs__srv__RemoteScriptContent_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RemoteScriptContent_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<RemoteScriptContent_Response>) -> bool;
}

// Corresponds to fairino_msgs__srv__RemoteScriptContent_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RemoteScriptContent_Response {
    /// 创建结果反馈，0-成功，-1-失败
    pub res: rosidl_runtime_rs::String,

}



impl Default for RemoteScriptContent_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fairino_msgs__srv__RemoteScriptContent_Response__init(&mut msg as *mut _) {
        panic!("Call to fairino_msgs__srv__RemoteScriptContent_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RemoteScriptContent_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteScriptContent_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteScriptContent_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fairino_msgs__srv__RemoteScriptContent_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RemoteScriptContent_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RemoteScriptContent_Response where Self: Sized {
  const TYPE_NAME: &'static str = "fairino_msgs/srv/RemoteScriptContent_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fairino_msgs__srv__RemoteScriptContent_Response() }
  }
}






#[link(name = "fairino_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__fairino_msgs__srv__RemoteCmdInterface() -> *const std::ffi::c_void;
}

// Corresponds to fairino_msgs__srv__RemoteCmdInterface
#[allow(missing_docs, non_camel_case_types)]
pub struct RemoteCmdInterface;

impl rosidl_runtime_rs::Service for RemoteCmdInterface {
    type Request = RemoteCmdInterface_Request;
    type Response = RemoteCmdInterface_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__fairino_msgs__srv__RemoteCmdInterface() }
    }
}




#[link(name = "fairino_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__fairino_msgs__srv__RemoteScriptContent() -> *const std::ffi::c_void;
}

// Corresponds to fairino_msgs__srv__RemoteScriptContent
#[allow(missing_docs, non_camel_case_types)]
pub struct RemoteScriptContent;

impl rosidl_runtime_rs::Service for RemoteScriptContent {
    type Request = RemoteScriptContent_Request;
    type Response = RemoteScriptContent_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__fairino_msgs__srv__RemoteScriptContent() }
    }
}


