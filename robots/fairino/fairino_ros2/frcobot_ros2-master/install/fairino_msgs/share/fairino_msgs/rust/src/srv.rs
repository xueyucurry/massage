#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to fairino_msgs__srv__RemoteCmdInterface_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RemoteCmdInterface_Request {
    /// ros用户输入的字符串指令，比如movej(p1,100)
    pub cmd_str: std::string::String,

}



impl Default for RemoteCmdInterface_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::RemoteCmdInterface_Request::default())
  }
}

impl rosidl_runtime_rs::Message for RemoteCmdInterface_Request {
  type RmwMsg = super::srv::rmw::RemoteCmdInterface_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cmd_str: msg.cmd_str.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cmd_str: msg.cmd_str.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      cmd_str: msg.cmd_str.to_string(),
    }
  }
}


// Corresponds to fairino_msgs__srv__RemoteCmdInterface_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RemoteCmdInterface_Response {
    /// 创建结果反馈，0-成功，-1-失败
    pub cmd_res: std::string::String,

}



impl Default for RemoteCmdInterface_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::RemoteCmdInterface_Response::default())
  }
}

impl rosidl_runtime_rs::Message for RemoteCmdInterface_Response {
  type RmwMsg = super::srv::rmw::RemoteCmdInterface_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cmd_res: msg.cmd_res.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cmd_res: msg.cmd_res.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      cmd_res: msg.cmd_res.to_string(),
    }
  }
}


// Corresponds to fairino_msgs__srv__RemoteScriptContent_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RemoteScriptContent_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub line_str: std::string::String,

}



impl Default for RemoteScriptContent_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::RemoteScriptContent_Request::default())
  }
}

impl rosidl_runtime_rs::Message for RemoteScriptContent_Request {
  type RmwMsg = super::srv::rmw::RemoteScriptContent_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        line_str: msg.line_str.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        line_str: msg.line_str.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      line_str: msg.line_str.to_string(),
    }
  }
}


// Corresponds to fairino_msgs__srv__RemoteScriptContent_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RemoteScriptContent_Response {
    /// 创建结果反馈，0-成功，-1-失败
    pub res: std::string::String,

}



impl Default for RemoteScriptContent_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::RemoteScriptContent_Response::default())
  }
}

impl rosidl_runtime_rs::Message for RemoteScriptContent_Response {
  type RmwMsg = super::srv::rmw::RemoteScriptContent_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        res: msg.res.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        res: msg.res.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      res: msg.res.to_string(),
    }
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


