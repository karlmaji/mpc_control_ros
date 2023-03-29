// Auto-generated. Do not edit!

// (in-package motion_capture.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class nokovStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pose3d = null;
      this.pose2d = null;
      this.acc_linear = null;
      this.vel_linear = null;
      this.vel_theta = null;
      this.yaw_omega = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pose3d')) {
        this.pose3d = initObj.pose3d
      }
      else {
        this.pose3d = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('pose2d')) {
        this.pose2d = initObj.pose2d
      }
      else {
        this.pose2d = new geometry_msgs.msg.Pose2D();
      }
      if (initObj.hasOwnProperty('acc_linear')) {
        this.acc_linear = initObj.acc_linear
      }
      else {
        this.acc_linear = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('vel_linear')) {
        this.vel_linear = initObj.vel_linear
      }
      else {
        this.vel_linear = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('vel_theta')) {
        this.vel_theta = initObj.vel_theta
      }
      else {
        this.vel_theta = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_omega')) {
        this.yaw_omega = initObj.yaw_omega
      }
      else {
        this.yaw_omega = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type nokovStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pose3d]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose3d, buffer, bufferOffset);
    // Serialize message field [pose2d]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.pose2d, buffer, bufferOffset);
    // Serialize message field [acc_linear]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.acc_linear, buffer, bufferOffset);
    // Serialize message field [vel_linear]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.vel_linear, buffer, bufferOffset);
    // Serialize message field [vel_theta]
    bufferOffset = _serializer.float64(obj.vel_theta, buffer, bufferOffset);
    // Serialize message field [yaw_omega]
    bufferOffset = _serializer.float64(obj.yaw_omega, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type nokovStamped
    let len;
    let data = new nokovStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose3d]
    data.pose3d = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose2d]
    data.pose2d = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [acc_linear]
    data.acc_linear = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [vel_linear]
    data.vel_linear = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [vel_theta]
    data.vel_theta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_omega]
    data.yaw_omega = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 144;
  }

  static datatype() {
    // Returns string type for a message object
    return 'motion_capture/nokovStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9c50287cd23c009f37db021e9ac09647';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    geometry_msgs/Pose pose3d
    geometry_msgs/Pose2D pose2d
    geometry_msgs/Vector3 acc_linear
    geometry_msgs/Vector3 vel_linear
    float64 vel_theta
    float64 yaw_omega
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Pose2D
    # Deprecated
    # Please use the full 3D pose.
    
    # In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.
    
    # If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.
    
    
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new nokovStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pose3d !== undefined) {
      resolved.pose3d = geometry_msgs.msg.Pose.Resolve(msg.pose3d)
    }
    else {
      resolved.pose3d = new geometry_msgs.msg.Pose()
    }

    if (msg.pose2d !== undefined) {
      resolved.pose2d = geometry_msgs.msg.Pose2D.Resolve(msg.pose2d)
    }
    else {
      resolved.pose2d = new geometry_msgs.msg.Pose2D()
    }

    if (msg.acc_linear !== undefined) {
      resolved.acc_linear = geometry_msgs.msg.Vector3.Resolve(msg.acc_linear)
    }
    else {
      resolved.acc_linear = new geometry_msgs.msg.Vector3()
    }

    if (msg.vel_linear !== undefined) {
      resolved.vel_linear = geometry_msgs.msg.Vector3.Resolve(msg.vel_linear)
    }
    else {
      resolved.vel_linear = new geometry_msgs.msg.Vector3()
    }

    if (msg.vel_theta !== undefined) {
      resolved.vel_theta = msg.vel_theta;
    }
    else {
      resolved.vel_theta = 0.0
    }

    if (msg.yaw_omega !== undefined) {
      resolved.yaw_omega = msg.yaw_omega;
    }
    else {
      resolved.yaw_omega = 0.0
    }

    return resolved;
    }
};

module.exports = nokovStamped;
