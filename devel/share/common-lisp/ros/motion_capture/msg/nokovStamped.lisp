; Auto-generated. Do not edit!


(cl:in-package motion_capture-msg)


;//! \htmlinclude nokovStamped.msg.html

(cl:defclass <nokovStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose3d
    :reader pose3d
    :initarg :pose3d
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (pose2d
    :reader pose2d
    :initarg :pose2d
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (acc_linear
    :reader acc_linear
    :initarg :acc_linear
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (vel_linear
    :reader vel_linear
    :initarg :vel_linear
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (vel_theta
    :reader vel_theta
    :initarg :vel_theta
    :type cl:float
    :initform 0.0)
   (yaw_omega
    :reader yaw_omega
    :initarg :yaw_omega
    :type cl:float
    :initform 0.0))
)

(cl:defclass nokovStamped (<nokovStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <nokovStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'nokovStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_capture-msg:<nokovStamped> is deprecated: use motion_capture-msg:nokovStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <nokovStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture-msg:header-val is deprecated.  Use motion_capture-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose3d-val :lambda-list '(m))
(cl:defmethod pose3d-val ((m <nokovStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture-msg:pose3d-val is deprecated.  Use motion_capture-msg:pose3d instead.")
  (pose3d m))

(cl:ensure-generic-function 'pose2d-val :lambda-list '(m))
(cl:defmethod pose2d-val ((m <nokovStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture-msg:pose2d-val is deprecated.  Use motion_capture-msg:pose2d instead.")
  (pose2d m))

(cl:ensure-generic-function 'acc_linear-val :lambda-list '(m))
(cl:defmethod acc_linear-val ((m <nokovStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture-msg:acc_linear-val is deprecated.  Use motion_capture-msg:acc_linear instead.")
  (acc_linear m))

(cl:ensure-generic-function 'vel_linear-val :lambda-list '(m))
(cl:defmethod vel_linear-val ((m <nokovStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture-msg:vel_linear-val is deprecated.  Use motion_capture-msg:vel_linear instead.")
  (vel_linear m))

(cl:ensure-generic-function 'vel_theta-val :lambda-list '(m))
(cl:defmethod vel_theta-val ((m <nokovStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture-msg:vel_theta-val is deprecated.  Use motion_capture-msg:vel_theta instead.")
  (vel_theta m))

(cl:ensure-generic-function 'yaw_omega-val :lambda-list '(m))
(cl:defmethod yaw_omega-val ((m <nokovStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture-msg:yaw_omega-val is deprecated.  Use motion_capture-msg:yaw_omega instead.")
  (yaw_omega m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <nokovStamped>) ostream)
  "Serializes a message object of type '<nokovStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose3d) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose2d) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acc_linear) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel_linear) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vel_theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw_omega))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <nokovStamped>) istream)
  "Deserializes a message object of type '<nokovStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose3d) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose2d) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acc_linear) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel_linear) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_theta) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_omega) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<nokovStamped>)))
  "Returns string type for a message object of type '<nokovStamped>"
  "motion_capture/nokovStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'nokovStamped)))
  "Returns string type for a message object of type 'nokovStamped"
  "motion_capture/nokovStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<nokovStamped>)))
  "Returns md5sum for a message object of type '<nokovStamped>"
  "9c50287cd23c009f37db021e9ac09647")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'nokovStamped)))
  "Returns md5sum for a message object of type 'nokovStamped"
  "9c50287cd23c009f37db021e9ac09647")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<nokovStamped>)))
  "Returns full string definition for message of type '<nokovStamped>"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose pose3d~%geometry_msgs/Pose2D pose2d~%geometry_msgs/Vector3 acc_linear~%geometry_msgs/Vector3 vel_linear~%float64 vel_theta~%float64 yaw_omega~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'nokovStamped)))
  "Returns full string definition for message of type 'nokovStamped"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose pose3d~%geometry_msgs/Pose2D pose2d~%geometry_msgs/Vector3 acc_linear~%geometry_msgs/Vector3 vel_linear~%float64 vel_theta~%float64 yaw_omega~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <nokovStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose3d))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose2d))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acc_linear))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel_linear))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <nokovStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'nokovStamped
    (cl:cons ':header (header msg))
    (cl:cons ':pose3d (pose3d msg))
    (cl:cons ':pose2d (pose2d msg))
    (cl:cons ':acc_linear (acc_linear msg))
    (cl:cons ':vel_linear (vel_linear msg))
    (cl:cons ':vel_theta (vel_theta msg))
    (cl:cons ':yaw_omega (yaw_omega msg))
))
