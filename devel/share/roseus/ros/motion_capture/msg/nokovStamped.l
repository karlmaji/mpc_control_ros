;; Auto-generated. Do not edit!


(when (boundp 'motion_capture::nokovStamped)
  (if (not (find-package "MOTION_CAPTURE"))
    (make-package "MOTION_CAPTURE"))
  (shadow 'nokovStamped (find-package "MOTION_CAPTURE")))
(unless (find-package "MOTION_CAPTURE::NOKOVSTAMPED")
  (make-package "MOTION_CAPTURE::NOKOVSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude nokovStamped.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass motion_capture::nokovStamped
  :super ros::object
  :slots (_header _pose3d _pose2d _acc_linear _vel_linear _vel_theta _yaw_omega ))

(defmethod motion_capture::nokovStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose3d __pose3d) (instance geometry_msgs::Pose :init))
    ((:pose2d __pose2d) (instance geometry_msgs::Pose2D :init))
    ((:acc_linear __acc_linear) (instance geometry_msgs::Vector3 :init))
    ((:vel_linear __vel_linear) (instance geometry_msgs::Vector3 :init))
    ((:vel_theta __vel_theta) 0.0)
    ((:yaw_omega __yaw_omega) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose3d __pose3d)
   (setq _pose2d __pose2d)
   (setq _acc_linear __acc_linear)
   (setq _vel_linear __vel_linear)
   (setq _vel_theta (float __vel_theta))
   (setq _yaw_omega (float __yaw_omega))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose3d
   (&rest __pose3d)
   (if (keywordp (car __pose3d))
       (send* _pose3d __pose3d)
     (progn
       (if __pose3d (setq _pose3d (car __pose3d)))
       _pose3d)))
  (:pose2d
   (&rest __pose2d)
   (if (keywordp (car __pose2d))
       (send* _pose2d __pose2d)
     (progn
       (if __pose2d (setq _pose2d (car __pose2d)))
       _pose2d)))
  (:acc_linear
   (&rest __acc_linear)
   (if (keywordp (car __acc_linear))
       (send* _acc_linear __acc_linear)
     (progn
       (if __acc_linear (setq _acc_linear (car __acc_linear)))
       _acc_linear)))
  (:vel_linear
   (&rest __vel_linear)
   (if (keywordp (car __vel_linear))
       (send* _vel_linear __vel_linear)
     (progn
       (if __vel_linear (setq _vel_linear (car __vel_linear)))
       _vel_linear)))
  (:vel_theta
   (&optional __vel_theta)
   (if __vel_theta (setq _vel_theta __vel_theta)) _vel_theta)
  (:yaw_omega
   (&optional __yaw_omega)
   (if __yaw_omega (setq _yaw_omega __yaw_omega)) _yaw_omega)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _pose3d
    (send _pose3d :serialization-length)
    ;; geometry_msgs/Pose2D _pose2d
    (send _pose2d :serialization-length)
    ;; geometry_msgs/Vector3 _acc_linear
    (send _acc_linear :serialization-length)
    ;; geometry_msgs/Vector3 _vel_linear
    (send _vel_linear :serialization-length)
    ;; float64 _vel_theta
    8
    ;; float64 _yaw_omega
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _pose3d
       (send _pose3d :serialize s)
     ;; geometry_msgs/Pose2D _pose2d
       (send _pose2d :serialize s)
     ;; geometry_msgs/Vector3 _acc_linear
       (send _acc_linear :serialize s)
     ;; geometry_msgs/Vector3 _vel_linear
       (send _vel_linear :serialize s)
     ;; float64 _vel_theta
       (sys::poke _vel_theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw_omega
       (sys::poke _yaw_omega (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _pose3d
     (send _pose3d :deserialize buf ptr-) (incf ptr- (send _pose3d :serialization-length))
   ;; geometry_msgs/Pose2D _pose2d
     (send _pose2d :deserialize buf ptr-) (incf ptr- (send _pose2d :serialization-length))
   ;; geometry_msgs/Vector3 _acc_linear
     (send _acc_linear :deserialize buf ptr-) (incf ptr- (send _acc_linear :serialization-length))
   ;; geometry_msgs/Vector3 _vel_linear
     (send _vel_linear :deserialize buf ptr-) (incf ptr- (send _vel_linear :serialization-length))
   ;; float64 _vel_theta
     (setq _vel_theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw_omega
     (setq _yaw_omega (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get motion_capture::nokovStamped :md5sum-) "9c50287cd23c009f37db021e9ac09647")
(setf (get motion_capture::nokovStamped :datatype-) "motion_capture/nokovStamped")
(setf (get motion_capture::nokovStamped :definition-)
      "Header header
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
")



(provide :motion_capture/nokovStamped "9c50287cd23c009f37db021e9ac09647")


