; Auto-generated. Do not edit!


(cl:in-package obstacle_gazebo-msg)


;//! \htmlinclude float64stamped.msg.html

(cl:defclass <float64stamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type (cl:vector std_msgs-msg:Float64)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:Float64 :initial-element (cl:make-instance 'std_msgs-msg:Float64))))
)

(cl:defclass float64stamped (<float64stamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <float64stamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'float64stamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name obstacle_gazebo-msg:<float64stamped> is deprecated: use obstacle_gazebo-msg:float64stamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <float64stamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader obstacle_gazebo-msg:header-val is deprecated.  Use obstacle_gazebo-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <float64stamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader obstacle_gazebo-msg:data-val is deprecated.  Use obstacle_gazebo-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <float64stamped>) ostream)
  "Serializes a message object of type '<float64stamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <float64stamped>) istream)
  "Deserializes a message object of type '<float64stamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:Float64))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<float64stamped>)))
  "Returns string type for a message object of type '<float64stamped>"
  "obstacle_gazebo/float64stamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'float64stamped)))
  "Returns string type for a message object of type 'float64stamped"
  "obstacle_gazebo/float64stamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<float64stamped>)))
  "Returns md5sum for a message object of type '<float64stamped>"
  "830039f8f5aa832134c491ccbf30ca05")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'float64stamped)))
  "Returns md5sum for a message object of type 'float64stamped"
  "830039f8f5aa832134c491ccbf30ca05")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<float64stamped>)))
  "Returns full string definition for message of type '<float64stamped>"
  (cl:format cl:nil "Header header~%std_msgs/Float64[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'float64stamped)))
  "Returns full string definition for message of type 'float64stamped"
  (cl:format cl:nil "Header header~%std_msgs/Float64[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <float64stamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <float64stamped>))
  "Converts a ROS message object to a list"
  (cl:list 'float64stamped
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
