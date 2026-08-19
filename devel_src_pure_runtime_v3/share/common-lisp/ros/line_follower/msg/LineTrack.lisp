; Auto-generated. Do not edit!


(cl:in-package line_follower-msg)


;//! \htmlinclude LineTrack.msg.html

(cl:defclass <LineTrack> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (valid
    :reader valid
    :initarg :valid
    :type cl:boolean
    :initform cl:nil)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (image_width
    :reader image_width
    :initarg :image_width
    :type cl:integer
    :initform 0)
   (image_height
    :reader image_height
    :initarg :image_height
    :type cl:integer
    :initform 0)
   (center_x_px
    :reader center_x_px
    :initarg :center_x_px
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (center_y_px
    :reader center_y_px
    :initarg :center_y_px
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (lookahead_x_px
    :reader lookahead_x_px
    :initarg :lookahead_x_px
    :type cl:float
    :initform 0.0)
   (lookahead_y_px
    :reader lookahead_y_px
    :initarg :lookahead_y_px
    :type cl:float
    :initform 0.0)
   (heading_error_rad
    :reader heading_error_rad
    :initarg :heading_error_rad
    :type cl:float
    :initform 0.0))
)

(cl:defclass LineTrack (<LineTrack>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LineTrack>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LineTrack)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name line_follower-msg:<LineTrack> is deprecated: use line_follower-msg:LineTrack instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LineTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_follower-msg:header-val is deprecated.  Use line_follower-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <LineTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_follower-msg:valid-val is deprecated.  Use line_follower-msg:valid instead.")
  (valid m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <LineTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_follower-msg:confidence-val is deprecated.  Use line_follower-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'image_width-val :lambda-list '(m))
(cl:defmethod image_width-val ((m <LineTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_follower-msg:image_width-val is deprecated.  Use line_follower-msg:image_width instead.")
  (image_width m))

(cl:ensure-generic-function 'image_height-val :lambda-list '(m))
(cl:defmethod image_height-val ((m <LineTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_follower-msg:image_height-val is deprecated.  Use line_follower-msg:image_height instead.")
  (image_height m))

(cl:ensure-generic-function 'center_x_px-val :lambda-list '(m))
(cl:defmethod center_x_px-val ((m <LineTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_follower-msg:center_x_px-val is deprecated.  Use line_follower-msg:center_x_px instead.")
  (center_x_px m))

(cl:ensure-generic-function 'center_y_px-val :lambda-list '(m))
(cl:defmethod center_y_px-val ((m <LineTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_follower-msg:center_y_px-val is deprecated.  Use line_follower-msg:center_y_px instead.")
  (center_y_px m))

(cl:ensure-generic-function 'lookahead_x_px-val :lambda-list '(m))
(cl:defmethod lookahead_x_px-val ((m <LineTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_follower-msg:lookahead_x_px-val is deprecated.  Use line_follower-msg:lookahead_x_px instead.")
  (lookahead_x_px m))

(cl:ensure-generic-function 'lookahead_y_px-val :lambda-list '(m))
(cl:defmethod lookahead_y_px-val ((m <LineTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_follower-msg:lookahead_y_px-val is deprecated.  Use line_follower-msg:lookahead_y_px instead.")
  (lookahead_y_px m))

(cl:ensure-generic-function 'heading_error_rad-val :lambda-list '(m))
(cl:defmethod heading_error_rad-val ((m <LineTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_follower-msg:heading_error_rad-val is deprecated.  Use line_follower-msg:heading_error_rad instead.")
  (heading_error_rad m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LineTrack>) ostream)
  "Serializes a message object of type '<LineTrack>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_height)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'center_x_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'center_x_px))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'center_y_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'center_y_px))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lookahead_x_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lookahead_y_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading_error_rad))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LineTrack>) istream)
  "Deserializes a message object of type '<LineTrack>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'center_x_px) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'center_x_px)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'center_y_px) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'center_y_px)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lookahead_x_px) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lookahead_y_px) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_error_rad) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LineTrack>)))
  "Returns string type for a message object of type '<LineTrack>"
  "line_follower/LineTrack")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LineTrack)))
  "Returns string type for a message object of type 'LineTrack"
  "line_follower/LineTrack")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LineTrack>)))
  "Returns md5sum for a message object of type '<LineTrack>"
  "a8f2dc59e6af2e2fcb8e42affc92dd20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LineTrack)))
  "Returns md5sum for a message object of type 'LineTrack"
  "a8f2dc59e6af2e2fcb8e42affc92dd20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LineTrack>)))
  "Returns full string definition for message of type '<LineTrack>"
  (cl:format cl:nil "std_msgs/Header header~%bool valid~%float32 confidence~%uint32 image_width~%uint32 image_height~%float32[] center_x_px~%float32[] center_y_px~%float32 lookahead_x_px~%float32 lookahead_y_px~%float32 heading_error_rad~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LineTrack)))
  "Returns full string definition for message of type 'LineTrack"
  (cl:format cl:nil "std_msgs/Header header~%bool valid~%float32 confidence~%uint32 image_width~%uint32 image_height~%float32[] center_x_px~%float32[] center_y_px~%float32 lookahead_x_px~%float32 lookahead_y_px~%float32 heading_error_rad~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LineTrack>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'center_x_px) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'center_y_px) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LineTrack>))
  "Converts a ROS message object to a list"
  (cl:list 'LineTrack
    (cl:cons ':header (header msg))
    (cl:cons ':valid (valid msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':image_width (image_width msg))
    (cl:cons ':image_height (image_height msg))
    (cl:cons ':center_x_px (center_x_px msg))
    (cl:cons ':center_y_px (center_y_px msg))
    (cl:cons ':lookahead_x_px (lookahead_x_px msg))
    (cl:cons ':lookahead_y_px (lookahead_y_px msg))
    (cl:cons ':heading_error_rad (heading_error_rad msg))
))
