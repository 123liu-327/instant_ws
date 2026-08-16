; Auto-generated. Do not edit!


(cl:in-package ucar_qr_decoder-msg)


;//! \htmlinclude ZBarDecodeRequest.msg.html

(cl:defclass <ZBarDecodeRequest> (roslisp-msg-protocol:ros-message)
  ((job_id
    :reader job_id
    :initarg :job_id
    :type cl:integer
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass ZBarDecodeRequest (<ZBarDecodeRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ZBarDecodeRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ZBarDecodeRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucar_qr_decoder-msg:<ZBarDecodeRequest> is deprecated: use ucar_qr_decoder-msg:ZBarDecodeRequest instead.")))

(cl:ensure-generic-function 'job_id-val :lambda-list '(m))
(cl:defmethod job_id-val ((m <ZBarDecodeRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_qr_decoder-msg:job_id-val is deprecated.  Use ucar_qr_decoder-msg:job_id instead.")
  (job_id m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <ZBarDecodeRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_qr_decoder-msg:mode-val is deprecated.  Use ucar_qr_decoder-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <ZBarDecodeRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_qr_decoder-msg:image-val is deprecated.  Use ucar_qr_decoder-msg:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ZBarDecodeRequest>)))
    "Constants for message type '<ZBarDecodeRequest>"
  '((:MODE_RAW_ONLY . 0)
    (:MODE_ENHANCED_ONLY . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ZBarDecodeRequest)))
    "Constants for message type 'ZBarDecodeRequest"
  '((:MODE_RAW_ONLY . 0)
    (:MODE_ENHANCED_ONLY . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ZBarDecodeRequest>) ostream)
  "Serializes a message object of type '<ZBarDecodeRequest>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ZBarDecodeRequest>) istream)
  "Deserializes a message object of type '<ZBarDecodeRequest>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ZBarDecodeRequest>)))
  "Returns string type for a message object of type '<ZBarDecodeRequest>"
  "ucar_qr_decoder/ZBarDecodeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ZBarDecodeRequest)))
  "Returns string type for a message object of type 'ZBarDecodeRequest"
  "ucar_qr_decoder/ZBarDecodeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ZBarDecodeRequest>)))
  "Returns md5sum for a message object of type '<ZBarDecodeRequest>"
  "42e4dcb65da54382e5d3347c34722c08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ZBarDecodeRequest)))
  "Returns md5sum for a message object of type 'ZBarDecodeRequest"
  "42e4dcb65da54382e5d3347c34722c08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ZBarDecodeRequest>)))
  "Returns full string definition for message of type '<ZBarDecodeRequest>"
  (cl:format cl:nil "uint8 MODE_RAW_ONLY=0~%uint8 MODE_ENHANCED_ONLY=1~%~%uint64 job_id~%uint8 mode~%sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ZBarDecodeRequest)))
  "Returns full string definition for message of type 'ZBarDecodeRequest"
  (cl:format cl:nil "uint8 MODE_RAW_ONLY=0~%uint8 MODE_ENHANCED_ONLY=1~%~%uint64 job_id~%uint8 mode~%sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ZBarDecodeRequest>))
  (cl:+ 0
     8
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ZBarDecodeRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'ZBarDecodeRequest
    (cl:cons ':job_id (job_id msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':image (image msg))
))
