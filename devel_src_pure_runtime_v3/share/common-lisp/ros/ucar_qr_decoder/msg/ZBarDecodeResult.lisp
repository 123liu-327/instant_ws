; Auto-generated. Do not edit!


(cl:in-package ucar_qr_decoder-msg)


;//! \htmlinclude ZBarDecodeResult.msg.html

(cl:defclass <ZBarDecodeResult> (roslisp-msg-protocol:ros-message)
  ((job_id
    :reader job_id
    :initarg :job_id
    :type cl:integer
    :initform 0)
   (decoded
    :reader decoded
    :initarg :decoded
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (decode_ms
    :reader decode_ms
    :initarg :decode_ms
    :type cl:float
    :initform 0.0)
   (stage
    :reader stage
    :initarg :stage
    :type cl:string
    :initform ""))
)

(cl:defclass ZBarDecodeResult (<ZBarDecodeResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ZBarDecodeResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ZBarDecodeResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucar_qr_decoder-msg:<ZBarDecodeResult> is deprecated: use ucar_qr_decoder-msg:ZBarDecodeResult instead.")))

(cl:ensure-generic-function 'job_id-val :lambda-list '(m))
(cl:defmethod job_id-val ((m <ZBarDecodeResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_qr_decoder-msg:job_id-val is deprecated.  Use ucar_qr_decoder-msg:job_id instead.")
  (job_id m))

(cl:ensure-generic-function 'decoded-val :lambda-list '(m))
(cl:defmethod decoded-val ((m <ZBarDecodeResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_qr_decoder-msg:decoded-val is deprecated.  Use ucar_qr_decoder-msg:decoded instead.")
  (decoded m))

(cl:ensure-generic-function 'decode_ms-val :lambda-list '(m))
(cl:defmethod decode_ms-val ((m <ZBarDecodeResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_qr_decoder-msg:decode_ms-val is deprecated.  Use ucar_qr_decoder-msg:decode_ms instead.")
  (decode_ms m))

(cl:ensure-generic-function 'stage-val :lambda-list '(m))
(cl:defmethod stage-val ((m <ZBarDecodeResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_qr_decoder-msg:stage-val is deprecated.  Use ucar_qr_decoder-msg:stage instead.")
  (stage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ZBarDecodeResult>) ostream)
  "Serializes a message object of type '<ZBarDecodeResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'job_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'job_id)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'decoded))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'decoded))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'decode_ms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'stage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'stage))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ZBarDecodeResult>) istream)
  "Deserializes a message object of type '<ZBarDecodeResult>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'job_id)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'decoded) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'decoded)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'decode_ms) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stage) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'stage) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ZBarDecodeResult>)))
  "Returns string type for a message object of type '<ZBarDecodeResult>"
  "ucar_qr_decoder/ZBarDecodeResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ZBarDecodeResult)))
  "Returns string type for a message object of type 'ZBarDecodeResult"
  "ucar_qr_decoder/ZBarDecodeResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ZBarDecodeResult>)))
  "Returns md5sum for a message object of type '<ZBarDecodeResult>"
  "41ac4872f56934370169b3f1087ab96f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ZBarDecodeResult)))
  "Returns md5sum for a message object of type 'ZBarDecodeResult"
  "41ac4872f56934370169b3f1087ab96f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ZBarDecodeResult>)))
  "Returns full string definition for message of type '<ZBarDecodeResult>"
  (cl:format cl:nil "uint64 job_id~%string[] decoded~%float32 decode_ms~%string stage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ZBarDecodeResult)))
  "Returns full string definition for message of type 'ZBarDecodeResult"
  (cl:format cl:nil "uint64 job_id~%string[] decoded~%float32 decode_ms~%string stage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ZBarDecodeResult>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'decoded) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4
     4 (cl:length (cl:slot-value msg 'stage))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ZBarDecodeResult>))
  "Converts a ROS message object to a list"
  (cl:list 'ZBarDecodeResult
    (cl:cons ':job_id (job_id msg))
    (cl:cons ':decoded (decoded msg))
    (cl:cons ':decode_ms (decode_ms msg))
    (cl:cons ':stage (stage msg))
))
