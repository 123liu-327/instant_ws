; Auto-generated. Do not edit!


(cl:in-package ucar_2026_competition_speech-srv)


;//! \htmlinclude Announce-request.msg.html

(cl:defclass <Announce-request> (roslisp-msg-protocol:ros-message)
  ((event
    :reader event
    :initarg :event
    :type cl:string
    :initform "")
   (item
    :reader item
    :initarg :item
    :type cl:string
    :initform "")
   (workshop
    :reader workshop
    :initarg :workshop
    :type cl:string
    :initform "")
   (decision
    :reader decision
    :initarg :decision
    :type cl:string
    :initform "")
   (text
    :reader text
    :initarg :text
    :type cl:string
    :initform "")
   (wait
    :reader wait
    :initarg :wait
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Announce-request (<Announce-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Announce-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Announce-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucar_2026_competition_speech-srv:<Announce-request> is deprecated: use ucar_2026_competition_speech-srv:Announce-request instead.")))

(cl:ensure-generic-function 'event-val :lambda-list '(m))
(cl:defmethod event-val ((m <Announce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_competition_speech-srv:event-val is deprecated.  Use ucar_2026_competition_speech-srv:event instead.")
  (event m))

(cl:ensure-generic-function 'item-val :lambda-list '(m))
(cl:defmethod item-val ((m <Announce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_competition_speech-srv:item-val is deprecated.  Use ucar_2026_competition_speech-srv:item instead.")
  (item m))

(cl:ensure-generic-function 'workshop-val :lambda-list '(m))
(cl:defmethod workshop-val ((m <Announce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_competition_speech-srv:workshop-val is deprecated.  Use ucar_2026_competition_speech-srv:workshop instead.")
  (workshop m))

(cl:ensure-generic-function 'decision-val :lambda-list '(m))
(cl:defmethod decision-val ((m <Announce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_competition_speech-srv:decision-val is deprecated.  Use ucar_2026_competition_speech-srv:decision instead.")
  (decision m))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <Announce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_competition_speech-srv:text-val is deprecated.  Use ucar_2026_competition_speech-srv:text instead.")
  (text m))

(cl:ensure-generic-function 'wait-val :lambda-list '(m))
(cl:defmethod wait-val ((m <Announce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_competition_speech-srv:wait-val is deprecated.  Use ucar_2026_competition_speech-srv:wait instead.")
  (wait m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Announce-request>) ostream)
  "Serializes a message object of type '<Announce-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'event))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'event))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'item))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'item))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'workshop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'workshop))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'decision))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'decision))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'wait) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Announce-request>) istream)
  "Deserializes a message object of type '<Announce-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'event) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'event) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'item) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'item) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'workshop) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'workshop) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'decision) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'decision) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'wait) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Announce-request>)))
  "Returns string type for a service object of type '<Announce-request>"
  "ucar_2026_competition_speech/AnnounceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Announce-request)))
  "Returns string type for a service object of type 'Announce-request"
  "ucar_2026_competition_speech/AnnounceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Announce-request>)))
  "Returns md5sum for a message object of type '<Announce-request>"
  "f3261cee1e2a84f216672d4d9f69791a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Announce-request)))
  "Returns md5sum for a message object of type 'Announce-request"
  "f3261cee1e2a84f216672d4d9f69791a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Announce-request>)))
  "Returns full string definition for message of type '<Announce-request>"
  (cl:format cl:nil "# Competition event: task1, task2, task3, task4, task5, or custom.~%string event~%# Used by task2 and task3.~%string item~%string workshop~%# Used by task4. Accepted aliases include left/right/straight/stop and Chinese names.~%string decision~%# Required by task1 and custom. Ignored by the fixed task2-task5 templates.~%string text~%# Wait until the conservative estimated playback duration has elapsed.~%bool wait~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Announce-request)))
  "Returns full string definition for message of type 'Announce-request"
  (cl:format cl:nil "# Competition event: task1, task2, task3, task4, task5, or custom.~%string event~%# Used by task2 and task3.~%string item~%string workshop~%# Used by task4. Accepted aliases include left/right/straight/stop and Chinese names.~%string decision~%# Required by task1 and custom. Ignored by the fixed task2-task5 templates.~%string text~%# Wait until the conservative estimated playback duration has elapsed.~%bool wait~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Announce-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'event))
     4 (cl:length (cl:slot-value msg 'item))
     4 (cl:length (cl:slot-value msg 'workshop))
     4 (cl:length (cl:slot-value msg 'decision))
     4 (cl:length (cl:slot-value msg 'text))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Announce-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Announce-request
    (cl:cons ':event (event msg))
    (cl:cons ':item (item msg))
    (cl:cons ':workshop (workshop msg))
    (cl:cons ':decision (decision msg))
    (cl:cons ':text (text msg))
    (cl:cons ':wait (wait msg))
))
;//! \htmlinclude Announce-response.msg.html

(cl:defclass <Announce-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (speech_text
    :reader speech_text
    :initarg :speech_text
    :type cl:string
    :initform "")
   (estimated_duration
    :reader estimated_duration
    :initarg :estimated_duration
    :type cl:float
    :initform 0.0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass Announce-response (<Announce-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Announce-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Announce-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucar_2026_competition_speech-srv:<Announce-response> is deprecated: use ucar_2026_competition_speech-srv:Announce-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Announce-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_competition_speech-srv:success-val is deprecated.  Use ucar_2026_competition_speech-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'speech_text-val :lambda-list '(m))
(cl:defmethod speech_text-val ((m <Announce-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_competition_speech-srv:speech_text-val is deprecated.  Use ucar_2026_competition_speech-srv:speech_text instead.")
  (speech_text m))

(cl:ensure-generic-function 'estimated_duration-val :lambda-list '(m))
(cl:defmethod estimated_duration-val ((m <Announce-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_competition_speech-srv:estimated_duration-val is deprecated.  Use ucar_2026_competition_speech-srv:estimated_duration instead.")
  (estimated_duration m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <Announce-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_competition_speech-srv:message-val is deprecated.  Use ucar_2026_competition_speech-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Announce-response>) ostream)
  "Serializes a message object of type '<Announce-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'speech_text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'speech_text))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'estimated_duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Announce-response>) istream)
  "Deserializes a message object of type '<Announce-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speech_text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'speech_text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'estimated_duration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Announce-response>)))
  "Returns string type for a service object of type '<Announce-response>"
  "ucar_2026_competition_speech/AnnounceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Announce-response)))
  "Returns string type for a service object of type 'Announce-response"
  "ucar_2026_competition_speech/AnnounceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Announce-response>)))
  "Returns md5sum for a message object of type '<Announce-response>"
  "f3261cee1e2a84f216672d4d9f69791a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Announce-response)))
  "Returns md5sum for a message object of type 'Announce-response"
  "f3261cee1e2a84f216672d4d9f69791a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Announce-response>)))
  "Returns full string definition for message of type '<Announce-response>"
  (cl:format cl:nil "bool success~%string speech_text~%float32 estimated_duration~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Announce-response)))
  "Returns full string definition for message of type 'Announce-response"
  (cl:format cl:nil "bool success~%string speech_text~%float32 estimated_duration~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Announce-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'speech_text))
     4
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Announce-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Announce-response
    (cl:cons ':success (success msg))
    (cl:cons ':speech_text (speech_text msg))
    (cl:cons ':estimated_duration (estimated_duration msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Announce)))
  'Announce-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Announce)))
  'Announce-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Announce)))
  "Returns string type for a service object of type '<Announce>"
  "ucar_2026_competition_speech/Announce")