; Auto-generated. Do not edit!


(cl:in-package ucar_2026_smart_factory_llm-srv)


;//! \htmlinclude ReasonPickupOrder-request.msg.html

(cl:defclass <ReasonPickupOrder-request> (roslisp-msg-protocol:ros-message)
  ((item_a
    :reader item_a
    :initarg :item_a
    :type cl:string
    :initform "")
   (item_b
    :reader item_b
    :initarg :item_b
    :type cl:string
    :initform "")
   (item_c
    :reader item_c
    :initarg :item_c
    :type cl:string
    :initform "")
   (voice_instruction
    :reader voice_instruction
    :initarg :voice_instruction
    :type cl:string
    :initform ""))
)

(cl:defclass ReasonPickupOrder-request (<ReasonPickupOrder-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReasonPickupOrder-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReasonPickupOrder-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucar_2026_smart_factory_llm-srv:<ReasonPickupOrder-request> is deprecated: use ucar_2026_smart_factory_llm-srv:ReasonPickupOrder-request instead.")))

(cl:ensure-generic-function 'item_a-val :lambda-list '(m))
(cl:defmethod item_a-val ((m <ReasonPickupOrder-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:item_a-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:item_a instead.")
  (item_a m))

(cl:ensure-generic-function 'item_b-val :lambda-list '(m))
(cl:defmethod item_b-val ((m <ReasonPickupOrder-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:item_b-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:item_b instead.")
  (item_b m))

(cl:ensure-generic-function 'item_c-val :lambda-list '(m))
(cl:defmethod item_c-val ((m <ReasonPickupOrder-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:item_c-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:item_c instead.")
  (item_c m))

(cl:ensure-generic-function 'voice_instruction-val :lambda-list '(m))
(cl:defmethod voice_instruction-val ((m <ReasonPickupOrder-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:voice_instruction-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:voice_instruction instead.")
  (voice_instruction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReasonPickupOrder-request>) ostream)
  "Serializes a message object of type '<ReasonPickupOrder-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'item_a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'item_a))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'item_b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'item_b))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'item_c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'item_c))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'voice_instruction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'voice_instruction))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReasonPickupOrder-request>) istream)
  "Deserializes a message object of type '<ReasonPickupOrder-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'item_a) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'item_a) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'item_b) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'item_b) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'item_c) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'item_c) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'voice_instruction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'voice_instruction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReasonPickupOrder-request>)))
  "Returns string type for a service object of type '<ReasonPickupOrder-request>"
  "ucar_2026_smart_factory_llm/ReasonPickupOrderRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReasonPickupOrder-request)))
  "Returns string type for a service object of type 'ReasonPickupOrder-request"
  "ucar_2026_smart_factory_llm/ReasonPickupOrderRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReasonPickupOrder-request>)))
  "Returns md5sum for a message object of type '<ReasonPickupOrder-request>"
  "6f2e4969829f8723b74f2ea5cfb7ffc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReasonPickupOrder-request)))
  "Returns md5sum for a message object of type 'ReasonPickupOrder-request"
  "6f2e4969829f8723b74f2ea5cfb7ffc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReasonPickupOrder-request>)))
  "Returns full string definition for message of type '<ReasonPickupOrder-request>"
  (cl:format cl:nil "# 三个二维码解析得到的子类名称（与扫描顺序一致即可）~%string item_a~%string item_b~%string item_c~%# 语音唤醒后的完整指令文本（含目标大类）~%string voice_instruction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReasonPickupOrder-request)))
  "Returns full string definition for message of type 'ReasonPickupOrder-request"
  (cl:format cl:nil "# 三个二维码解析得到的子类名称（与扫描顺序一致即可）~%string item_a~%string item_b~%string item_c~%# 语音唤醒后的完整指令文本（含目标大类）~%string voice_instruction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReasonPickupOrder-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'item_a))
     4 (cl:length (cl:slot-value msg 'item_b))
     4 (cl:length (cl:slot-value msg 'item_c))
     4 (cl:length (cl:slot-value msg 'voice_instruction))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReasonPickupOrder-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReasonPickupOrder-request
    (cl:cons ':item_a (item_a msg))
    (cl:cons ':item_b (item_b msg))
    (cl:cons ':item_c (item_c msg))
    (cl:cons ':voice_instruction (voice_instruction msg))
))
;//! \htmlinclude ReasonPickupOrder-response.msg.html

(cl:defclass <ReasonPickupOrder-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (error_message
    :reader error_message
    :initarg :error_message
    :type cl:string
    :initform "")
   (announcement_physical
    :reader announcement_physical
    :initarg :announcement_physical
    :type cl:string
    :initform "")
   (announcement_simulation
    :reader announcement_simulation
    :initarg :announcement_simulation
    :type cl:string
    :initform "")
   (announcement_full
    :reader announcement_full
    :initarg :announcement_full
    :type cl:string
    :initform "")
   (pickup_item
    :reader pickup_item
    :initarg :pickup_item
    :type cl:string
    :initform "")
   (pickup_major
    :reader pickup_major
    :initarg :pickup_major
    :type cl:string
    :initform "")
   (pickup_workshop
    :reader pickup_workshop
    :initarg :pickup_workshop
    :type cl:string
    :initform "")
   (sim_item
    :reader sim_item
    :initarg :sim_item
    :type cl:string
    :initform "")
   (sim_major
    :reader sim_major
    :initarg :sim_major
    :type cl:string
    :initform "")
   (sim_workshop
    :reader sim_workshop
    :initarg :sim_workshop
    :type cl:string
    :initform "")
   (raw_model_reply
    :reader raw_model_reply
    :initarg :raw_model_reply
    :type cl:string
    :initform ""))
)

(cl:defclass ReasonPickupOrder-response (<ReasonPickupOrder-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReasonPickupOrder-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReasonPickupOrder-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucar_2026_smart_factory_llm-srv:<ReasonPickupOrder-response> is deprecated: use ucar_2026_smart_factory_llm-srv:ReasonPickupOrder-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:success-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'error_message-val :lambda-list '(m))
(cl:defmethod error_message-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:error_message-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:error_message instead.")
  (error_message m))

(cl:ensure-generic-function 'announcement_physical-val :lambda-list '(m))
(cl:defmethod announcement_physical-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:announcement_physical-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:announcement_physical instead.")
  (announcement_physical m))

(cl:ensure-generic-function 'announcement_simulation-val :lambda-list '(m))
(cl:defmethod announcement_simulation-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:announcement_simulation-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:announcement_simulation instead.")
  (announcement_simulation m))

(cl:ensure-generic-function 'announcement_full-val :lambda-list '(m))
(cl:defmethod announcement_full-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:announcement_full-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:announcement_full instead.")
  (announcement_full m))

(cl:ensure-generic-function 'pickup_item-val :lambda-list '(m))
(cl:defmethod pickup_item-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:pickup_item-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:pickup_item instead.")
  (pickup_item m))

(cl:ensure-generic-function 'pickup_major-val :lambda-list '(m))
(cl:defmethod pickup_major-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:pickup_major-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:pickup_major instead.")
  (pickup_major m))

(cl:ensure-generic-function 'pickup_workshop-val :lambda-list '(m))
(cl:defmethod pickup_workshop-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:pickup_workshop-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:pickup_workshop instead.")
  (pickup_workshop m))

(cl:ensure-generic-function 'sim_item-val :lambda-list '(m))
(cl:defmethod sim_item-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:sim_item-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:sim_item instead.")
  (sim_item m))

(cl:ensure-generic-function 'sim_major-val :lambda-list '(m))
(cl:defmethod sim_major-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:sim_major-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:sim_major instead.")
  (sim_major m))

(cl:ensure-generic-function 'sim_workshop-val :lambda-list '(m))
(cl:defmethod sim_workshop-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:sim_workshop-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:sim_workshop instead.")
  (sim_workshop m))

(cl:ensure-generic-function 'raw_model_reply-val :lambda-list '(m))
(cl:defmethod raw_model_reply-val ((m <ReasonPickupOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_2026_smart_factory_llm-srv:raw_model_reply-val is deprecated.  Use ucar_2026_smart_factory_llm-srv:raw_model_reply instead.")
  (raw_model_reply m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReasonPickupOrder-response>) ostream)
  "Serializes a message object of type '<ReasonPickupOrder-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'error_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'error_message))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'announcement_physical))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'announcement_physical))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'announcement_simulation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'announcement_simulation))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'announcement_full))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'announcement_full))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pickup_item))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pickup_item))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pickup_major))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pickup_major))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pickup_workshop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pickup_workshop))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'sim_item))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'sim_item))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'sim_major))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'sim_major))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'sim_workshop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'sim_workshop))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'raw_model_reply))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'raw_model_reply))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReasonPickupOrder-response>) istream)
  "Deserializes a message object of type '<ReasonPickupOrder-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'error_message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'announcement_physical) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'announcement_physical) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'announcement_simulation) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'announcement_simulation) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'announcement_full) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'announcement_full) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pickup_item) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pickup_item) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pickup_major) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pickup_major) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pickup_workshop) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pickup_workshop) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sim_item) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'sim_item) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sim_major) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'sim_major) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sim_workshop) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'sim_workshop) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'raw_model_reply) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'raw_model_reply) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReasonPickupOrder-response>)))
  "Returns string type for a service object of type '<ReasonPickupOrder-response>"
  "ucar_2026_smart_factory_llm/ReasonPickupOrderResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReasonPickupOrder-response)))
  "Returns string type for a service object of type 'ReasonPickupOrder-response"
  "ucar_2026_smart_factory_llm/ReasonPickupOrderResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReasonPickupOrder-response>)))
  "Returns md5sum for a message object of type '<ReasonPickupOrder-response>"
  "6f2e4969829f8723b74f2ea5cfb7ffc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReasonPickupOrder-response)))
  "Returns md5sum for a message object of type 'ReasonPickupOrder-response"
  "6f2e4969829f8723b74f2ea5cfb7ffc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReasonPickupOrder-response>)))
  "Returns full string definition for message of type '<ReasonPickupOrder-response>"
  (cl:format cl:nil "bool success~%string error_message~%# 赛方要求的播报格式（可拆成两句给 TTS）~%string announcement_physical~%string announcement_simulation~%string announcement_full~%# 结构化结果，便于下游记录 / 调试~%string pickup_item~%string pickup_major~%string pickup_workshop~%string sim_item~%string sim_major~%string sim_workshop~%string raw_model_reply~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReasonPickupOrder-response)))
  "Returns full string definition for message of type 'ReasonPickupOrder-response"
  (cl:format cl:nil "bool success~%string error_message~%# 赛方要求的播报格式（可拆成两句给 TTS）~%string announcement_physical~%string announcement_simulation~%string announcement_full~%# 结构化结果，便于下游记录 / 调试~%string pickup_item~%string pickup_major~%string pickup_workshop~%string sim_item~%string sim_major~%string sim_workshop~%string raw_model_reply~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReasonPickupOrder-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'error_message))
     4 (cl:length (cl:slot-value msg 'announcement_physical))
     4 (cl:length (cl:slot-value msg 'announcement_simulation))
     4 (cl:length (cl:slot-value msg 'announcement_full))
     4 (cl:length (cl:slot-value msg 'pickup_item))
     4 (cl:length (cl:slot-value msg 'pickup_major))
     4 (cl:length (cl:slot-value msg 'pickup_workshop))
     4 (cl:length (cl:slot-value msg 'sim_item))
     4 (cl:length (cl:slot-value msg 'sim_major))
     4 (cl:length (cl:slot-value msg 'sim_workshop))
     4 (cl:length (cl:slot-value msg 'raw_model_reply))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReasonPickupOrder-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReasonPickupOrder-response
    (cl:cons ':success (success msg))
    (cl:cons ':error_message (error_message msg))
    (cl:cons ':announcement_physical (announcement_physical msg))
    (cl:cons ':announcement_simulation (announcement_simulation msg))
    (cl:cons ':announcement_full (announcement_full msg))
    (cl:cons ':pickup_item (pickup_item msg))
    (cl:cons ':pickup_major (pickup_major msg))
    (cl:cons ':pickup_workshop (pickup_workshop msg))
    (cl:cons ':sim_item (sim_item msg))
    (cl:cons ':sim_major (sim_major msg))
    (cl:cons ':sim_workshop (sim_workshop msg))
    (cl:cons ':raw_model_reply (raw_model_reply msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReasonPickupOrder)))
  'ReasonPickupOrder-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReasonPickupOrder)))
  'ReasonPickupOrder-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReasonPickupOrder)))
  "Returns string type for a service object of type '<ReasonPickupOrder>"
  "ucar_2026_smart_factory_llm/ReasonPickupOrder")