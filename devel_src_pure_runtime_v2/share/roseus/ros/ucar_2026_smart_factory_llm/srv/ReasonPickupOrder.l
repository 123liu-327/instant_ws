;; Auto-generated. Do not edit!


(when (boundp 'ucar_2026_smart_factory_llm::ReasonPickupOrder)
  (if (not (find-package "UCAR_2026_SMART_FACTORY_LLM"))
    (make-package "UCAR_2026_SMART_FACTORY_LLM"))
  (shadow 'ReasonPickupOrder (find-package "UCAR_2026_SMART_FACTORY_LLM")))
(unless (find-package "UCAR_2026_SMART_FACTORY_LLM::REASONPICKUPORDER")
  (make-package "UCAR_2026_SMART_FACTORY_LLM::REASONPICKUPORDER"))
(unless (find-package "UCAR_2026_SMART_FACTORY_LLM::REASONPICKUPORDERREQUEST")
  (make-package "UCAR_2026_SMART_FACTORY_LLM::REASONPICKUPORDERREQUEST"))
(unless (find-package "UCAR_2026_SMART_FACTORY_LLM::REASONPICKUPORDERRESPONSE")
  (make-package "UCAR_2026_SMART_FACTORY_LLM::REASONPICKUPORDERRESPONSE"))

(in-package "ROS")





(defclass ucar_2026_smart_factory_llm::ReasonPickupOrderRequest
  :super ros::object
  :slots (_item_a _item_b _item_c _voice_instruction ))

(defmethod ucar_2026_smart_factory_llm::ReasonPickupOrderRequest
  (:init
   (&key
    ((:item_a __item_a) "")
    ((:item_b __item_b) "")
    ((:item_c __item_c) "")
    ((:voice_instruction __voice_instruction) "")
    )
   (send-super :init)
   (setq _item_a (string __item_a))
   (setq _item_b (string __item_b))
   (setq _item_c (string __item_c))
   (setq _voice_instruction (string __voice_instruction))
   self)
  (:item_a
   (&optional __item_a)
   (if __item_a (setq _item_a __item_a)) _item_a)
  (:item_b
   (&optional __item_b)
   (if __item_b (setq _item_b __item_b)) _item_b)
  (:item_c
   (&optional __item_c)
   (if __item_c (setq _item_c __item_c)) _item_c)
  (:voice_instruction
   (&optional __voice_instruction)
   (if __voice_instruction (setq _voice_instruction __voice_instruction)) _voice_instruction)
  (:serialization-length
   ()
   (+
    ;; string _item_a
    4 (length _item_a)
    ;; string _item_b
    4 (length _item_b)
    ;; string _item_c
    4 (length _item_c)
    ;; string _voice_instruction
    4 (length _voice_instruction)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _item_a
       (write-long (length _item_a) s) (princ _item_a s)
     ;; string _item_b
       (write-long (length _item_b) s) (princ _item_b s)
     ;; string _item_c
       (write-long (length _item_c) s) (princ _item_c s)
     ;; string _voice_instruction
       (write-long (length _voice_instruction) s) (princ _voice_instruction s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _item_a
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _item_a (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _item_b
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _item_b (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _item_c
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _item_c (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _voice_instruction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _voice_instruction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ucar_2026_smart_factory_llm::ReasonPickupOrderResponse
  :super ros::object
  :slots (_success _error_message _announcement_physical _announcement_simulation _announcement_full _pickup_item _pickup_major _pickup_workshop _sim_item _sim_major _sim_workshop _raw_model_reply ))

(defmethod ucar_2026_smart_factory_llm::ReasonPickupOrderResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:error_message __error_message) "")
    ((:announcement_physical __announcement_physical) "")
    ((:announcement_simulation __announcement_simulation) "")
    ((:announcement_full __announcement_full) "")
    ((:pickup_item __pickup_item) "")
    ((:pickup_major __pickup_major) "")
    ((:pickup_workshop __pickup_workshop) "")
    ((:sim_item __sim_item) "")
    ((:sim_major __sim_major) "")
    ((:sim_workshop __sim_workshop) "")
    ((:raw_model_reply __raw_model_reply) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _error_message (string __error_message))
   (setq _announcement_physical (string __announcement_physical))
   (setq _announcement_simulation (string __announcement_simulation))
   (setq _announcement_full (string __announcement_full))
   (setq _pickup_item (string __pickup_item))
   (setq _pickup_major (string __pickup_major))
   (setq _pickup_workshop (string __pickup_workshop))
   (setq _sim_item (string __sim_item))
   (setq _sim_major (string __sim_major))
   (setq _sim_workshop (string __sim_workshop))
   (setq _raw_model_reply (string __raw_model_reply))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:error_message
   (&optional __error_message)
   (if __error_message (setq _error_message __error_message)) _error_message)
  (:announcement_physical
   (&optional __announcement_physical)
   (if __announcement_physical (setq _announcement_physical __announcement_physical)) _announcement_physical)
  (:announcement_simulation
   (&optional __announcement_simulation)
   (if __announcement_simulation (setq _announcement_simulation __announcement_simulation)) _announcement_simulation)
  (:announcement_full
   (&optional __announcement_full)
   (if __announcement_full (setq _announcement_full __announcement_full)) _announcement_full)
  (:pickup_item
   (&optional __pickup_item)
   (if __pickup_item (setq _pickup_item __pickup_item)) _pickup_item)
  (:pickup_major
   (&optional __pickup_major)
   (if __pickup_major (setq _pickup_major __pickup_major)) _pickup_major)
  (:pickup_workshop
   (&optional __pickup_workshop)
   (if __pickup_workshop (setq _pickup_workshop __pickup_workshop)) _pickup_workshop)
  (:sim_item
   (&optional __sim_item)
   (if __sim_item (setq _sim_item __sim_item)) _sim_item)
  (:sim_major
   (&optional __sim_major)
   (if __sim_major (setq _sim_major __sim_major)) _sim_major)
  (:sim_workshop
   (&optional __sim_workshop)
   (if __sim_workshop (setq _sim_workshop __sim_workshop)) _sim_workshop)
  (:raw_model_reply
   (&optional __raw_model_reply)
   (if __raw_model_reply (setq _raw_model_reply __raw_model_reply)) _raw_model_reply)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _error_message
    4 (length _error_message)
    ;; string _announcement_physical
    4 (length _announcement_physical)
    ;; string _announcement_simulation
    4 (length _announcement_simulation)
    ;; string _announcement_full
    4 (length _announcement_full)
    ;; string _pickup_item
    4 (length _pickup_item)
    ;; string _pickup_major
    4 (length _pickup_major)
    ;; string _pickup_workshop
    4 (length _pickup_workshop)
    ;; string _sim_item
    4 (length _sim_item)
    ;; string _sim_major
    4 (length _sim_major)
    ;; string _sim_workshop
    4 (length _sim_workshop)
    ;; string _raw_model_reply
    4 (length _raw_model_reply)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _error_message
       (write-long (length _error_message) s) (princ _error_message s)
     ;; string _announcement_physical
       (write-long (length _announcement_physical) s) (princ _announcement_physical s)
     ;; string _announcement_simulation
       (write-long (length _announcement_simulation) s) (princ _announcement_simulation s)
     ;; string _announcement_full
       (write-long (length _announcement_full) s) (princ _announcement_full s)
     ;; string _pickup_item
       (write-long (length _pickup_item) s) (princ _pickup_item s)
     ;; string _pickup_major
       (write-long (length _pickup_major) s) (princ _pickup_major s)
     ;; string _pickup_workshop
       (write-long (length _pickup_workshop) s) (princ _pickup_workshop s)
     ;; string _sim_item
       (write-long (length _sim_item) s) (princ _sim_item s)
     ;; string _sim_major
       (write-long (length _sim_major) s) (princ _sim_major s)
     ;; string _sim_workshop
       (write-long (length _sim_workshop) s) (princ _sim_workshop s)
     ;; string _raw_model_reply
       (write-long (length _raw_model_reply) s) (princ _raw_model_reply s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _error_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _error_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _announcement_physical
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _announcement_physical (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _announcement_simulation
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _announcement_simulation (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _announcement_full
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _announcement_full (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _pickup_item
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _pickup_item (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _pickup_major
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _pickup_major (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _pickup_workshop
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _pickup_workshop (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _sim_item
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _sim_item (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _sim_major
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _sim_major (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _sim_workshop
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _sim_workshop (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _raw_model_reply
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _raw_model_reply (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ucar_2026_smart_factory_llm::ReasonPickupOrder
  :super ros::object
  :slots ())

(setf (get ucar_2026_smart_factory_llm::ReasonPickupOrder :md5sum-) "6f2e4969829f8723b74f2ea5cfb7ffc4")
(setf (get ucar_2026_smart_factory_llm::ReasonPickupOrder :datatype-) "ucar_2026_smart_factory_llm/ReasonPickupOrder")
(setf (get ucar_2026_smart_factory_llm::ReasonPickupOrder :request) ucar_2026_smart_factory_llm::ReasonPickupOrderRequest)
(setf (get ucar_2026_smart_factory_llm::ReasonPickupOrder :response) ucar_2026_smart_factory_llm::ReasonPickupOrderResponse)

(defmethod ucar_2026_smart_factory_llm::ReasonPickupOrderRequest
  (:response () (instance ucar_2026_smart_factory_llm::ReasonPickupOrderResponse :init)))

(setf (get ucar_2026_smart_factory_llm::ReasonPickupOrderRequest :md5sum-) "6f2e4969829f8723b74f2ea5cfb7ffc4")
(setf (get ucar_2026_smart_factory_llm::ReasonPickupOrderRequest :datatype-) "ucar_2026_smart_factory_llm/ReasonPickupOrderRequest")
(setf (get ucar_2026_smart_factory_llm::ReasonPickupOrderRequest :definition-)
      "# 三个二维码解析得到的子类名称（与扫描顺序一致即可）
string item_a
string item_b
string item_c
# 语音唤醒后的完整指令文本（含目标大类）
string voice_instruction
---
bool success
string error_message
# 赛方要求的播报格式（可拆成两句给 TTS）
string announcement_physical
string announcement_simulation
string announcement_full
# 结构化结果，便于下游记录 / 调试
string pickup_item
string pickup_major
string pickup_workshop
string sim_item
string sim_major
string sim_workshop
string raw_model_reply

")

(setf (get ucar_2026_smart_factory_llm::ReasonPickupOrderResponse :md5sum-) "6f2e4969829f8723b74f2ea5cfb7ffc4")
(setf (get ucar_2026_smart_factory_llm::ReasonPickupOrderResponse :datatype-) "ucar_2026_smart_factory_llm/ReasonPickupOrderResponse")
(setf (get ucar_2026_smart_factory_llm::ReasonPickupOrderResponse :definition-)
      "# 三个二维码解析得到的子类名称（与扫描顺序一致即可）
string item_a
string item_b
string item_c
# 语音唤醒后的完整指令文本（含目标大类）
string voice_instruction
---
bool success
string error_message
# 赛方要求的播报格式（可拆成两句给 TTS）
string announcement_physical
string announcement_simulation
string announcement_full
# 结构化结果，便于下游记录 / 调试
string pickup_item
string pickup_major
string pickup_workshop
string sim_item
string sim_major
string sim_workshop
string raw_model_reply

")



(provide :ucar_2026_smart_factory_llm/ReasonPickupOrder "6f2e4969829f8723b74f2ea5cfb7ffc4")


