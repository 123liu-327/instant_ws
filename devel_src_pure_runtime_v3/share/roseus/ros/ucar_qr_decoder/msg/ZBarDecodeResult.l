;; Auto-generated. Do not edit!


(when (boundp 'ucar_qr_decoder::ZBarDecodeResult)
  (if (not (find-package "UCAR_QR_DECODER"))
    (make-package "UCAR_QR_DECODER"))
  (shadow 'ZBarDecodeResult (find-package "UCAR_QR_DECODER")))
(unless (find-package "UCAR_QR_DECODER::ZBARDECODERESULT")
  (make-package "UCAR_QR_DECODER::ZBARDECODERESULT"))

(in-package "ROS")
;;//! \htmlinclude ZBarDecodeResult.msg.html


(defclass ucar_qr_decoder::ZBarDecodeResult
  :super ros::object
  :slots (_job_id _decoded _decode_ms _stage ))

(defmethod ucar_qr_decoder::ZBarDecodeResult
  (:init
   (&key
    ((:job_id __job_id) 0)
    ((:decoded __decoded) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:decode_ms __decode_ms) 0.0)
    ((:stage __stage) "")
    )
   (send-super :init)
   (setq _job_id (round __job_id))
   (setq _decoded __decoded)
   (setq _decode_ms (float __decode_ms))
   (setq _stage (string __stage))
   self)
  (:job_id
   (&optional __job_id)
   (if __job_id (setq _job_id __job_id)) _job_id)
  (:decoded
   (&optional __decoded)
   (if __decoded (setq _decoded __decoded)) _decoded)
  (:decode_ms
   (&optional __decode_ms)
   (if __decode_ms (setq _decode_ms __decode_ms)) _decode_ms)
  (:stage
   (&optional __stage)
   (if __stage (setq _stage __stage)) _stage)
  (:serialization-length
   ()
   (+
    ;; uint64 _job_id
    8
    ;; string[] _decoded
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _decoded)) 4
    ;; float32 _decode_ms
    4
    ;; string _stage
    4 (length _stage)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint64 _job_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _job_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _job_id) (= (length (_job_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_job_id . bv) 0) 0) s)
              (write-long (ash (elt (_job_id . bv) 1) -1) s))
             ((and (class _job_id) (= (length (_job_id . bv)) 1)) ;; big1
              (write-long (elt (_job_id . bv) 0) s)
              (write-long (if (>= _job_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _job_id s)(write-long (if (>= _job_id 0) 0 #xffffffff) s)))
     ;; string[] _decoded
     (write-long (length _decoded) s)
     (dolist (elem _decoded)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float32 _decode_ms
       (sys::poke _decode_ms (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _stage
       (write-long (length _stage) s) (princ _stage s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint64 _job_id
     
#+(or :alpha :irix6 :x86_64)
      (setf _job_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _job_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string[] _decoded
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _decoded (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _decoded i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float32 _decode_ms
     (setq _decode_ms (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _stage
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _stage (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ucar_qr_decoder::ZBarDecodeResult :md5sum-) "41ac4872f56934370169b3f1087ab96f")
(setf (get ucar_qr_decoder::ZBarDecodeResult :datatype-) "ucar_qr_decoder/ZBarDecodeResult")
(setf (get ucar_qr_decoder::ZBarDecodeResult :definition-)
      "uint64 job_id
string[] decoded
float32 decode_ms
string stage

")



(provide :ucar_qr_decoder/ZBarDecodeResult "41ac4872f56934370169b3f1087ab96f")


