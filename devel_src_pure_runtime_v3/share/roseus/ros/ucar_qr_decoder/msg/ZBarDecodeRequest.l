;; Auto-generated. Do not edit!


(when (boundp 'ucar_qr_decoder::ZBarDecodeRequest)
  (if (not (find-package "UCAR_QR_DECODER"))
    (make-package "UCAR_QR_DECODER"))
  (shadow 'ZBarDecodeRequest (find-package "UCAR_QR_DECODER")))
(unless (find-package "UCAR_QR_DECODER::ZBARDECODEREQUEST")
  (make-package "UCAR_QR_DECODER::ZBARDECODEREQUEST"))

(in-package "ROS")
;;//! \htmlinclude ZBarDecodeRequest.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(intern "*MODE_RAW_ONLY*" (find-package "UCAR_QR_DECODER::ZBARDECODEREQUEST"))
(shadow '*MODE_RAW_ONLY* (find-package "UCAR_QR_DECODER::ZBARDECODEREQUEST"))
(defconstant ucar_qr_decoder::ZBarDecodeRequest::*MODE_RAW_ONLY* 0)
(intern "*MODE_ENHANCED_ONLY*" (find-package "UCAR_QR_DECODER::ZBARDECODEREQUEST"))
(shadow '*MODE_ENHANCED_ONLY* (find-package "UCAR_QR_DECODER::ZBARDECODEREQUEST"))
(defconstant ucar_qr_decoder::ZBarDecodeRequest::*MODE_ENHANCED_ONLY* 1)

(defun ucar_qr_decoder::ZBarDecodeRequest-to-symbol (const)
  (cond
        ((= const 0) 'ucar_qr_decoder::ZBarDecodeRequest::*MODE_RAW_ONLY*)
        ((= const 1) 'ucar_qr_decoder::ZBarDecodeRequest::*MODE_ENHANCED_ONLY*)
        (t nil)))

(defclass ucar_qr_decoder::ZBarDecodeRequest
  :super ros::object
  :slots (_job_id _mode _image ))

(defmethod ucar_qr_decoder::ZBarDecodeRequest
  (:init
   (&key
    ((:job_id __job_id) 0)
    ((:mode __mode) 0)
    ((:image __image) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _job_id (round __job_id))
   (setq _mode (round __mode))
   (setq _image __image)
   self)
  (:job_id
   (&optional __job_id)
   (if __job_id (setq _job_id __job_id)) _job_id)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:serialization-length
   ()
   (+
    ;; uint64 _job_id
    8
    ;; uint8 _mode
    1
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
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
     ;; uint8 _mode
       (write-byte _mode s)
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
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
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;;
   self)
  )

(setf (get ucar_qr_decoder::ZBarDecodeRequest :md5sum-) "42e4dcb65da54382e5d3347c34722c08")
(setf (get ucar_qr_decoder::ZBarDecodeRequest :datatype-) "ucar_qr_decoder/ZBarDecodeRequest")
(setf (get ucar_qr_decoder::ZBarDecodeRequest :definition-)
      "uint8 MODE_RAW_ONLY=0
uint8 MODE_ENHANCED_ONLY=1

uint64 job_id
uint8 mode
sensor_msgs/Image image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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

")



(provide :ucar_qr_decoder/ZBarDecodeRequest "42e4dcb65da54382e5d3347c34722c08")


