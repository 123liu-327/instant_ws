;; Auto-generated. Do not edit!


(when (boundp 'line_follower::LineTrack)
  (if (not (find-package "LINE_FOLLOWER"))
    (make-package "LINE_FOLLOWER"))
  (shadow 'LineTrack (find-package "LINE_FOLLOWER")))
(unless (find-package "LINE_FOLLOWER::LINETRACK")
  (make-package "LINE_FOLLOWER::LINETRACK"))

(in-package "ROS")
;;//! \htmlinclude LineTrack.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass line_follower::LineTrack
  :super ros::object
  :slots (_header _valid _confidence _image_width _image_height _center_x_px _center_y_px _lookahead_x_px _lookahead_y_px _heading_error_rad ))

(defmethod line_follower::LineTrack
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:valid __valid) nil)
    ((:confidence __confidence) 0.0)
    ((:image_width __image_width) 0)
    ((:image_height __image_height) 0)
    ((:center_x_px __center_x_px) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:center_y_px __center_y_px) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:lookahead_x_px __lookahead_x_px) 0.0)
    ((:lookahead_y_px __lookahead_y_px) 0.0)
    ((:heading_error_rad __heading_error_rad) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _valid __valid)
   (setq _confidence (float __confidence))
   (setq _image_width (round __image_width))
   (setq _image_height (round __image_height))
   (setq _center_x_px __center_x_px)
   (setq _center_y_px __center_y_px)
   (setq _lookahead_x_px (float __lookahead_x_px))
   (setq _lookahead_y_px (float __lookahead_y_px))
   (setq _heading_error_rad (float __heading_error_rad))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:valid
   (&optional (__valid :null))
   (if (not (eq __valid :null)) (setq _valid __valid)) _valid)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:image_width
   (&optional __image_width)
   (if __image_width (setq _image_width __image_width)) _image_width)
  (:image_height
   (&optional __image_height)
   (if __image_height (setq _image_height __image_height)) _image_height)
  (:center_x_px
   (&optional __center_x_px)
   (if __center_x_px (setq _center_x_px __center_x_px)) _center_x_px)
  (:center_y_px
   (&optional __center_y_px)
   (if __center_y_px (setq _center_y_px __center_y_px)) _center_y_px)
  (:lookahead_x_px
   (&optional __lookahead_x_px)
   (if __lookahead_x_px (setq _lookahead_x_px __lookahead_x_px)) _lookahead_x_px)
  (:lookahead_y_px
   (&optional __lookahead_y_px)
   (if __lookahead_y_px (setq _lookahead_y_px __lookahead_y_px)) _lookahead_y_px)
  (:heading_error_rad
   (&optional __heading_error_rad)
   (if __heading_error_rad (setq _heading_error_rad __heading_error_rad)) _heading_error_rad)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _valid
    1
    ;; float32 _confidence
    4
    ;; uint32 _image_width
    4
    ;; uint32 _image_height
    4
    ;; float32[] _center_x_px
    (* 4    (length _center_x_px)) 4
    ;; float32[] _center_y_px
    (* 4    (length _center_y_px)) 4
    ;; float32 _lookahead_x_px
    4
    ;; float32 _lookahead_y_px
    4
    ;; float32 _heading_error_rad
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _image_width
       (write-long _image_width s)
     ;; uint32 _image_height
       (write-long _image_height s)
     ;; float32[] _center_x_px
     (write-long (length _center_x_px) s)
     (dotimes (i (length _center_x_px))
       (sys::poke (elt _center_x_px i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _center_y_px
     (write-long (length _center_y_px) s)
     (dotimes (i (length _center_y_px))
       (sys::poke (elt _center_y_px i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _lookahead_x_px
       (sys::poke _lookahead_x_px (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lookahead_y_px
       (sys::poke _lookahead_y_px (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_error_rad
       (sys::poke _heading_error_rad (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _image_width
     (setq _image_width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _image_height
     (setq _image_height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _center_x_px
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _center_x_px (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _center_x_px i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _center_y_px
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _center_y_px (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _center_y_px i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _lookahead_x_px
     (setq _lookahead_x_px (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lookahead_y_px
     (setq _lookahead_y_px (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_error_rad
     (setq _heading_error_rad (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get line_follower::LineTrack :md5sum-) "a8f2dc59e6af2e2fcb8e42affc92dd20")
(setf (get line_follower::LineTrack :datatype-) "line_follower/LineTrack")
(setf (get line_follower::LineTrack :definition-)
      "std_msgs/Header header
bool valid
float32 confidence
uint32 image_width
uint32 image_height
float32[] center_x_px
float32[] center_y_px
float32 lookahead_x_px
float32 lookahead_y_px
float32 heading_error_rad

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



(provide :line_follower/LineTrack "a8f2dc59e6af2e2fcb8e42affc92dd20")


