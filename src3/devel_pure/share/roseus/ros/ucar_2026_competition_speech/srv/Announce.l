;; Auto-generated. Do not edit!


(when (boundp 'ucar_2026_competition_speech::Announce)
  (if (not (find-package "UCAR_2026_COMPETITION_SPEECH"))
    (make-package "UCAR_2026_COMPETITION_SPEECH"))
  (shadow 'Announce (find-package "UCAR_2026_COMPETITION_SPEECH")))
(unless (find-package "UCAR_2026_COMPETITION_SPEECH::ANNOUNCE")
  (make-package "UCAR_2026_COMPETITION_SPEECH::ANNOUNCE"))
(unless (find-package "UCAR_2026_COMPETITION_SPEECH::ANNOUNCEREQUEST")
  (make-package "UCAR_2026_COMPETITION_SPEECH::ANNOUNCEREQUEST"))
(unless (find-package "UCAR_2026_COMPETITION_SPEECH::ANNOUNCERESPONSE")
  (make-package "UCAR_2026_COMPETITION_SPEECH::ANNOUNCERESPONSE"))

(in-package "ROS")





(defclass ucar_2026_competition_speech::AnnounceRequest
  :super ros::object
  :slots (_event _item _workshop _decision _text _wait ))

(defmethod ucar_2026_competition_speech::AnnounceRequest
  (:init
   (&key
    ((:event __event) "")
    ((:item __item) "")
    ((:workshop __workshop) "")
    ((:decision __decision) "")
    ((:text __text) "")
    ((:wait __wait) nil)
    )
   (send-super :init)
   (setq _event (string __event))
   (setq _item (string __item))
   (setq _workshop (string __workshop))
   (setq _decision (string __decision))
   (setq _text (string __text))
   (setq _wait __wait)
   self)
  (:event
   (&optional __event)
   (if __event (setq _event __event)) _event)
  (:item
   (&optional __item)
   (if __item (setq _item __item)) _item)
  (:workshop
   (&optional __workshop)
   (if __workshop (setq _workshop __workshop)) _workshop)
  (:decision
   (&optional __decision)
   (if __decision (setq _decision __decision)) _decision)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:wait
   (&optional (__wait :null))
   (if (not (eq __wait :null)) (setq _wait __wait)) _wait)
  (:serialization-length
   ()
   (+
    ;; string _event
    4 (length _event)
    ;; string _item
    4 (length _item)
    ;; string _workshop
    4 (length _workshop)
    ;; string _decision
    4 (length _decision)
    ;; string _text
    4 (length _text)
    ;; bool _wait
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _event
       (write-long (length _event) s) (princ _event s)
     ;; string _item
       (write-long (length _item) s) (princ _item s)
     ;; string _workshop
       (write-long (length _workshop) s) (princ _workshop s)
     ;; string _decision
       (write-long (length _decision) s) (princ _decision s)
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;; bool _wait
       (if _wait (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _event
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _event (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _item
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _item (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _workshop
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _workshop (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _decision
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _decision (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _wait
     (setq _wait (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ucar_2026_competition_speech::AnnounceResponse
  :super ros::object
  :slots (_success _speech_text _estimated_duration _message ))

(defmethod ucar_2026_competition_speech::AnnounceResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:speech_text __speech_text) "")
    ((:estimated_duration __estimated_duration) 0.0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _speech_text (string __speech_text))
   (setq _estimated_duration (float __estimated_duration))
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:speech_text
   (&optional __speech_text)
   (if __speech_text (setq _speech_text __speech_text)) _speech_text)
  (:estimated_duration
   (&optional __estimated_duration)
   (if __estimated_duration (setq _estimated_duration __estimated_duration)) _estimated_duration)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _speech_text
    4 (length _speech_text)
    ;; float32 _estimated_duration
    4
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _speech_text
       (write-long (length _speech_text) s) (princ _speech_text s)
     ;; float32 _estimated_duration
       (sys::poke _estimated_duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _speech_text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _speech_text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _estimated_duration
     (setq _estimated_duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ucar_2026_competition_speech::Announce
  :super ros::object
  :slots ())

(setf (get ucar_2026_competition_speech::Announce :md5sum-) "f3261cee1e2a84f216672d4d9f69791a")
(setf (get ucar_2026_competition_speech::Announce :datatype-) "ucar_2026_competition_speech/Announce")
(setf (get ucar_2026_competition_speech::Announce :request) ucar_2026_competition_speech::AnnounceRequest)
(setf (get ucar_2026_competition_speech::Announce :response) ucar_2026_competition_speech::AnnounceResponse)

(defmethod ucar_2026_competition_speech::AnnounceRequest
  (:response () (instance ucar_2026_competition_speech::AnnounceResponse :init)))

(setf (get ucar_2026_competition_speech::AnnounceRequest :md5sum-) "f3261cee1e2a84f216672d4d9f69791a")
(setf (get ucar_2026_competition_speech::AnnounceRequest :datatype-) "ucar_2026_competition_speech/AnnounceRequest")
(setf (get ucar_2026_competition_speech::AnnounceRequest :definition-)
      "# Competition event: task1, task2, task3, task4, task5, or custom.
string event
# Used by task2 and task3.
string item
string workshop
# Used by task4. Accepted aliases include left/right/straight/stop and Chinese names.
string decision
# Required by task1 and custom. Ignored by the fixed task2-task5 templates.
string text
# Wait until the conservative estimated playback duration has elapsed.
bool wait
---
bool success
string speech_text
float32 estimated_duration
string message

")

(setf (get ucar_2026_competition_speech::AnnounceResponse :md5sum-) "f3261cee1e2a84f216672d4d9f69791a")
(setf (get ucar_2026_competition_speech::AnnounceResponse :datatype-) "ucar_2026_competition_speech/AnnounceResponse")
(setf (get ucar_2026_competition_speech::AnnounceResponse :definition-)
      "# Competition event: task1, task2, task3, task4, task5, or custom.
string event
# Used by task2 and task3.
string item
string workshop
# Used by task4. Accepted aliases include left/right/straight/stop and Chinese names.
string decision
# Required by task1 and custom. Ignored by the fixed task2-task5 templates.
string text
# Wait until the conservative estimated playback duration has elapsed.
bool wait
---
bool success
string speech_text
float32 estimated_duration
string message

")



(provide :ucar_2026_competition_speech/Announce "f3261cee1e2a84f216672d4d9f69791a")


