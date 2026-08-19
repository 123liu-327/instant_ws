
(cl:in-package :asdf)

(defsystem "line_follower-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LineTrack" :depends-on ("_package_LineTrack"))
    (:file "_package_LineTrack" :depends-on ("_package"))
  ))