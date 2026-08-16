
(cl:in-package :asdf)

(defsystem "ucar_qr_decoder-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "ZBarDecodeRequest" :depends-on ("_package_ZBarDecodeRequest"))
    (:file "_package_ZBarDecodeRequest" :depends-on ("_package"))
    (:file "ZBarDecodeResult" :depends-on ("_package_ZBarDecodeResult"))
    (:file "_package_ZBarDecodeResult" :depends-on ("_package"))
  ))