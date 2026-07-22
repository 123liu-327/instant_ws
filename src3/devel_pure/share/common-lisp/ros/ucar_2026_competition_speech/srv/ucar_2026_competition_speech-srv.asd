
(cl:in-package :asdf)

(defsystem "ucar_2026_competition_speech-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Announce" :depends-on ("_package_Announce"))
    (:file "_package_Announce" :depends-on ("_package"))
  ))