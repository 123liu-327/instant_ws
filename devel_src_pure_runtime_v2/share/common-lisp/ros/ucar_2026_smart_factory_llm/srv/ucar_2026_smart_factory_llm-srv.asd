
(cl:in-package :asdf)

(defsystem "ucar_2026_smart_factory_llm-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ReasonPickupOrder" :depends-on ("_package_ReasonPickupOrder"))
    (:file "_package_ReasonPickupOrder" :depends-on ("_package"))
  ))