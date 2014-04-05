
(cl:in-package :asdf)

(defsystem "mlnpredicates-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :mlnpredicates-msg
)
  :components ((:file "_package")
    (:file "MLNPredicates" :depends-on ("_package_MLNPredicates"))
    (:file "_package_MLNPredicates" :depends-on ("_package"))
  ))