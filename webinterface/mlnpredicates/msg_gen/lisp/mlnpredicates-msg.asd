
(cl:in-package :asdf)

(defsystem "mlnpredicates-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MLNPredicate" :depends-on ("_package_MLNPredicate"))
    (:file "_package_MLNPredicate" :depends-on ("_package"))
  ))