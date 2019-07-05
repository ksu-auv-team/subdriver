
(cl:in-package :asdf)

(defsystem "message_test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Info" :depends-on ("_package_Info"))
    (:file "_package_Info" :depends-on ("_package"))
  ))