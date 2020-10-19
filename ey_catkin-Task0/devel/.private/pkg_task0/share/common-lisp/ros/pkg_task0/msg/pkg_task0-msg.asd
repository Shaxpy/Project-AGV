
(cl:in-package :asdf)

(defsystem "pkg_task0-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "myMessage" :depends-on ("_package_myMessage"))
    (:file "_package_myMessage" :depends-on ("_package"))
  ))