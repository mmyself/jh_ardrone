
(cl:in-package :asdf)

(defsystem "ar_localizer-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GenerateLocations" :depends-on ("_package_GenerateLocations"))
    (:file "_package_GenerateLocations" :depends-on ("_package"))
  ))