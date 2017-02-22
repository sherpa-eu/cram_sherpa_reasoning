(defsystem cram-sherpa-spatial-relations
  :author "none"
  :license "BSD"
  :depends-on (cram-designators
               cram-location-costmap
               cram-prolog
               roslisp
               cram-semantic-map-costmap
               cram-robot-pose-gaussian-costmap
               cl-tf
               cl-transforms
               cram-json-prolog
               cram-utilities
               ;; hmi_interpreter-srv
               alexandria)
  :components
  ((:module "src"
    :components
    ((:file "package")
     ;; (:file "costmap-knowledge" :depends-on("package"))
     (:file "cost-functions" :depends-on ("package"))
     (:file "prolog" :depends-on ("package" "cost-functions"))))))
