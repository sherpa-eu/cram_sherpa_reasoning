(defsystem cram-sherpa-spatial-relations
  :author "none"
  :license "BSD"
  :depends-on (cram-designators
               cram-location-costmap
               cram-prolog
               roslisp
               cram-semantic-map-costmap
               ;;cram-bullet-reasoning
               ;;cram-bullet-reasoning-belief-state
               ;;cram-plan-library:
               ;;  cram-bullet-reasoning-designators
               ;; cram-beliefstate
               cl-tf
               ;;cram-semantic-map-designators
               ;;gazebo_msgs-srv
               hmi_interpreter-srv
               alexandria)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "costmap-knowledge" :depends-on("package"))
     (:file "cost-functions" :depends-on ("package"))
     (:file "prolog" :depends-on ("package" "cost-functions"))))))
