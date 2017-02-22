(in-package :cram-sherpa-spatial-relations)

(defmethod costmap-generator-name->score ((name (eql 'collisions))) 10)

(defclass reasoning-generator () ())
(defmethod costmap-generator-name->score ((name reasoning-generator)) 7)

(defclass gaussian-generator () ())     
(defmethod costmap-generator-name->score ((name gaussian-generator)) 6)
(defclass range-generator () ())
(defmethod costmap-generator-name->score ((name range-generator)) 2)
(defmethod costmap-generator-name->score ((name (eql 'semantic-map-free-space))) 11)

(defun json-call-dim (name)
  (if(json-prolog:check-connection)
     (let*((newname (concatenate 'string "http://knowrob.org/kb/unreal_log.owl#" name))
           (liste (cram-utilities:lazy-car
                    (json-prolog:prolog
                     `("object_dimensions" ,newname ?d ?w ?h)))))
 (cl-transforms:make-3d-vector (float (cdr (third liste)))
                                   (float (cdr (first liste)))
                                   (float (cdr (second liste)))))))
     
(defun json-call-pose (name)
(let((pose NIL))
  (if(json-prolog:check-connection)
     (let*((newname (concatenate 'string "http://knowrob.org/kb/unreal_log.owl#" name))
           (liste (first (cram-utilities:lazy-car
                   (json-prolog:prolog
                    `("current_object_pose" ,newname ?pose))))))
       (if (not (null liste))
           (setf pose
                 (cl-transforms:make-pose
                  (cl-transforms:make-3d-vector (float (second  liste))
                                                (float (third liste))
                                                (float (fourth liste)))
                  (cl-transforms:make-quaternion (float (sixth liste))
                                                 (float (seventh liste))
                                                 (float (eighth liste))
                                                 (float (fifth liste))))))))
  pose))
                                             
(defun get-pose-by-call()
  (roslisp:wait-for-service "add_costmap_name" 10)
  (json-call-pose (slot-value (roslisp:call-service "add_costmap_name" 'hmi_interpreter-srv:text_parser :goal "get") 'hmi_interpreter-srv:result)))

(defun get-dim-by-call()
  (roslisp:wait-for-service "add_costmap_name" 10)
  (json-call-dim (slot-value (roslisp:call-service "add_costmap_name" 'hmi_interpreter-srv:text_parser :goal "get") 'hmi_interpreter-srv:result)))


(defun sherpa-metadata ()
  (format t "serpa-metadata~%")
  (let((pose (get-pose-by-call))
       (dim (get-dim-by-call)))
    (list :width (cl-transforms:y dim)
          :height (cl-transforms:z dim)
          :resolution 0.9
          :origin-x (- (cl-transforms:x
                     (cl-transforms:origin pose)) 5)
          :origin-y (- (cl-transforms:y
                      (cl-transforms:origin pose)) 5 ))))
 
(def-prolog-handler sherpa-costmap (bdgs ?cm)
  (list
   (if (or (not bdgs) (is-var (var-value ?cm bdgs)))
       (add-bdg (var-value ?cm bdgs)
                (apply #'make-instance 'location-costmap (sherpa-metadata))
                bdgs)
       (when (typep (var-value ?cm bdgs) 'location-costmap)
         bdgs))))



(def-fact-group sherpa-reasoning-costmap (desig-costmap)
  (<- (desig-costmap ?desig ?costmap)
    (sherpa-costmap ?costmap)
    (format "json-call-dim323~%")
    (prepositions ?desig ?costmap))

  (<- (prepositions ?desig ?costmap)
    (desig-prop ?desig (:next-to ?objname))
    (lisp-fun get-elem-depend-agent-pose ?objname ?objpose)
    (lisp-fun json-call-pose ?objname ?objpose)
    (instance-of gaussian-generator ?gaussian-generator-id)
    (costmap-add-function ?gaussian-generator-id
                          (make-location-cost-function ?objpose  6.5)
                          ?costmap)
    (lisp-fun json-call-dim ?objname ?objdim)
     (costmap-add-height-generator
      (make-constant-height-function ?objdim ?objpose ?resulting-z)
      ?costmap))
  
  (<- (prepositions ?desig ?costmap)
    (desig-prop ?desig (:ontop ?objname))
    (format "json-call-dim1~%")
    (lisp-fun json-call-pose ?objname ?objpose)
    (format "json-call-dim2~%")
    (instance-of gaussian-generator ?gaussian-generator-id)
    (costmap-add-function ?gaussian-generator-id
                          (make-location-cost-function ?objpose  0.5)
                          ?costmap)
    (format "json-call-dim3~%")
    (lisp-fun json-call-dim ?objname ?objdim)
    (format "json-call-dim~%")
    (costmap-add-height-generator
     (make-constant-height-function ?objdim ?objpose ?resulting-z)
     ?costmap)
    (adapt-map ?costmap ?objname))
  
  (<- (prepositions ?desig ?costmap)
    (desig-prop ?desig (:left ?objname))
    (lisp-fun json-call-pose ?objname ?objpose)
    (instance-of gaussian-generator ?gaussian-generator-id)
    (costmap-add-function ?gaussian-generator-id
                          (make-spatial-cost-function ?objpose  6.5)
                          ?costmap)
    (lisp-fun json-call-dim ?objname ?objdim)
     (costmap-add-height-generator
      (make-constant-height-function ?objdim ?objpose ?resulting-z)
      ?costmap))

  (<- (prepositions ?desig ?costmap)
    (desig-prop ?desig (:right ?objname))
    (lisp-fun json-call-pose ?objname ?objpose)
    (instance-of gaussian-generator ?gaussian-generator-id)
    (costmap-add-function ?gaussian-generator-id
                          (make-spatial-location-cost-function ?objpose  6.5)
                          ?costmap)
    (lisp-fun json-call-dim ?objname ?objdim)
     (costmap-add-height-generator
      (make-constant-height-function ?objdim ?objpose ?resulting-z)
      ?costmap))

    (<- (prepositions ?desig ?costmap)
    (desig-prop ?desig (:behind ?objname))
    (lisp-fun json-call-pose ?objname ?objpose)
    (instance-of gaussian-generator ?gaussian-generator-id)
    (costmap-add-function ?gaussian-generator-id
                          (make-spatial-cost-function ?objpose  6.5)
                          ?costmap)
    (lisp-fun json-call-dim ?objname ?objdim)
     (costmap-add-height-generator
      (make-constant-height-function ?objdim ?objpose ?resulting-z)
      ?costmap))

  (<- (adapt-map ?costmap ?objname)
    (format "json-call-dim2~%")
    (costmap-padding ?padding)
    (costmap-add-function semantic-map-free-space
                          (make-semantic-map-costmap-by-agent
                           ?objname :invert :t :padding ?padding)
                          ?costmap)
    (sherpa-costmap ?costmap)
    (instance-of gaussian-generator ?gaussian-generator-id)
    (costmap-add-function ?gaussian-generator-id
                          (make-spatial-cost-function ?objpose  6.5)
                          ?costmap)))

