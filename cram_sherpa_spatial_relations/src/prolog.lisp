(in-package :cram-sherpa-spatial-relations)

(defmethod costmap-generator-name->score ((name (eql 'collisions))) 10)

(defclass reasoning-generator () ())
(defmethod costmap-generator-name->score ((name reasoning-generator)) 7)

(defclass gaussian-generator () ())     
(defmethod costmap-generator-name->score ((name gaussian-generator)) 6)
(defclass range-generator () ())
(defmethod costmap-generator-name->score ((name range-generator)) 2)
(defmethod costmap-generator-name->score ((name (eql 'semantic-map-free-space))) 11)

(defvar *sherpa-location-costmap-publisher* nil)
(defvar *sherpa-marker-publisher* nil)
(defparameter *sherpa-last-published-marker-index* nil)

(defun costmap-marker-pub-init ()
  (setf *sherpa-location-costmap-publisher*
        (roslisp:advertise "visualization_marker_array" "visualization_msgs/MarkerArray"))
  (setf *sherpa-marker-publisher*
        (roslisp:advertise "visualization_marker" "visualization_msgs/Marker")))

(roslisp-utilities:register-ros-init-function costmap-marker-pub-init)

(defun remove-markers-up-to-index (index)
  (let ((removers
          (loop for i from 0 to index
                collect (roslisp:make-message "visualization_msgs/Marker"
                                              (std_msgs-msg:frame_id header) cram-tf:*fixed-frame*
                                              (visualization_msgs-msg:ns) ""
                                              (visualization_msgs-msg:id) i
                                              (visualization_msgs-msg:action)
                                              (roslisp-msg-protocol:symbol-code
                                               'visualization_msgs-msg:marker
                                               :delete)))))
    (when removers
      (roslisp:publish *sherpa-location-costmap-publisher*
                       (roslisp:make-message
                        "visualization_msgs/MarkerArray"
                        (visualization_msgs-msg:markers)
                        (map 'vector #'identity removers))))))

(defmethod location-costmap:on-visualize-costmap sherpa ((map location-costmap:location-costmap))
  (sherpa-publish-location-costmap map :threshold 0.0005))

(defun sherpa-publish-location-costmap (map &key (frame-id cram-tf:*fixed-frame*)
                                              (threshold 0.0005))
  (when *sherpa-location-costmap-publisher*
    (multiple-value-bind (markers last-index)
        (location-costmap::location-costmap->marker-array
         map :frame-id frame-id
             :threshold threshold
             :z (slot-value map 'location-costmap:visualization-z)
             :hsv-colormap t
             :elevate-costmap nil)
      (when *sherpa-last-published-marker-index*
        (remove-markers-up-to-index *sherpa-last-published-marker-index*))
      (setf *sherpa-last-published-marker-index* last-index)
      (roslisp:publish *sherpa-location-costmap-publisher* markers))))

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
                                             
;; (defun get-pose-by-call()
;;   (roslisp:wait-for-service "add_costmap_name" 10)
;;   (json-call-pose (slot-value (roslisp:call-service "add_costmap_name" 'hmi_interpreter-srv:text_parser :goal "get") 'hmi_interpreter-srv:result)))

;; (defun get-dim-by-call()
;;   (roslisp:wait-for-service "add_costmap_name" 10)
;;   (json-call-dim (slot-value (roslisp:call-service "add_costmap_name" 'hmi_interpreter-srv:text_parser :goal "get") 'hmi_interpreter-srv:result)))


(defun sherpa-metadata (objname)
  (let((pose (json-call-pose objname))
       (dim (json-call-dim objname)))
     (format t "sherpa-metadata pose ~a dim ~a~%" pose dim)
    (list :width (+ 4 (cl-transforms:x dim))
          :height (+ 4 (cl-transforms:y dim))
          :resolution 0.8
          :origin-x (- (cl-transforms:x
                     (cl-transforms:origin pose)) 2 (/ (cl-transforms:x dim) 2))
          :origin-y (- (cl-transforms:y
                      (cl-transforms:origin pose)) 2 (/ (cl-transforms:y dim) 2))
          :visualization-z (+ 5 (/ (cl-transforms:z dim) 2) (cl-transforms:z (cl-transforms:origin pose))))))
 
(def-prolog-handler sherpa-costmap (bdgs ?objname ?cm)
  (list
   (if (or (not bdgs) (is-var (var-value ?cm bdgs)))
       (add-bdg (var-value ?cm bdgs)
                (apply #'make-instance 'location-costmap
                       (sherpa-metadata (cut:var-value ?objname bdgs)))
                bdgs)
       (when (typep (var-value ?cm bdgs) 'location-costmap)
         bdgs))))



(def-fact-group sherpa-reasoning-costmap (desig-costmap)
  (<- (desig-costmap ?desig ?costmap)
    (format "inside hier~%")
    (or (desig-prop ?desig (:next-to ?objname))
        (desig-prop ?desig (:around ?objname))
        (desig-prop ?desig (:behind ?objname))
        (desig-prop ?desig (:in-front-of ?objname))
        (desig-prop ?desig (:right ?objname))
        (desig-prop ?desig (:right-of ?objname))
        (desig-prop ?desig (:ontop ?objname))
        (desig-prop ?desig (:left ?objname))
        (desig-prop ?desig (:left-of ?objname)))
    (sherpa-costmap ?objname ?costmap)
    (prepositions ?desig ?costmap))

;;TODO: left, right from human perspective, and cut out bounding box  
  (<- (prepositions ?desig ?costmap)
    (desig-prop ?desig (:ontop ?objname))
    (lisp-fun make-geom-object ?objname ?geom)
    (costmap-add-function reasoning-generator
                          (make-semantic-map-costmap ?geom)
                          ?costmap))
  
   (<- (prepositions ?desig ?costmap)
     (or (desig-prop ?desig (:right ?objname))
         (desig-prop ?desig (:right-of ?objname)))
     (desig-prop ?desig (:viewpoint ?viewpoint))
     (lisp-fun json-call-pose ?objname ?objpose)
     (instance-of reasoning-generator ?reasoning-generator-id)
     (costmap-add-function
      ?reasoning-generator-id
      (make-spatial-relations-cost-function ?objpose :Y > 0.0 ?viewpoint)
      ?costmap))
    
   (<- (prepositions ?desig ?costmap)
     (or (desig-prop ?desig (:left ?objname))
         (desig-prop ?desig (:left-of ?objname)))
     (desig-prop ?desig (:viewpoint ?viewpoint))
     (lisp-fun json-call-pose ?objname ?objpose)
     (instance-of reasoning-generator ?reasoning-generator-id)
     (costmap-add-function
      ?reasoning-generator-id
      (make-spatial-relations-cost-function ?objpose :Y < 0.0 ?viewpoint)
      ?costmap))

  (<- (prepositions ?desig ?costmap)
    (desig-prop ?desig (:behind ?objname))
    (desig-prop ?desig (:viewpoint ?viewpoint))
    (lisp-fun json-call-pose ?objname ?objpose)
    (instance-of reasoning-generator ?reasoning-generator-id)
    (costmap-add-function
     ?reasoning-generator-id
     (make-spatial-relations-cost-function ?objpose :X < 0.9 ?viewpoint)
     ?costmap))

    (<- (prepositions ?desig ?costmap)
    (desig-prop ?desig (:in-front-of ?objname))
    (desig-prop ?desig (:viewpoint ?viewpoint))
    (lisp-fun json-call-pose ?objname ?objpose)
    (instance-of reasoning-generator ?reasoning-generator-id)
    (costmap-add-function
     ?reasoning-generator-id
     (make-spatial-relations-cost-function ?objpose :X > 0.9 ?viewpoint)
     ?costmap))
  

  (<- (adapt-map ?costmap ?objname ?objpose)
    (sherpa-costmap ?costmap)
    (costmap-add-function semantic-map-free-space
                         (make-semantic-map-costmap-by-agent
                          ?objname :invert nil :padding 0.1)
                         ?costmap)))
