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
 (cl-transforms:make-3d-vector (float (cdr (third liste))) ;;third
                                   (float (cdr (first liste))) ;;first
                                   (float (cdr (second liste))))))) ;;second

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

(defun sherpa-metadata (objname)
  (let* ((pose (json-call-pose objname))
         (dim (json-call-dim objname))
         (bb (2d-object-bb dim pose))
         (costmap-dim (cl-transforms:v+
                       (cl-transforms:v- (second bb) (first bb))
                       (cl-transforms:make-3d-vector 4 4 0))))
    (list :width (cl-transforms:x costmap-dim)
          :height (cl-transforms:y costmap-dim)
          :resolution 0.8
          :origin-x (- (cl-transforms:x (first bb)) 2)
          :origin-y (- (cl-transforms:y (first bb)) 2)
          :visualization-z (+ 5 (/ (cl-transforms:z dim) 2)
                              (cl-transforms:z (cl-transforms:origin pose))))))

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
    (or (desig-prop ?desig (:next-to ?objname))
        (desig-prop ?desig (:around ?objname))
        (desig-prop ?desig (:behind ?objname))
        (desig-prop ?desig (:front ?objname))
        (desig-prop ?desig (:right ?objname))
        (desig-prop ?desig (:right-of ?objname))
        (desig-prop ?desig (:ontop ?objname))
        (desig-prop ?desig (:left ?objname))
        (desig-prop ?desig (:left-of ?objname)))
    (sherpa-costmap ?objname ?costmap)
    (prepositions ?desig ?costmap))

  (<- (prepositions ?desig ?costmap)
    (desig-prop ?desig (:ontop ?objname))
    (desig-prop ?desig (:viewpoint ?viewpoint))
    (lisp-fun json-call-pose ?objname ?objpose)
    (-> (lisp-pred string-equal "Helipad_6Rv1W" ?objname)
        (prep-gauss ?desig ?costmap ?objpose)
        (prep-loc ?desig ?costmap ?objname)))
  
  (<- (prep-gauss ?desig ?costmap ?objpose)
    (instance-of gaussian-generator ?gaussian-generator-id)
    (costmap-add-function ?gaussian-generator-id
                          (make-location-cost-function ?objpose 0.4)
                          ?costmap))

  (<- (prep-loc ?desig ?costmap ?objname)
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
      (make-spatial-relations-cost-function ?objpose :Y < 0.0 ?viewpoint)
      ?costmap))
    
   (<- (prepositions ?desig ?costmap)
     (or (desig-prop ?desig (:left ?objname))
         (desig-prop ?desig (:left-of ?objname)))
     (desig-prop ?desig (:viewpoint ?viewpoint))
       (lisp-fun json-call-pose ?objname ?objpose)
     (instance-of reasoning-generator ?reasoning-generator-id)
     (costmap-add-function
      ?reasoning-generator-id
      (make-spatial-relations-cost-function ?objpose :Y > 0.0 ?viewpoint)
      ?costmap)
     (instance-of gaussian-generator ?gaussian-generator-id)
     (costmap-add-function ?gaussian-generator-id
                           (make-location-cost-function ?objpose  2.5)
                           ?costmap))
 
  
  (<- (prepositions ?desig ?costmap)
    (desig-prop ?desig (:behind ?objname))
    (desig-prop ?desig (:viewpoint ?viewpoint))
    (lisp-fun json-call-pose ?objname ?objpose)
    (instance-of reasoning-generator ?reasoning-generator-id)
    (costmap-add-function
     ?reasoning-generator-id
     (make-spatial-relations-cost-function ?objpose :X > 0.0 ?viewpoint)
     ?costmap))

  (<- (prepositions ?desig ?costmap)
    (desig-prop ?desig (:front ?objname))
    (desig-prop ?desig (:viewpoint ?viewpoint))
    (lisp-fun json-call-pose ?objname ?objpose)
    (instance-of reasoning-generator ?reasoning-generator-id)
    (costmap-add-function
     ?reasoning-generator-id
     (make-spatial-relations-cost-function ?objpose :X < 0.0 ?viewpoint)
     ?costmap))

   (<- (prepositions ?desig ?costmap)
     (or (desig-prop ?desig (:next-to ?objname))
         (desig-prop ?desig (:around ?objname)))
    (desig-prop ?desig (:viewpoint ?viewpoint))
    (lisp-fun json-call-pose ?objname ?objpose)
    (lisp-fun make-geom-object ?objname ?geom)
     (costmap-add-function
     semantic-map-free-space
     (make-semantic-map-costmap-cut ?geom :invert t)
     ?costmap)
     (instance-of gaussian-generator ?gaussian-generator-id)
     (costmap-add-function ?gaussian-generator-id
                           (make-location-cost-function ?objpose  2.5)
                           ?costmap))
 
  (<- (adapt-map ?costmap ?objname ?objpose)
    (sherpa-costmap ?costmap)
    (costmap-add-function semantic-map-free-space
                         (make-semantic-map-costmap-by-agent
                          ?objname :invert nil :padding 0.1)
                         ?costmap)))
