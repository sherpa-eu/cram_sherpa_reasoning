(in-package :cram-sherpa-spatial-relations)

(defun get-elem-depend-agent-pose (elemname &optional (viewpoint "busy_genius"))
  (let*((pose (json-call-pose elemname))
        (pub NIL))
    (if (not (string-equal viewpoint "busy_genius"))
        (setf viewpoint (format NIL "~a/base_link" viewpoint)))
    (setf pub (cl-tf:set-transform *tf* (cl-transforms-stamped:make-transform-stamped "map" elemname (roslisp:ros-time) (cl-transforms:origin pose) (cl-transforms:orientation pose))))
    (cl-transforms-stamped:transform->pose (cl-tf:lookup-transform *tf* viewpoint elemname))))


(defun make-geom-object (objname)
  (let((pose (json-call-pose objname))
       (dim (json-call-dim objname))
       (type (get-elem-type objname)))
  (cram-semantic-map-utils::make-instance
     'cram-semantic-map-utils:semantic-map-geom
     :type type
     :name objname
     :dimensions dim
     :owl-name "owl-ei-daunt-nau"
     :urdf-link-name "urdf-ei-daunt-nau-either"
     :pose pose)))

(defun get-elem-type (name)
 (let*((type NIL))
       (cond ((or (search "tree" name)
                  (search "Tree" name))
              (setf type "tree"))
             ((or (search "road" name)
                  (search "Road" name))
              (setf type "road"))
             ((or (search "rock" name)
                  (search "Rock" name))
                   (setf type "rock"))
             ((or (search "lake" name)
                  (search "Lake" name))
              (setf type "lake"))
             ((or (search "tunnel" name)
                  (search "Tunnel" name))
                   (setf type "tunnel"))
                  ((or (search "cottage" name)
                       (search "Cottage" name))
                   (setf type "cottage"))
                  ((or (search "bridge" name)
                       (search "Bridge" name))
                   (setf type "bridge"))
                  ((search "pylon" name)
                   (setf type "pylon"))
                  ((search "Helipad" name)
                   (setf type "Helipad")))              
   type))

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
