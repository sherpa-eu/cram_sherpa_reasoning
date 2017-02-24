(in-package :cram-sherpa-spatial-relations)

(defvar *tf* NIL)
(defvar *pub* NIL)

(defun init-tf ()
  (setf *tf* (make-instance 'cl-tf:transform-listener))
  (setf *pub* (cl-tf:make-transform-broadcaster))
  )

(defun make-location-function (loc std-dev)
  (let ((loc (cl-transforms:origin loc)))
    (make-gauss-cost-function loc `((,(float (* std-dev std-dev) 0.0d0) 0.0d0)
                                    (0.0d0 ,(float (* std-dev std-dev)))))))

(defun make-constant-height-function (dim objpose height)
    (setf height  (+ (+ (cl-transforms:z (cl-transforms:origin objpose))
                        (cl-transforms:z dim)) 1))
    (lambda (x y)
      (declare (ignore x y))
      (list height)))

(defun make-semantic-map-costmap (object)
  "Generates a semantic-map costmap for all `objects'. `objects' is a
list of SEM-MAP-UTILS:SEMANTIC-MAP-GEOMs"
  (make-instance 'map-costmap-generator
    :generator-function (semantic-map-costmap::make-semantic-map-object-costmap-generator object)))

(defun get-elem-depend-agent-pose (elemname &optional (viewpoint "busy_genius"))
  (let*((pose (json-call-pose elemname))
        (pub NIL))
    (if (not (string-equal viewpoint "busy_genius"))
        (setf viewpoint (format NIL "~a/base_link" viewpoint)))
    (setf pub (cl-tf:set-transform *tf* (cl-transforms-stamped:make-transform-stamped "map" elemname (roslisp:ros-time) (cl-transforms:origin pose) (cl-transforms:orientation pose))))
    (cl-transforms-stamped:transform->pose (cl-tf:lookup-transform *tf* viewpoint elemname))))

(defun make-geom-object (objname)
  (setf cram-tf:*fixed-frame* "map")
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

(defun make-spatial-relations-cost-function (location axis pred threshold viewpoint)
  (format t "location is ~a~%" location)
   (if (not (string-equal "busy_genius" viewpoint))
       (setf viewpoint (format nil "~a/base_link" viewpoint)))
  (setf cram-tf:*fixed-frame* viewpoint)
  (roslisp:ros-info (sherpa-spatial-relations) "calculate the costmap")
  (let* ((new-loc (cl-transforms:make-pose
                   (cl-transforms:origin location)
                   (cl-transforms:make-identity-rotation)))
         (transformation (cl-transforms:pose->transform new-loc)) 
         (world->location-transformation (cl-transforms:transform-inv transformation)))
    (lambda (x y)
      (let* ((point (cl-transforms:transform-point world->location-transformation
                                                   (cl-transforms:make-3d-vector x y 0)))
             (coord (ecase axis
                      (:x (cl-transforms:x point))
                      (:y (cl-transforms:y point))))
             (mode (sqrt (+   (* (cl-transforms:x point) (cl-transforms:x point))
                             (* (cl-transforms:y point) (cl-transforms:y point))))))
        (if (funcall pred coord 0.0d0)
            (if (> (abs (/ coord mode)) threshold)
                (abs (/ coord mode))
                0.0d0)
            0.0d0)))))

(roslisp-utilities:register-ros-init-function init-tf)

