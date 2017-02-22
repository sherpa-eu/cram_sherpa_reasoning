(in-package :cram-sherpa-spatial-relations)

(defvar *tf* NIL)
(defvar *pub-init* NIL)

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

(defun make-spatial-cost-function (location axis pred threshold)
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

(defun get-elem-depend-agent-pose (elemname &optional (viewpoint "busy_genius"))
  (let*((pose (json-call-pose elemname))
        (pub NIL))
    (format t "teeest ~a~%" pose)
    (if (not (string-equal viewpoint "busy_genius"))
        (setf viewpoint (format NIL "~a/base_link" viewpoint)))
    (format t "komisch ~a~%" (cl-transforms-stamped:make-transform-stamped "map" elemname (roslisp:ros-time) (cl-transforms:origin pose) (cl-transforms:orientation pose)))
      (format t "teeest123 ~a~%" *tf*)
    (setf pub (cl-tf:set-transform *tf* (cl-transforms-stamped:make-transform-stamped "map" elemname (roslisp:ros-time) (cl-transforms:origin pose) (cl-transforms:orientation pose))))
      (format t "teeest123 ~a~%" *tf*)
    (cl-transforms-stamped:transform->pose (cl-tf:lookup-transform *tf* viewpoint elemname))))


(defun make-geom-object (objname)
  (let((pose (json-call-pose objname))
       (type (get-elem-type objname)))
    (cram-semantic-map-utils::make-instance
     'cram-semantic-map-utils:semantic-map-geom
     :type type
     :name objname
     :owl-name "owl-ei-daunt-nau"
     :urdf-link-name "urdf-ei-daunt-nau-either"
     :pose pose)))
    
(defun make-semantic-map-costmap-by-agent (objects &key invert (padding 0.0))
  "Generates a semantic-map costmap for all `objects'. `objects' is a
list of SEM-MAP-UTILS:SEMANTIC-MAP-GEOMs"
  (let* ((objects (list (make-geom-object objects)))
         (costmap-generators (mapcar (lambda (object)
                                      (make-semantic-map-object-costmap-by-agent-generator
                                       object :padding padding))
                                    (cut:force-ll objects))))
    (flet ((invert-matrix (matrix)
             (declare (type cma:double-matrix matrix))
             (dotimes (row (cma:height matrix) matrix)
               (dotimes (column (cma:width matrix))
                 (if (eql (aref matrix row column) 0.0d0)
                     (setf (aref matrix row column) 1.0d0)
                     (setf (aref matrix row column) 0.0d0)))))
           (generator (costmap-metadata matrix)
             (declare (type cma:double-matrix matrix))
             (dolist (generator costmap-generators matrix)
               (setf matrix (funcall generator costmap-metadata matrix)))))
      (make-instance 'map-costmap-generator
        :generator-function (if invert
                                (alexandria:compose #'invert-matrix #'generator)
                                #'generator)))))

(defun make-semantic-map-object-costmap-by-agent-generator (object &key (padding 0.0))
  (declare (type sem-map-utils:semantic-map-geom object))
           (let* ((transform (cl-transforms:pose->transform  (cl-transforms:make-pose (cl-transforms:origin (get-elem-depend-agent-pose (sem-map-utils:name object)) (cl-transforms:make-identity-rotation)))))
                  (dimensions (cl-transforms:v+
                               (json-call-dim  (sem-map-utils:name object))
                               (cl-transforms:make-3d-vector padding padding padding)))
                  (pt->obj-transform (cl-transforms:transform-inv transform))
                  (z-value (cl-transforms:z (cl-transforms:translation transform))))
    (destructuring-bind ((obj-min obj-max)
                         (local-min local-max))
        (list (semantic-map-costmap::2d-object-bb dimensions transform)
              (semantic-map-costmap::2d-object-bb dimensions))
      (flet ((generator-function (semantic-map-costmap::costmap-metadata result)
               (with-slots (origin-x origin-y resolution) costmap-metadata
                 ;; For performance reasons, we first check if the point is
                 ;; inside the object's bounding box in map and then check if it
                 ;; really is inside the object.
                 (let ((min-index-x (map-coordinate->array-index
                                     (cl-transforms:x obj-min)
                                     resolution origin-x))
                       (max-index-x (map-coordinate->array-index
                                     (cl-transforms:x obj-max)
                                     resolution origin-x))
                       (min-index-y (map-coordinate->array-index
                                     (cl-transforms:y obj-min)
                                     resolution origin-y))
                       (max-index-y (map-coordinate->array-index
                                     (cl-transforms:y obj-max)
                                     resolution origin-y)))
                   (loop for y-index from min-index-y to max-index-y
                         for y from (- (cl-transforms:y obj-min) resolution)
                           by resolution do
                             (loop for x-index from min-index-x to max-index-x
                                   for x from (- (cl-transforms:x obj-min) resolution)
                                     by resolution do
                                       (when (semantic-map-costmap::inside-aabb
                                              local-min local-max
                                              (cl-transforms:transform-point
                                               pt->obj-transform
                                               (cl-transforms:make-3d-vector
                                                x y z-value)))
                                         (setf (aref result y-index x-index) 1.0d0))))))
               result))
        #'generator-function))))


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
  
(roslisp-utilities:register-ros-init-function init-tf)

