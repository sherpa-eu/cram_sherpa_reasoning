(in-package :cram-sherpa-spatial-relations)

(defvar *tf* NIL)

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
