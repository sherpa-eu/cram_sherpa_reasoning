(in-package :cram-sherpa-spatial-relations)

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
