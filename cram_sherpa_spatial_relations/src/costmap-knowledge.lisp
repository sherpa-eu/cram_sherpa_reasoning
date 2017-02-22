(in-package :cram-sherpa-spatial-relations)

(def-fact-group costmap-metadata ()
   (<- (costmap-size 10 10))
  (<- (costmap-origin 0 0))
  (<- (costmap-resolution 0.9))
  (<- (costmap-padding 0.1)))

;;(desig::disable-location-validation-function 'btr-desig::validate-designator-solution)
;;(desig::disable-location-validation-function 'btr-desig::check-ik-solution)
