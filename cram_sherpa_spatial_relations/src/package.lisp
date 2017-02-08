(in-package :cl-user)

(desig-props:def-desig-package cram-sherpa-spatial-relations
    (:use #:desig
          #:location-costmap
          #:common-lisp
          #:cram-prolog
          #:btr
          #:cram-utilities
	  #:cram-bullet-reasoning
          #:cram-bullet-reasoning-belief-state
          #:cram-language-implementation)
  (:shadowing-import-from #:btr object pose object-pose width height name)
  (:shadowing-import-from #:cpl-impl #:fail))
