(ros::roseus "make-diabolo-mesh")
1;4205;0c
(setq *diabolo-top*
      (make-solid-of-revolution
       (mapcar #'(lambda (x) (scale 1e1 x)) (list #f(0 0 1.5) #f(1 0 1.5) #f(3 0 2.5) #f(5 0 4.5) #f(6 0 7.5) #f(7 0 7.5) #f(6 0 4.5) #f(4 0 2.5) #f(2 0 1.5) #f(1 0 0.1) #f(0 0 0.1)))
       )
      )
(setq *diabolo-bottom*
      (make-solid-of-revolution
       (mapcar #'(lambda (x) (scale 1e1 x))
	       (mapcar #'(lambda (x) (float-vector (elt x 0) (elt x 1) (- (elt x 2)))) (reverse (list #f(0 0 1.5) #f(1 0 1.5) #f(3 0 2.5) #f(5 0 4.5) #f(6 0 7.5) #f(7 0 7.5) #f(6 0 4.5) #f(4 0 2.5) #f(2 0 1.5) #f(1 0 0.1) #f(0 0 0.1)))))
       )
      )
(setq *axis* (make-cylinder 5 20))
(send *axis* :translate #f(0 0 -10))
(send *axis* :assoc *diabolo-top*)
(send *axis* :assoc *diabolo-bottom*)
(setq *link*
      (instance bodyset-link :init (make-cascoords) :bodies (list *axis* *diabolo-bottom* *diabolo-top*)))
(objects (list *link*))


(defclass diabolo-model
  :super cascaded-link
  :slots ())
(defmethod diabolo-model
  (:init
   ()
   (let (b)
     (send-super :init)
     (setq b (instance bodyset-link :init (make-cascoords) :bodies (list *link*)))
     (setq links (list b))
     (send self :assoc b)
     (send self :init-ending)
     self))
  )

(setq *obj* (instance diabolo-model :init))
(send *obj* :name "diabolo")
;;(eus2collada *obj* "package://jsk_diabolo_pr2/meshes")
(eus2collada *obj* "../meshes")

;; to see mesh model
;; meshlab diabolo.dae

;; generate nil_link_mesh.dae
;; rosrun collada_urdf collada_to_urdf diabolo.dae --mesh_output_dir .
