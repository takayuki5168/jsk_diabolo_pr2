;; idle_diabolo.launch   ;; launch files for idle
;;   roslaunch relay_kinect.launch   ;;30FPS出るpointsをpublish
;;   rosrun jsk_pcl_ros tf_transform_cloud ~input:=/kinect_head_remote/depth_registered/points _target_frame_id:=base_footprint   ;;Kinectの座標を変換して/tf_transform_cloud/outputにpublish
;;   rosrun pr2_juggle calc_idle_diabolo_state   ;;ディアボロの傾きを計算
;; rosrun rviz rviz -d ~/.rviz/idle.rviz

(defun idle-control (right-x-diff no-z) ;; |right-x-diff| < *max-x-dif*(= 220)
  ;; set z
  (setq *just-now-time* (/ (send (ros::time-now) :to-nsec) 1000000))
  (if (eq no-z nil)
      (if (> (- *just-now-time* *now-time*) 1300) ;;700) ;; 1300
	  (progn
	    (setq *now-time* *just-now-time*)
	    (setq *right-z-diff* (- *right-z-diff*))
	    )
	)
    (setq *right-z-diff* 0)
    )
  
  (setq *default-right-diff-cube* (send (send *default-right-cube* :copy-coords) :translate (float-vector right-x-diff 0 *right-z-diff*))) ;; TODO y-diff
  (setq *default-left-diff-cube* (send (send *default-left-cube* :copy-coords) :translate (float-vector (- right-x-diff) 0 (- *right-z-diff*)))) ;; TODO y-diff

  (send *default-right-diff-cube* :rotate 0.2 :x)
  (send *default-left-diff-cube* :rotate -0.2 :x)  
  
  (send *pr2* :rarm :inverse-kinematics *default-right-diff-cube*)
  (send *pr2* :larm :inverse-kinematics *default-left-diff-cube*)
  )

(defun idle (&optional (roll-valid t) (pitch-valid t) (yaw-valid t) &key (teach nil) (diabolo-system nil) (simulate nil))
  (if (eq roll-valid t)
      (progn
	(send *ri* :angle-vector *roll-1-pos* 5000)
	(send *ri* :wait-interpolation)
	(send *ri* :angle-vector *roll-2-pos* 5000)
	(send *ri* :wait-interpolation)
	(send *ri* :angle-vector *roll-3-pos* 3000)
	(send *ri* :wait-interpolation)
        (unix::usleep 3000)
	(send *ri* :angle-vector *roll-4-pos* 1200)
	(send *ri* :wait-interpolation-smooth 800)
	)
    )
  
  (setq *idle-control-x-diff* 0)
  (setq *max-x-diff* 100) ;; TODO  
  (setq *min-x-diff* 0)
  (setq *now-time* (/ (send (ros::time-now) :to-nsec) 1000000))
  (setq *right-z-diff* 150)
  (send *ri* :angle-vector (idle-control 0 simulate) 1500) ;;800)
  (send *pr2* :head-neck-p :joint-angle 50)  

  (if (eq simulate t)
      (setq *tfb* (instance ros::transform-broadcaster :init))
    )
  (do-until-key
   ;; subscribe
   (ros::spin-once "idle")
   (if (eq diabolo-system t)
       (if (eq diabolo-system t)       
	   (ros::spin-once "diabolo-system")
	 )
     )
   (if (eq simulate t)
       (ros::spin-once "diabolo-system-simulate")
     )

   (setq idle-msg (instance std_msgs::float64 :init))   
   (send idle-msg :data 1)
   (ros::publish "idle" idle-msg)
   
   (progn ;; pitch
     ;; P制御
     (if (eq teach t) ;; when teach
	 (progn
	   (setq *joy-arm-input* (elt *joy* 3))
	   (setq *idle-control-x-diff* (* *joy-arm-input* 100))
	   )
       (progn
	 (if (eq diabolo-system t) ;; when diabolo-system
	     (progn
	       (setq *idle-control-x-diff* (- (* *diabolo-system-input-arm* 1000) 700))
	       (print (list "[DiaboloSystem] input arm" *diabolo-system-input-arm*))
	       (print *idle-control-x-diff*)
	       )
	   (progn
	     (setq *idle-control-x-diff* (* *idle-diabolo-pitch* 10)) ;; when controller TODO params
	     )
	   )
	 )
       )

     ;; 最大最小制限
     (if (> *idle-control-x-diff* *max-x-diff*)
	 (setq *idle-control-x-diff* *max-x-diff*))
     (if (< *idle-control-x-diff* (- *max-x-diff*))
	 (setq *idle-control-x-diff* (- *max-x-diff*)))

     ;; 不感帯
     (if (< *idle-control-x-diff* *min-x-diff*)
	 (if (> *idle-control-x-diff* (- *min-x-diff*))
	     (setq *idle-control-x-diff* 0)))

     ;; 実機での動作
     (if (eq pitch-valid nil)
	 (setq *idle-control-x-diff* 0)
       )
     (setq *idle-ec* (idle-control *idle-control-x-diff* simulate))     
     (bench (send *ri* :angle-vector *idle-ec* 1500)) ;;800)
     )

   (progn ;; yaw
     (if (eq teach t) ;; when teach
	 (progn
	   (setq *joy-base-input* (elt *joy* 0))
	   (setq *idle-diabolo-yaw* (* *joy-base-input* 30))
	   )
       (progn
	 (if (eq diabolo-system t) ;; when diabolo system
	     (progn
	       (setq *idle-diabolo-yaw* (* (/ (* *diabolo-system-input-base* 180) 3.14) 1)) ;;TODOTODO   magic number 100
	       )
	   )
	 )
       )    
     ;; 外れ値除去
     (if (> *idle-diabolo-yaw* 60)
	 (setq *idle-diabolo-yaw* 0))
     (if (< *idle-diabolo-yaw* -60)
	 (setq *idle-diabolo-yaw* 0))
     
     ;; 最大角度制限
     (if (> *idle-diabolo-yaw* 20) ;;20)
	 (setq *idle-diabolo-yaw* 20)) ;;20))
     (if (< *idle-diabolo-yaw* -20) ;;-20)
	 (setq *idle-diabolo-yaw* -20)) ;;-20))
     
     (setq *radius* 0.7) ;;(/ (elt (send (send *pr2* :rarm :end-coords) :worldpos) 0) 1000)) ;; [m] ;; TODO
     (setq *radius* (* *radius* 0.8)) ;; TODO
     (setq *idle-diabolo-yaw-rad* (/ (* *idle-diabolo-yaw* 3.14) 180))
     
     (setq *diff-x* (* *radius* (- 1 (cos *idle-diabolo-yaw-rad*))))
     (setq *diff-y* (* *radius* (sin (- *idle-diabolo-yaw-rad*))))

     ;; 実機での動作
     (if (eq yaw-valid t)
	 (if (eq simulate t)
	     (progn
	       )
	   (progn
	     (setq *diff-theta* *idle-diabolo-yaw-rad*)
	     (setq *goal* (send *ri* :move-trajectory *diff-x* *diff-y* *diff-theta* 1000)) ;;10000))
	     (send *ac* :send-goal *goal*)
	     )
	   )
       )
     )

   ;; when simulation
   (if (eq simulate t)
       (progn
	 ;; make tf-broadcaster from /map to /base_footprint
	 (setq *diff-theta* *idle-diabolo-yaw-rad*)	 
	 (setq *ret* (make-coords :pos (float-vector (* *diff-x* 1000) (* *diff-y* 1000) 0) :rpy (float-vector *diff-theta* 0 0)))
	 (setq *frame-id* "/map")
	 (setq *type* "/base_footprint")
	 (send *tfb* :send-transform *ret* *frame-id* *type*)
	 
	 ;; publish joint state
	 (send *ri* :publish-joint-state)
	 (ros::sleep)
	 )
     )
   )
  
  (send *pr2* :rarm :inverse-kinematics *default-right-cube*)  
  (send *pr2* :larm :inverse-kinematics *default-left-cube*)   
  (setq *default-pos* (send *pr2* :angle-vector))
  (send *ri* :angle-vector *default-pos* 5000)

  (if (eq yaw-valid t)
      (if (eq simulate t)
	  (progn
	    )
	(progn
	  (setq *goal* (send *ri* :move-trajectory 0 0 0 10000))
	  (send *ac* :send-goal *goal*)
	  )
	)
    )
  )

;;(defun publish-left-angle-vector (time)   ;; after solving ik
;;  (setq *idle-angle-vector-joint-names* (mapcar #'(lambda (val) (send (send val :joint) :name)) (send *pr2* :larm)))  
;;  (setq *idle-angle-vector-positions* (map float-vector #'deg2rad (send *pr2* :larm :angle-vector)))
;;  (setq *idle-angle-vector-time* time)
;;
;;  ;; init msg
;;  (setq *idle-angle-vector-msg* (instance control_msgs::FollowJointTrajectoryActionGoal :init))
;;
;;  ;; assign data to msg
;;  (setq *ros-time-sec* (send (send (ros::time) :now) :sec))
;;  (setq *ros-time-nsec* (send (send (ros::time) :now) :nsec))  
;;  (let ((i (instance trajectory_msgs::JointTrajectoryPoint :init)))
;;    (send i :positions  *idle-angle-vector-positions*)
;;    (send i :velocities  #f(0.0 0.0 0.0 0.0 0.0 0.0 0.0))
;;    (send (send i :time_from_start) :sec (/ *idle-angle-vector-time* 1000))
;;    (send (send i :time_from_start) :nsec (- *idle-angle-vector-time* (* (/ *idle-angle-vector-time* 1000) 1000)))
;;    (send (send (send (send *idle-angle-vector-msg* :goal :trajectory) :header) :stamp) :sec *ros-time-sec*)
;;    (send (send (send (send *idle-angle-vector-msg* :goal :trajectory) :header) :stamp) :nsec *ros-time-nsec*)
;;    (send *idle-angle-vector-msg* :goal :trajectory :points (list i)) ;; TODO
;;    )
;;
;;  (send *idle-angle-vector-msg* :header :seq 1)
;;  (send (send (send *idle-angle-vector-msg* :header) :stamp)
;;	:sec *ros-time-sec*)
;;  (send (send (send *idle-angle-vector-msg* :header) :stamp)
;;	:nsec *ros-time-nsec*)
;;
;;  (send *idle-angle-vector-msg*
;;	:goal :trajectory :joint_names *idle-angle-vector-joint-names*)
;;
;;  (ros::publish "/l_arm_controller/follow_joint_trajectory/goal" *idle-angle-vector-msg*)
;;  )
;;
;;(defun publish-right-angle-vector (time)   ;; after solving ik
;;  (setq *idle-angle-vector-joint-names* (mapcar #'(lambda (val) (send (send val :joint) :name)) (send *pr2* :rarm)))  
;;  (setq *idle-angle-vector-positions* (map float-vector #'deg2rad (send *pr2* :rarm :angle-vector)))
;;  (setq *idle-angle-vector-time* time)
;;
;;  ;; init msg
;;  (setq *idle-angle-vector-msg* (instance control_msgs::FollowJointTrajectoryActionGoal :init))
;;
;;  ;; assign data to msg
;;  (setq *ros-time-sec* (send (send (ros::time) :now) :sec))
;;  (setq *ros-time-nsec* (send (send (ros::time) :now) :nsec))  
;;  (let ((i (instance trajectory_msgs::JointTrajectoryPoint :init)))
;;    (send i :positions  *idle-angle-vector-positions*)
;;    (send i :velocities  #f(0.0 0.0 0.0 0.0 0.0 0.0 0.0))
;;    (send (send i :time_from_start) :sec (/ *idle-angle-vector-time* 1000))
;;    (send (send i :time_from_start) :nsec (- *idle-angle-vector-time* (* (/ *idle-angle-vector-time* 1000) 1000)))
;;    (send (send (send (send *idle-angle-vector-msg* :goal :trajectory) :header) :stamp) :sec *ros-time-sec*)
;;    (send (send (send (send *idle-angle-vector-msg* :goal :trajectory) :header) :stamp) :nsec *ros-time-nsec*)
;;    (send *idle-angle-vector-msg* :goal :trajectory :points (list i)) ;; TODO
;;    )
;;
;;  (send *idle-angle-vector-msg* :header :seq 1)
;;  (send (send (send *idle-angle-vector-msg* :header) :stamp)
;;	:sec *ros-time-sec*)
;;  (send (send (send *idle-angle-vector-msg* :header) :stamp)
;;	:nsec *ros-time-nsec*)
;;
;;  (send *idle-angle-vector-msg*
;;	:goal :trajectory :joint_names *idle-angle-vector-joint-names*)
;;
;;  (ros::publish "/r_arm_controller/follow_joint_trajectory/goal" *idle-angle-vector-msg*)
;;  )
