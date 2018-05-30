;; ;;idle用launchファイル群
;; idle_diabolo.launch
;;   ;;30FPS出るpointsをpublish
;;   roslaunch relay_kinect.launch ;;25Hz
;;
;;   ;;Kinectの座標を変換して/tf_transform_cloud/outputにpublish
;;   rosrun jsk_pcl_ros tf_transform_cloud ~input:=/kinect_head_remote/depth_registered/points _target_frame_id:=base_footprint ;;25Hz
;;
;;   ;;ディアボロの傾きを計算
;;   rosrun pr2_juggle calc_idle_diabolo_state ;;25Hz

;; ;;toss用launchファイル群
;; toss_diabolo.launch
;;   ;;30FPS出るpointsをpublish
;;   roslaunch relay_kinect.launch ;;25Hz
;;
;;   ;;Kinectの座標を変換して/tf_transform_cloud/outputにpublish
;;   rosrun jsk_pcl_ros tf_transform_cloud ~input:=/kinect_head_remote/depth_registered/points _target_frame_id:=base_footprint ;;25Hz
;;
;;   ;;ディアボロの座標を計算
;;   rosrun pr2_juggle calc_toss_diabolo_pos ;;25Hz


;; ;;システム同定のためのロガー
;; python state-input-logger.py


;; sudo service chrony restart

;; ;;rosbag を取る
;; rosbag record -O diabolo0.bag  /diabolo_marker/diabolo_marker /diabolo_marker/diabolo_cube_marker /calc_idle_diabolo_state/points /calc_idle_diabolo_state/pitch_points /tf

(defun init ()
  (setq *idle-diabolo-pitch* 0)
  (setq *idle-diabolo-yaw* 0)
  
  (setq *toss-diabolo-x* 0)
  (setq *toss-diabolo-y* 0)
  (setq *toss-diabolo-z* 0)
  (setq *toss-diabolo-x-predict* 0)  
  
  (setq *particle-system-input-arm* 0)
  (setq *particle-system-input-base* 0)
  
  (ros::load-ros-manifest "roseus")

  ;;
  ;; Subscriber
  ;;
  (ros::roseus "listener")

  ;; subscribe idle diabolo state
  (ros::create-nodehandle "idle")  
  (ros::subscribe "calc_idle_diabolo_state/diabolo_state" std_msgs::Float64MultiArray
  		  #'(lambda (msg) (progn
  				    (setq *idle-diabolo-pitch* (elt (send msg :data) 0))
  				    (setq *idle-diabolo-yaw* (elt (send msg :data) 1))				    
  				    ))
		  :groupname "idle")
  
  ;; subscribe toss diabolo pos
  (ros::create-nodehandle "toss")
  (ros::subscribe "calc_toss_diabolo_pos/pos_float" std_msgs::Float64MultiArray
		  #'(lambda (msg) (progn
				    (setq *toss-diabolo-x* (elt (send msg :data) 0))
				    (setq *toss-diabolo-y* (elt (send msg :data) 1))
				    (setq *toss-diabolo-z* (elt (send msg :data) 2))				    				    
				    ))
		  :groupname "toss")
  (ros::subscribe "calc_toss_diabolo_pos/pos_x" std_msgs::Float64
		  #'(lambda (msg) (progn
				    (setq *toss-diabolo-x-predict* (send msg :data))
				    ))
		  :groupname "toss")
  
  ;; subscribe joy
  ;;(ros::subscribe "joy" sensor_msgs::Joy
  ;;		  #'(lambda (msg) (setq *joy* (send msg :axes))))

  ;; subscribe ParticleSystem
  ;;(ros::subscribe "particle_system/arm" std_msgs::float64
  ;;		  #'(lambda (msg) (setq *particle-system-input-arm* (send msg :data))))
  ;;(ros::subscribe "particle_system/base" std_msgs::float64
  ;;		  #'(lambda (msg) (setq *particle-system-input-base* (send msg :data))))

  ;; publish whether idle or not
  (ros::advertise "idle" std_msgs::float64 1)

  
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init)
  
  (send *ri* :angle-vector (send *pr2* :reset-pose) 5000)
  (setq *ac* (pr2-interface-move-base-trajectory-action *ri*))
  
  (send *ri* :start-grasp)
  (send *pr2* :head-neck-p :joint-angle 50)

  (solve-ik)
  
  (objects (list *pr2*))
  )

(defun solve-ik ()
  (send *pr2* :reset-pose)
  (send *pr2* :start-grasp)  
  (send *pr2* :head-neck-p :joint-angle 50)
  
  (progn ;; default 500~900
    (setq *default-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -120 1050)))
    (send *pr2* :rarm :inverse-kinematics *default-right-cube*)  
    (setq *default-left-cube* (make-cube 100 100 100 :pos (float-vector 700 120 1050)))
    (send *pr2* :larm :inverse-kinematics *default-left-cube*)   
    (setq *default-pos* (send *pr2* :angle-vector))
    )

  (progn  ;; idle
    (setq *idle-right-up-cube* (make-cube 100 100 100 :pos (float-vector 700 -150 1150)))
    (send *pr2* :rarm :inverse-kinematics *idle-right-up-cube*)  
    (setq *idle-left-down-cube* (make-cube 100 100 100 :pos (float-vector 700 150 950)))
    (send *pr2* :larm :inverse-kinematics *idle-left-down-cube*)    
    (setq *idle-1-pos* (send *pr2* :angle-vector))
    
    (setq *idle-right-down-cube* (make-cube 100 100 100 :pos (float-vector 700 -150 950)))
    (send *pr2* :rarm :inverse-kinematics *idle-right-down-cube*)
    (setq *idle-left-up-cube* (make-cube 100 100 100 :pos (float-vector 700 150 1150)))
    (send *pr2* :larm :inverse-kinematics *idle-left-up-cube*)   
    (setq *idle-2-pos* (send *pr2* :angle-vector))
    )
  
  (progn ;; roll
    (setq *roll-1-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -50 1050)))
    (send *pr2* :rarm :inverse-kinematics *roll-1-right-cube*)
    (setq *roll-1-left-cube* (make-cube 100 100 100 :pos (float-vector 700 250 1050)))
    (send *pr2* :larm :inverse-kinematics *roll-1-left-cube*)   
    (setq *roll-1-pos* (send *pr2* :angle-vector))

    (setq *roll-2-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -50 900)))
    (send *pr2* :rarm :inverse-kinematics *roll-2-right-cube*)  
    (setq *roll-2-left-cube* (make-cube 100 100 100 :pos (float-vector 700 250 900)))
    (send *pr2* :larm :inverse-kinematics *roll-2-left-cube*)    
    (setq *roll-2-pos* (send *pr2* :angle-vector))
    
    (setq *roll-3-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -700 900)))
    (send *pr2* :rarm :inverse-kinematics *roll-3-right-cube*)  
    (setq *roll-3-left-cube* (make-cube 100 100 100 :pos (float-vector 700 -400 900)))
    (send *pr2* :larm :inverse-kinematics *roll-3-left-cube* :rotation-axis :z)    
    (setq *roll-3-pos* (send *pr2* :angle-vector))
    
    (setq *roll-4-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -150 850)))
    (send *pr2* :rarm :inverse-kinematics *roll-4-right-cube*)  
    (setq *roll-4-left-cube* (make-cube 100 100 100 :pos (float-vector 700 150 850)))
    (send *pr2* :larm :inverse-kinematics *roll-4-left-cube*)    
    (setq *roll-4-pos* (send *pr2* :angle-vector))
    )

  (setq *solve-toss-ik* t)
  (when (eq *solve-toss-ik* t)
    (progn ;; toss CAUTION for purple stick
      (send *pr2* :angle-vector #f(50.0 16.191 44.0774 31.227 -101.578 295.967 -52.3229 -46.5188 -16.6466 44.8379 -31.9883 -101.951 -272.998 -97.2327 394.953 0.0 5.0))
      (send *pr2* :head-neck-p :joint-angle 5)
      
      (setq *toss-start-right-cube* (make-cube 100 100 100 :pos (float-vector 500 -350 850))) ;; 500 -350 750)))
      (send *toss-start-right-cube* :rotate (* pi 0.5) :x)
      (send *toss-start-right-cube* :rotate (* pi 0.5) :z)
      (send *toss-start-right-cube* :rotate (* pi -0.4) :y)
      (send *pr2* :rarm :inverse-kinematics *toss-start-right-cube*)
      (setq *toss-start-left-cube* (make-cube 100 100 100 :pos (float-vector 500 350 850))) ;; 500 350 750)))
      (send *toss-start-left-cube* :rotate (* pi -0.5) :x)
      (send *toss-start-left-cube* :rotate (* pi -0.5) :z)
      (send *toss-start-left-cube* :rotate (* pi -0.4) :y)
      (send *pr2* :larm :inverse-kinematics *toss-start-left-cube*)
      (setq *toss-start-pos* (send *pr2* :angle-vector))
      ;;
      ;;;;(setq *toss-mid-right-cube* (make-cube 100 100 100 :pos (float-vector 450 -450 1200)))
      ;;;;(send *toss-mid-right-cube* :rotate (* pi 0.5) :x)
      ;;;;(send *toss-mid-right-cube* :rotate (* pi 0.5) :z)
      ;;;;;;(send *toss-mid-right-cube* :rotate (* pi 0.2) :y)                        
      ;;;;(send *pr2* :rarm :inverse-kinematics *toss-mid-right-cube*)
      ;;;;(setq *toss-mid-left-cube* (make-cube 100 100 100 :pos (float-vector 450 450 1200)))
      ;;;;(send *toss-mid-left-cube* :rotate (* pi -0.5) :x)
      ;;;;(send *toss-mid-left-cube* :rotate (* pi -0.5) :z)
      ;;;;;;(send *toss-mid-left-cube* :rotate (* pi 0.3) :y)                              
      ;;;;(send *pr2* :larm :inverse-kinematics *toss-mid-left-cube*)
      ;;;;(setq *toss-mid-pos* (send *pr2* :angle-vector))
      ;;
      (setq *toss-goal-right-cube* (make-cube 100 100 100 :pos (float-vector 500 -730 1250))) ;; 500 -730 1250
      (send *toss-goal-right-cube* :rotate (* pi 0.5) :x)
      (send *toss-goal-right-cube* :rotate (* pi 0.5) :z)
      (send *toss-goal-right-cube* :rotate (* pi -0.4) :y)
      ;;(send *toss-goal-right-cube* :rotate (* pi -0.2) :x)                              
      (send *pr2* :rarm :inverse-kinematics *toss-goal-right-cube*)
      (setq *toss-goal-left-cube* (make-cube 100 100 100 :pos (float-vector 500 730 1250))) ;; 500 730 1250
      (send *toss-goal-left-cube* :rotate (* pi -0.5) :x)
      (send *toss-goal-left-cube* :rotate (* pi -0.5) :z)
      (send *toss-goal-left-cube* :rotate (* pi -0.4) :y)
      ;;(send *toss-goal-left-cube* :rotate (* pi 0.2) :x)
      (send *pr2* :larm :inverse-kinematics *toss-goal-left-cube*)
      (setq *toss-goal-pos* (send *pr2* :angle-vector))
      
      (setq *toss-save-right-cube* (make-cube 100 100 100 :pos (float-vector 580 -720 1000))) ;; 580 -720
      (send *toss-save-right-cube* :rotate (* pi 0.5) :x)
      (send *toss-save-right-cube* :rotate (* pi 0.5) :z)
      (send *toss-save-right-cube* :rotate (* pi -0.4) :y)                        
      (send *pr2* :rarm :inverse-kinematics *toss-save-right-cube*)
      (setq *toss-save-left-cube* (make-cube 100 100 100 :pos (float-vector 580 720 1000))) ;; 580 720
      (send *toss-save-left-cube* :rotate (* pi -0.5) :x)
      (send *toss-save-left-cube* :rotate (* pi -0.5) :z)
      (send *toss-save-left-cube* :rotate (* pi -0.4) :y)                              
      (send *pr2* :larm :inverse-kinematics *toss-save-left-cube*)
      (setq *toss-save-pos* (send *pr2* :angle-vector))
      
      )
    )
  )

;;
;; toss
;;
(defun toss-control (y-diff) ;; |right-x-diff| < *max-x-dif*(= 220)
  (setq *toss-save-right-diff-cube* (send (send *toss-save-right-cube* :copy-coords) :translate (float-vector 0 (- y-diff) 0)))
  (setq *toss-save-left-diff-cube* (send (send *toss-save-left-cube* :copy-coords) :translate (float-vector 0 y-diff 0)))

  (send *pr2* :rarm :inverse-kinematics *toss-save-right-diff-cube* :stop 3 :revert-if-fail nil)
  (send *pr2* :larm :inverse-kinematics *toss-save-left-diff-cube* :stop 3 :revert-if-fail nil)
  )

(defun toss-only ()
  ;;(send *ri* :angle-vector *toss-start-pos* 3000)
  ;;(send *ri* :wait-interpolation)
  (send *ri* :angle-vector *toss-goal-pos* 400 :default-controller 0 :end-coords-interpolation t :min-time 0.1)  
  (send *ri* :wait-interpolation-smooth 450)  
  (send *ri* :angle-vector *toss-save-pos* 800)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector *toss-start-pos* 3000)  
  )

(defun toss ()
  (send *ri* :angle-vector *toss-start-pos* 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector *toss-goal-pos* 400 :default-controller 0 :end-coords-interpolation t :min-time 0.1)  
  (send *ri* :wait-interpolation-smooth 500)  
  (send *ri* :angle-vector *toss-save-pos* 800)

  (setq *toss-max-y-diff* 100)
  (setq *toss-min-y-diff* -300)  

  (setq *now-time* 0)
  (do-until-key

   (setq *past-time* *now-time*)
   (setq *now-time* (/ (send (ros::time-now) :to-nsec) 1000000))
   (print (- *now-time* *past-time*))
   
   ;; subscribe
   (ros::spin-once "toss")

   ;; control
   (setq *toss-control-y-diff* (- (* *toss-diabolo-x-predict* 1000) 580))

   ;; max min of control diff
   (if (> *toss-control-y-diff* *toss-max-y-diff*)
       (setq *toss-control-y-diff* *toss-max-y-diff*))
   (if (< *toss-control-y-diff* *toss-min-y-diff*)
       (setq *toss-control-y-diff* *toss-min-y-diff*))

   ;; :angle-vector
   (setq *toss-ec* (toss-control *toss-control-y-diff*))
   (send *ri* :angle-vector *toss-ec* 500 :default-controller 0 :min-time 0.3)
   )

  (send *ri* :angle-vector *toss-start-pos* 3000)
  )

;;
;; idle
;;
(defun idle-control (right-x-diff) ;; |right-x-diff| < *max-x-dif*(= 220)
  ;; set z
  (setq *just-now-time* (/ (send (ros::time-now) :to-nsec) 1000000))
  (if (> (- *just-now-time* *now-time*) 1300) ;;700) ;; 1300
      (progn
	(setq *now-time* *just-now-time*)
	(setq *right-z-diff* (- *right-z-diff*))
	)
    )
  
  (setq *default-right-diff-cube* (send (send *default-right-cube* :copy-coords) :translate (float-vector right-x-diff 0 *right-z-diff*))) ;; TODO y-diff
  (setq *default-left-diff-cube* (send (send *default-left-cube* :copy-coords) :translate (float-vector (- right-x-diff) 0 (- *right-z-diff*)))) ;; TODO y-diff

  (send *default-right-diff-cube* :rotate 0.2 :x)
  (send *default-left-diff-cube* :rotate -0.2 :x)  
  
  (send *pr2* :rarm :inverse-kinematics *default-right-diff-cube*)
  (send *pr2* :larm :inverse-kinematics *default-left-diff-cube*)
  )

(defun idle (&optional (roll-valid t) (pitch-valid t) (yaw-valid t) &key (teach nil) (particle-system nil))
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
  (send *ri* :angle-vector (idle-control 0) 1500) ;;800)
  (send *pr2* :head-neck-p :joint-angle 50)  
  
  (do-until-key
   ;; subscribe
   (ros::spin-once "idle")
   
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
	 (if (eq particle-system t) ;; when particle-system
	     (progn
	       (setq *idle-control-x-diff* (- (* *particle-system-input-arm* 1000) 700))
	       (print *particle-system-input-arm*)
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
     (send *ri* :angle-vector (idle-control *idle-control-x-diff*) 1500) ;;800)
     )

   (progn ;; yaw
     (if (eq teach t) ;; when teach
	 (progn
	   (setq *joy-base-input* (elt *joy* 0))
	   (setq *idle-diabolo-yaw* (* *joy-base-input* 30))
	   )
       (progn
	 (if (eq particle-system t) ;; when particle system
	     (progn
	       (setq *idle-diabolo-yaw* (* (/ (* *particle-system-input-base* 180) 3.14) 100)) ;;TODOTODO   magic number 100
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
     ;;(setq *diff-x* 0)
     ;;(setq *diff-y* 0)

     ;; 実機での動作
     (if (eq yaw-valid t)
	 (progn
	   (progn
	     ;;(setq *diff-theta* *diabolo-yaw*)
	     ;;(send *ri* :go-pos-unsafe *diff-x* *diff-y* *diff-theta*)
	     )
	   (progn
	     (setq *diff-theta* *idle-diabolo-yaw-rad*)
	     (setq *goal* (send *ri* :move-trajectory *diff-x* *diff-y* *diff-theta* 1000)) ;;10000))
	     (send *ac* :send-goal *goal*)
	     )
	   )
       )
     )
   )
  
  (send *pr2* :rarm :inverse-kinematics *default-right-cube*)  
  (send *pr2* :larm :inverse-kinematics *default-left-cube*)   
  (setq *default-pos* (send *pr2* :angle-vector))
  (send *ri* :angle-vector *default-pos* 5000)

  (if (eq yaw-valid t)
      (progn
      (setq *goal* (send *ri* :move-trajectory 0 0 0 10000))
      (send *ac* :send-goal *goal*)
      )
    )
  )
