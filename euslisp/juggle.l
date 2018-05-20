;; launchファイル起動スクリプト
;; start_launch.sh

;;     30FPS出るpointsをpublish
;;     roslaunch relay_kinect.launch

;;     Kinectの座標を変換して/tf_transform_cloud/outputにpublish
;;     rosrun jsk_pcl_ros tf_transform_cloud ~input:=/kinect_head_remote/depth_registered/points _target_frame_id:=base_footprint

;;     ディアボロの傾きを計算
;;     rosrun pr2_juggle calc_diabolo_state ~input:=/tf_transform_cloud/output


;; rvizでビジュアライズ
;; rosrun pr2_juggle diabolo_marker
;; rosrun rviz rviz

;; システム同定のためのロガー
;; python state-input-logger.py


;; sudo service chrony restart

;; rosbag を取る
;; rosbag record -O diabolo0.bag  /diabolo_marker/diabolo_marker /diabolo_marker/diabolo_cube_marker /calc_diabolo_state/points /tf

(defun init ()
  (setq *diabolo-pitch* 0)
  (setq *diabolo-yaw* 0)
  (setq *gmm-input-arm* 0)
  (setq *gmm-input-base* 0)
  
  (ros::load-ros-manifest "roseus")

  ;;
  ;; Subscriber
  ;;
  (ros::roseus "listener")

  ;; subscribe diabolo
  (ros::subscribe "calc_diabolo_state/pitch" std_msgs::float64
		  #'(lambda (msg) (setq *diabolo-pitch* (send msg :data))))
  (ros::subscribe "calc_diabolo_state/yaw" std_msgs::float64
		  #'(lambda (msg) (setq *diabolo-yaw* (send msg :data))))

  ;; subscribe joy
  (ros::subscribe "joy" sensor_msgs::Joy
		  #'(lambda (msg) (setq *joy* (send msg :axes))))

  ;; subscribe gmm
  (ros::subscribe "particle_system/arm" std_msgs::float64
		  #'(lambda (msg) (setq *gmm-input-arm* (send msg :data))))
  (ros::subscribe "particle_system/base" std_msgs::float64
		  #'(lambda (msg) (setq *gmm-input-base* (send msg :data))))

  ;;
  ;; Publisher
  ;;
  (ros::advertise "idle" std_msgs::float64 1)

  
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init)
  
  (send *ri* :angle-vector (send *pr2* :reset-pose))
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

  (setq *solve-jump-ik* t)
  (when (eq *solve-jump-ik* t)
    (progn ;; jump
      (setq *jump-start-right-cube* (make-cube 100 100 100 :pos (float-vector 600 -150 1000)))
      ;;(send *jump-start-right-cube* :rotate (+ (/ pi 2) (* pi 2)) :z)              
      (send *pr2* :rarm :inverse-kinematics *jump-start-right-cube*)
      (setq *jump-start-left-cube* (make-cube 100 100 100 :pos (float-vector 600 150 1000)))
      (send *jump-start-left-cube* :rotate (/ -pi 2) :z)                    
      (send *pr2* :larm :inverse-kinematics *jump-start-left-cube*)
      (setq *jump-1-pos* (send *pr2* :angle-vector))

      ;;(setq *jump-mid-right-cube* (make-cube 100 100 100 :pos (float-vector 600 -500 900)))
      ;;(send *pr2* :rarm :inverse-kinematics *jump-mid-right-cube*)
      ;;(setq *jump-mid-left-cube* (make-cube 100 100 100 :pos (float-vector 600 500 900)))
      ;;(send *pr2* :larm :inverse-kinematics *jump-mid-left-cube*)
      ;;(setq *jump-2-pos* (send *pr2* :angle-vector))
      ;;
      ;;
      ;;(setq *jump-mid2-right-cube* (make-cube 100 100 100 :pos (float-vector 600 -500 900)))
      ;;(send *jump-mid2-right-cube* :rotate 2.0 :x)        
      ;;(send *pr2* :rarm :inverse-kinematics *jump-mid2-right-cube*)
      ;;(setq *jump-mid2-left-cube* (make-cube 100 100 100 :pos (float-vector 600 500 900)))
      ;;(send *jump-mid2-left-cube* :rotate -2.0 :x)                
      ;;(send *pr2* :larm :inverse-kinematics *jump-mid2-left-cube*)
      ;;(setq *jump-3-pos* (send *pr2* :angle-vector))

      (setq *jump-mid3-right-cube* (make-cube 100 100 100 :pos (float-vector 600 -500 1250)))
      ;;(send *jump-mid3-right-cube* :rotate 4.0 :x)        
      (send *pr2* :rarm :inverse-kinematics *jump-mid3-right-cube*)
      (setq *jump-mid3-left-cube* (make-cube 100 100 100 :pos (float-vector 600 500 1250)))
      (send *jump-mid3-left-cube* :rotate (/ -pi ) :z)            
      (send *pr2* :larm :inverse-kinematics *jump-mid3-left-cube*)
      (setq *jump-4-pos* (send *pr2* :angle-vector))

      ;;(setq *jump-mid4-right-cube* (make-cube 100 100 100 :pos (float-vector 600 -500 900)))
      ;;;;(send *jump-mid4-right-cube* :rotate (/ pi 2) :z)    
      ;;(send *pr2* :rarm :inverse-kinematics *jump-mid4-right-cube*)
      ;;(setq *jump-mid4-left-cube* (make-cube 100 100 100 :pos (float-vector 600 500 900)))
      ;;(send *jump-mid4-left-cube* :rotate (/ pi 2) :z)    
      ;;(send *pr2* :larm :inverse-kinematics *jump-mid4-left-cube*)
      ;;(setq *jump-5-pos* (send *pr2* :angle-vector))
      ;;
      ;;(setq *jump-end-right-cube* (make-cube 100 100 100 :pos (float-vector 600 -650 1250)))
      ;;(send *jump-end-right-cube* :rotate -0.4 :x)
      ;;(send *pr2* :rarm :inverse-kinematics *jump-end-right-cube*)
      ;;(setq *jump-end-left-cube* (make-cube 100 100 100 :pos (float-vector 600 650 1250)))
      ;;(send *jump-end-left-cube* :rotate 0.4 :x)
      ;;(send *pr2* :larm :inverse-kinematics *jump-end-left-cube*)
      ;;(setq *jump-3-pos* (send *pr2* :angle-vector))
      )
    )
  )

(defun jump ()
  (send *ri* :angle-vector *jump-1-pos*)
  (send *ri* :wait-interpolation)
  ;;(send *ri* :angle-vector *jump-2-pos*)
  ;;(send *ri* :wait-interpolation)
  ;;(send *ri* :angle-vector-sequence *jump-3-pos* *jump-4-pos* 400)
  ;;(send *ri* :angle-vector *jump-4-pos* 400)
  ;;(send *ri* :wait-interpolation)
  (send *ri* :angle-vector *jump-4-pos* 1000)
  (send *ri* :wait-interpolation)  
  (send *ri* :angle-vector *jump-1-pos*)  
  )

(defun control (right-x-diff) ;; |right-x-diff| < *max-x-dif*(= 220)
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

(defun idle (&optional (roll-valid t) (pitch-valid t) (yaw-valid t) &key (teach nil) (gmm nil))
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
  
  (setq *control-x-diff* 0)
  (setq *max-x-diff* 100) ;; TODO  
  (setq *min-x-diff* 0)
  (setq *now-time* (/ (send (ros::time-now) :to-nsec) 1000000))
  (setq *right-z-diff* 150)
  (send *ri* :angle-vector (control 0) 1500) ;;800)  
  
  (do-until-key
   ;; subscribe
   (ros::spin-once)
   
   (setq idle-msg (instance std_msgs::float64 :init))   
   (send idle-msg :data 1)
   (ros::publish "idle" idle-msg)
   
   (progn ;; pitch
     ;; P制御
     (if (eq teach t) ;; when teach
	 (progn
	   (setq *joy-arm-input* (elt *joy* 3))
	   (setq *control-x-diff* (* *joy-arm-input* 100))
	   )
       (progn
	 (if (eq gmm t) ;; when gmm
	     (progn
	       (setq *control-x-diff* (- (* *gmm-input-arm* 1000) 700))
	       (print *gmm-input-arm*)
	       (print *control-x-diff*)
	       )
	   (progn
	     (setq *control-x-diff* (* *diabolo-pitch* 10)) ;; when controller TODO params
	     )
	   )
	 )
       )

     ;; 最大最小制限
     (if (> *control-x-diff* *max-x-diff*)
	 (setq *control-x-diff* *max-x-diff*))
     (if (< *control-x-diff* (- *max-x-diff*))
	 (setq *control-x-diff* (- *max-x-diff*)))

     ;; 不感帯
     (if (< *control-x-diff* *min-x-diff*)
	 (if (> *control-x-diff* (- *min-x-diff*))
	     (setq *control-x-diff* 0)))

     ;; 実機での動作
     (if (eq pitch-valid nil)
	 (setq *control-x-diff* 0)
       )
     (send *ri* :angle-vector (control *control-x-diff*) 1500) ;;800)
     )

   (progn ;; yaw
     (if (eq teach t) ;; when teach
	 (progn
	   (setq *joy-base-input* (elt *joy* 0))
	   (setq *diabolo-yaw* (* *joy-base-input* 30))
	   )
       (progn
	 (if (eq gmm t) ;; when gmm
	     (progn
	       (setq *diabolo-yaw* (* (/ (* *gmm-input-base* 180) 3.14) 100)) ;;TODOTODO   magic number 100
	       )
	   )
	 )
       )    
     ;; 外れ値除去
     (if (> *diabolo-yaw* 60)
	 (setq *diabolo-yaw* 0))
     (if (< *diabolo-yaw* -60)
	 (setq *diabolo-yaw* 0))
     
     ;; 最大角度制限
     (if (> *diabolo-yaw* 10) ;;20)
	 (setq *diabolo-yaw* 10)) ;;20))
     (if (< *diabolo-yaw* -10) ;;-20)
	 (setq *diabolo-yaw* -10)) ;;-20))
     
     (setq *radius* 0.7) ;;(/ (elt (send (send *pr2* :rarm :end-coords) :worldpos) 0) 1000)) ;; [m] ;; TODO
     (setq *radius* (* *radius* 0.8)) ;; TODO
     (setq *diabolo-yaw-rad* (/ (* *diabolo-yaw* 3.14) 180))
     
     (setq *diff-x* (* *radius* (- 1 (cos *diabolo-yaw-rad*))))
     (setq *diff-y* (* *radius* (sin (- *diabolo-yaw-rad*))))
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
	     (setq *diff-theta* *diabolo-yaw-rad*)
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
