;; rosrun

;; 30FPS出るpointsをpublish
;; roslaunch relay_kinect.launch

;; Kinectの座標を変換して/tf_transform_cloud/outputにpublish
;; rosrun jsk_pcl_ros tf_transform_cloud ~input:=/kinect_head_remote/depth_registered/points _target_frame_id:=base_footprint

;; ディアボロの傾きを計算
;; rosrun pr2_juggle sample_pcl ~input:=/tf_transform_cloud/output

;; システム同定のためのロガー
;; python logger.py

;; sudo service chrony restart

;; HSIフィルタを使いたい時
;; rosrun jsk_pcl_ros hsi_color_filter ~input:=/kinect_head/depth_registered/quater/throttled/points

(defun init ()
  (setq *diabolo-pitch* 0)
  (setq *diabolo-yaw* 0)  
  
  (ros::load-ros-manifest "roseus")
  (ros::roseus "listener")
  (ros::subscribe "sample_pcl/diabolo/pitch" std_msgs::float64
		  #'(lambda (msg) (setq *diabolo-pitch* (send msg :data))))
  (ros::subscribe "sample_pcl/diabolo/yaw" std_msgs::float64
		  #'(lambda (msg) (setq *diabolo-yaw* (send msg :data))))
  
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
  )

(defun control (right-x-diff) ;; |right-x-diff| < *max-x-dif*(= 220)
  ;; set z
  (setq *just-now-time* (/ (send (ros::time-now) :to-nsec) 1000000))
  (if (> (- *just-now-time* *now-time*) 1300) ;; 700
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

(defun idle (&optional (roll-valid t) (pitch-valid t) (yaw-valid t))
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
  
  (do-until-key
   ;; subscribe
   (ros::spin-once)
   
   (progn ;; pitch
     ;; P制御
     (setq *control-x-diff* (* *diabolo-pitch* 10)) ;; TODO params

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
     (send *ri* :angle-vector (control *control-x-diff*) 1500)
     )

   (progn ;; yaw
     ;; 外れ値除去
     (if (> *diabolo-yaw* 60)
	 (setq *diabolo-yaw* 0))
     (if (< *diabolo-yaw* -60)
	 (setq *diabolo-yaw* 0))
     
     ;; 最大角度制限
     (if (> *diabolo-yaw* 20)
	 (setq *diabolo-yaw* 20))
     (if (< *diabolo-yaw* -20)
	 (setq *diabolo-yaw* -20))
     
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
	     (setq *goal* (send *ri* :move-trajectory *diff-x* *diff-y* *diff-theta* 10000))
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
