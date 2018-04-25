;; rosrun

;; Kinectの座標を変換して/tf_transform_cloud/outputにpublish
;; rosrun jsk_pcl_ros tf_transform_cloud ~input:=/kinect_head/depth_registered/quater/throttled/points _target_frame_id:=base_footprint
;; (rosrun jsk_pcl_ros tf_transform_cloud ~input:=/kinect_head/depth_registered/points _target_frame_id:=base_footprint)

;; ディアボロの傾きを計算
;; rosrun pr2_juggle sample_pcl ~input:=/tf_transform_cloud/output


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

  (setq *ac* (pr2-interface-move-base-trajectory-action *ri*))
  
  (send *ri* :start-grasp)
  (send *pr2* :head-neck-p :joint-angle 50)
  
  (objects (list *pr2*))

  (progn  ;; default 500~900
    (setq *default-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -120 1050)))
    (send *pr2* :rarm :inverse-kinematics *default-right-cube*)  
    (setq *default-left-cube* (make-cube 100 100 100 :pos (float-vector 700 120 1050)))
    (send *pr2* :larm :inverse-kinematics *default-left-cube*)   
    (setq *default-pos* (send *pr2* :angle-vector))
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
    
    (setq *roll-4-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -150 900)))
    (send *pr2* :rarm :inverse-kinematics *roll-4-right-cube*)  
    (setq *roll-4-left-cube* (make-cube 100 100 100 :pos (float-vector 700 150 900)))
    (send *pr2* :larm :inverse-kinematics *roll-4-left-cube*)    
    (setq *roll-4-pos* (send *pr2* :angle-vector))
    
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

  (progn ;; jump
    (setq *jump-start-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -150 1000)))
    (send *pr2* :rarm :inverse-kinematics *jump-start-right-cube*)  
    (setq *jump-start-left-cube* (make-cube 100 100 100 :pos (float-vector 700 150 1000)))
    (send *pr2* :larm :inverse-kinematics *jump-start-left-cube*)    
    (setq *jump-1-pos* (send *pr2* :angle-vector))

    (setq *jump-end-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -600 1000)))
    (send *pr2* :rarm :inverse-kinematics *jump-end-right-cube*)
    (setq *jump-end-left-cube* (make-cube 100 100 100 :pos (float-vector 700 600 1000)))
    (send *pr2* :larm :inverse-kinematics *jump-end-left-cube*)   
    (setq *jump-2-pos* (send *pr2* :angle-vector))
    )
  
  (objects (list *pr2*))
  )


(defun roll ()
  (send *ri* :angle-vector *roll-1-pos* 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector *roll-2-pos* 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector *roll-3-pos* 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector *roll-2-pos* 1500)
  )

(defun idle ()
  (send *ri* :angle-vector-sequence (list *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos*) (list 1000))  
  )

;; roll to idle
(defun roll-idle ()
  (send *ri* :angle-vector *roll-1-pos* 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector *roll-2-pos* 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector *roll-3-pos* 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-sequence (list *roll-4-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos*) (list 800))  
  )

;; roll-stable
(defun roll-stable (&optional (pitch-valid t) (yaw-valid t))
  (send *ri* :angle-vector *roll-1-pos* 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector *roll-2-pos* 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector *roll-3-pos* 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector *roll-4-pos* 200)

  (pitch-yaw-stable pitch-valid yaw-valid)
  )

(defun control (right-diff) ;; |right-diff| < *max-dif*(= 220)
  (setq *default-right-diff-cube* (send (send *default-right-cube* :copy-coords) :translate (float-vector right-diff 0 0)))
  (setq *default-left-diff-cube* (send (send *default-left-cube* :copy-coords) :translate (float-vector (- right-diff) 0 0)))  
  (send *pr2* :rarm :inverse-kinematics *default-right-diff-cube*)
  (send *pr2* :larm :inverse-kinematics *default-left-diff-cube*)  
  )

(defun pitch-yaw-stable (&optional (pitch-valid t) (yaw-valid t))
  (setq *control-diff* 0)
  ;;(setq *max-diff* 220) ;;TODO
  (setq *max-diff* 200)  
  (setq *min-diff* 0)
  
  (do-until-key
   ;; subscribe
   (ros::spin-once)
   
   (progn ;; pitch
     ;; P制御
     (setq *control-diff* (* *diabolo-pitch* 20)) ;; TODO params

     ;; 最大最小制限
     (if (> *control-diff* *max-diff*)
	 (setq *control-diff* *max-diff*))
     (if (< *control-diff* (- *max-diff*))
	 (setq *control-diff* (- *max-diff*)))

     ;; 不感帯
     (if (< *control-diff* *min-diff*)
	 (if (> *control-diff* (- *min-diff*))
	     (setq *control-diff* 0)))

     ;; 実機での動作
     (if (eq pitch-valid t)
	 (send *ri* :angle-vector (control *control-diff*) 2000)
       )
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
     
     ;;(if (> *diabolo-yaw* 0)
     ;;    (setq *diabolo-yaw* 5))
     ;;(if (< *diabolo-yaw* 0)
     ;;    (setq *diabolo-yaw* -5))
     
     (setq *radius* (/ (elt (send (send *pr2* :rarm :end-coords) :worldpos) 0) 1000)) ;; [m]
     (setq *radius* (* *radius* 0.7)) ;; TODO
     (setq *diabolo-yaw-rad* (/ (* *diabolo-yaw* 3.14) 180))
     
     ;;(setq *diff-x* (* *radius* (- 1 (cos *diabolo-yaw-rad*))))
     ;;(setq *diff-y* (* *radius* (sin (- *diabolo-yaw-rad*))))
     (setq *diff-x* 0)
     (setq *diff-y* 0)

     ;; 実機での動作
     (if (eq yaw-valid t)
	 (progn
	   (progn
	     (setq *diff-theta* *diabolo-yaw*)
	     (send *ri* :go-pos-unsafe *diff-x* *diff-y* *diff-theta*)
	     )
	   (progn
	     ;;(setq *diff-theta* *diabolo-yaw-rad*)
	     ;;(setq *goal* (send *ri* :move-trajectory *diff-x* *diff-y* *diff-theta* 10000))
	     ;;(send *ac* :send-goal *goal*)
	     )
	   )
       )
     )

   )
  (send *pr2* :rarm :inverse-kinematics *default-right-cube*)  
  (send *pr2* :larm :inverse-kinematics *default-left-cube*)   
  (setq *default-pos* (send *pr2* :angle-vector))
  (send *ri* :angle-vector *default-pos* 5000)
  )

;;(defun idle-infinite ()
;;  (send *ri* :angle-vector-sequence
;;	(list *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos*
;;	      *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos*
;;	      *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos*
;;	      *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos* *idle-2-pos* *idle-1-pos*)
;;	(list 1000)
;;	)  
;;  )

;;(defun jump ()
;;  (send *ri* :angle-vector *jump-1-pos*)
;;  (send *ri* :wait-interpolation)
;;  (send *ri* :angle-vector *jump-2-pos* 1000)  
;;  )

;;(defun jump-interpolation ()
;;  (send *ri* :angle-vector *jump-1-pos*)
;;  (send *ri* :wait-interpolation)
;;  (dotimes (i 10)
;;    (send *ri* :angle-vector (midpoint (* 0.1 (+ i 1)) *jump-1-pos* *jump-2-pos*) 500))
;;  )


