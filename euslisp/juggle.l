;; idle_diabolo.laucnh

;; toss_diabolo.launch

;; utils
;;   python diabolo_system.py -m ../log/diabolo_system/goodmodel_612_0.h5  ;; train, test, simulate, calc next optimized input by using self-calculated model
;;   python state-input-logger.py   ;; logger of state(pitch, yaw) and input(arm, base)

;; when record rosbag
;;   rosbag record -O diabolo.bag  /calc_idle_diabolo_state/diabolo_points /calc_idle_diabolo_state/diabolo_state /calc_idle_diabolo_state/pitch_points /calc_idle_diabolo_state/marker_cube /calc_idle_diabolo_state/marker_mid /calc_idle_diabolo_state/marker_state /tf
;; when play rosbag
;;   rosbag play diabolo.bag
;;   roslaunch pr2_description upload_pr2.launch  ;; upload robot model after play rosbag

;; simulate idle using diabolo system
;;   idle_diabolo_simulate.launch
;;     roslaunch pr2_description upload_pr2.launch
;;     roseus juggle.l
;;       idle nil t t :simulate t :diabolo-system t ;; subscribe diabolo-system-input and send *ri*
;;       (idle nil t t :simulate t ;; subscribe state and calc input and send *ri*)
;;     rosrun jsk_diaoblo_pr2 publish_diabolo_model   ;; publish diabolo dae model
;;     roslaunch jsk_diabolo_pr2 robot_state_publisher   ;; translate joint_states to tf for RViz
;;     rosrun rviz rviz -d ~/.rviz/idle_simulate.rviz
;;   python diabolo_system.py -m ../log/diabolo_system/goodmodel_612_0.h5 -a 1 ;; simulate(calc and publish next state) and optimize_input(calc and publish next input) 


;;  TODO
;;    :angle-vectorが遅い
;;      topicに直接publishする
;;    falling diaboloの予測
;;    toss
;;    use local variable

(ros::load-ros-manifest "control_msgs")

(defun init ()
  (ros::load-ros-manifest "roseus")
  (ros::roseus "juggle-system")

  ;;
  ;; init variables
  ;;
  (setq *idle-diabolo-pitch* 0)
  (setq *idle-diabolo-yaw* 0)
  
  (setq *toss-diabolo-x* 0)
  (setq *toss-diabolo-x-predict* 0)
  
  (setq *diabolo-system-input-arm* 0)
  (setq *diabolo-system-input-base* 0)
  
  ;;
  ;; Subscriber
  ;;
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
  (ros::subscribe "calc_toss_diabolo_pos/diabolo_pos_x" std_msgs::Float64
		  #'(lambda (msg) (progn
				    (setq *toss-diabolo-x* (send msg :data))
				    ))
		  :groupname "toss")
  
  ;; subscribe joy
  (ros::create-nodehandle "joy")
  (ros::subscribe "joy" sensor_msgs::Joy
  		  #'(lambda (msg) (setq *joy* (send msg :axes)))
		  :groupname "joy")

  ;; subscribe DiaboloSystem
  (ros::create-nodehandle "diabolo-system")
  (ros::subscribe "diabolo_system/diabolo_input" std_msgs::Float64MultiArray
  		  #'(lambda (msg) (progn
				    (setq *diabolo-system-input-arm* (elt (send msg :data) 0))
				    (setq *diabolo-system-input-base* (elt (send msg :data) 1))
				    ))
		  :groupname "diabolo-system")
  (ros::create-nodehandle "diabolo-system-simulate")
  (ros::subscribe "diabolo_system/diabolo_state" std_msgs::Float64MultiArray
  		  #'(lambda (msg) (progn
				    (setq *idle-diabolo-pitch* (elt (send msg :data) 0))
				    (setq *idle-diabolo-yaw* (elt (send msg :data) 1))
				    ))
		  :groupname "diabolo-system-simulate")

  ;;
  ;; Publisher
  ;;
  ;; publish whether idle or not
  (ros::advertise "idle" std_msgs::float64 1)

  ;; publish larm
  (ros::advertise "/l_arm_controller/follow_joint_trajectory/goal" control_msgs::FollowJointTrajectoryActionGoal 1)  

  ;;
  ;; init PR2
  ;;
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init)
  
  (send *ri* :angle-vector (send *pr2* :reset-pose) 5000)
  (setq *ac* (pr2-interface-move-base-trajectory-action *ri*))
  
  (send *ri* :start-grasp)
  (send *pr2* :head-neck-p :joint-angle 50)

  (solve-ik)
  
  (objects (list *pr2*))
  
  (load "package://jsk_diabolo_pr2/euslisp/utils.l")
  (load "package://jsk_diabolo_pr2/euslisp/idle.l")
  (load "package://jsk_diabolo_pr2/euslisp/toss.l")
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

      ;; start
      (setq *toss-start-right-cube* (make-cube 100 100 100 :pos (float-vector 500 -350 850))) ;; 500 -350 850)))
      (send *toss-start-right-cube* :rotate (* pi 0.5) :x)
      (send *toss-start-right-cube* :rotate (* pi 0.5) :z)
      (send *toss-start-right-cube* :rotate (* pi -0.4) :y)
      (send *pr2* :rarm :inverse-kinematics *toss-start-right-cube*)
      (setq *toss-start-left-cube* (make-cube 100 100 100 :pos (float-vector 500 350 850))) ;; 500 350 850)))
      (send *toss-start-left-cube* :rotate (* pi -0.5) :x)
      (send *toss-start-left-cube* :rotate (* pi -0.5) :z)
      (send *toss-start-left-cube* :rotate (* pi -0.4) :y)
      (send *pr2* :larm :inverse-kinematics *toss-start-left-cube*)
      (setq *toss-start-pos* (send *pr2* :angle-vector))

      ;; goal
      (setq *toss-goal-right-cube* (make-cube 100 100 100 :pos (float-vector 500 -740 1250))) ;; 500 -730 1250
      (send *toss-goal-right-cube* :rotate (* pi 0.5) :x)
      (send *toss-goal-right-cube* :rotate (* pi 0.5) :z)
      (send *toss-goal-right-cube* :rotate (* pi -0.4) :y)
      ;;(send *toss-goal-right-cube* :rotate (* pi -0.2) :x)                              
      (send *pr2* :rarm :inverse-kinematics *toss-goal-right-cube*)
      (setq *toss-goal-left-cube* (make-cube 100 100 100 :pos (float-vector 500 740 1250))) ;; 500 730 1250
      (send *toss-goal-left-cube* :rotate (* pi -0.5) :x)
      (send *toss-goal-left-cube* :rotate (* pi -0.5) :z)
      (send *toss-goal-left-cube* :rotate (* pi -0.4) :y)
      ;;(send *toss-goal-left-cube* :rotate (* pi 0.2) :x)
      (send *pr2* :larm :inverse-kinematics *toss-goal-left-cube*)
      (setq *toss-goal-pos* (send *pr2* :angle-vector))

      ;; save
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
