;; toss_diabolo.launch   ;; launch files for toss
;;   roslaunch relay_kinect.launch   ;;30FPS出るpointsをpublish
;;   rosrun jsk_pcl_ros tf_transform_cloud ~input:=/kinect_head_remote/depth_registered/points _target_frame_id:=base_footprint   ;;Kinectの座標を変換して/tf_transform_cloud/outputにpublish
;;   rosrun pr2_juggle calc_toss_diabolo_pos   ;;ディアボロの座標を計算
;; rosrun rviz rviz -d ~/.rviz/toss.rviz

(defun toss-control (y-diff) ;; |right-x-diff| < *max-x-dif*(= 220)
  (setq *toss-save-right-diff-cube* (send (send *toss-save-right-cube* :copy-coords) :translate (float-vector 0 (- y-diff) 0)))
  (setq *toss-save-left-diff-cube* (send (send *toss-save-left-cube* :copy-coords) :translate (float-vector 0 y-diff 0)))

  (send *pr2* :rarm :inverse-kinematics *toss-save-right-diff-cube* :stop 3 :revert-if-fail nil)
  (send *pr2* :larm :inverse-kinematics *toss-save-left-diff-cube* :stop 3 :revert-if-fail nil)
  )

(defun toss-only ()
  ;;(send *ri* :angle-vector *toss-start-pos* 3000)
  ;;(send *ri* :wait-interpolation)
  (send *ri* :angle-vector *toss-goal-pos* 400 :default-controller 0 :end-coords-interpolation t :min-time 0.2)    
  (send *ri* :wait-interpolation-smooth 250)  
  (send *ri* :angle-vector *toss-save-pos* 800)
  ;;(send *ri* :wait-interpolation)
  ;;(send *ri* :angle-vector *toss-start-pos* 3000)  
  )

(defun toss ()
  ;;(send *ri* :angle-vector *toss-start-pos* 3000)
  ;;(send *ri* :wait-interpolation)
  ;;(send *ri* :angle-vector *toss-goal-pos* 400 :default-controller 0 :end-coords-interpolation t :min-time 0.1)  
  ;;(send *ri* :wait-interpolation-smooth 500)  
  ;;(send *ri* :angle-vector *toss-save-pos* 800)

  (setq *toss-max-y-diff* 100)
  (setq *toss-min-y-diff* -300)  

  (setq *now-time* 0)
  (setq *toss-cnt* 0)
  (do-until-key
   (setq *toss-cnt* (+ *toss-cnt* 1))
   (setq *past-time* *now-time*)
   (setq *now-time* (/ (send (ros::time-now) :to-nsec) 1000000))
   (print (list "one loop time" (- *now-time* *past-time*)))
   
   ;; subscribe
   (ros::spin-once "toss")

   ;; control
   (setq *toss-control-y-diff* (- (* *toss-diabolo-x* 1000) 580))

   ;; max min of control diff
   (if (> *toss-control-y-diff* *toss-max-y-diff*)
       (setq *toss-control-y-diff* *toss-max-y-diff*))
   (if (< *toss-control-y-diff* *toss-min-y-diff*)
       (setq *toss-control-y-diff* *toss-min-y-diff*))

   ;; :angle-vector
   (setq *toss-ec* (toss-control *toss-control-y-diff*))
   (send *pr2* :angle-vector *toss-ec*)
   ;;(toss-control *toss-control-y-diff*)
   ;;(print (list *toss-diabolo-x* *toss-control-y-diff*))
   ;;(print *toss-ec*)   
   ;;(print (send *pr2* :larm :angle-vector))
   ;;(bench (send *ri* :angle-vector *toss-ec* 500 :default-controller 0 :min-time 0.3))
   (if (eq (mod *toss-cnt* 5) 0)
       (bench (progn (publish-left-angle-vector 700)  (publish-right-angle-vector 700)))
     )
   )
  
  (send *pr2* :angle-vector *toss-start-pos*)
  (bench (progn (publish-left-angle-vector 3000)  (publish-right-angle-vector 3000)))
  )

