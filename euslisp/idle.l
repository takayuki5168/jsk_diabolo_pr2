;;(setq *demo* (ros::get-param "/diabolo_euslisp_idle/demo"))
;;(setq *trigger* (ros::get-param "/diabolo_euslisp_idle/trigger"))

;; init
(load "juggle.l")
(init)

(do-until-key)

;; solve IK
(solve-ik)

;; move y
(send *ri* :go-pos-unsafe 0 1 0)

;; move x
(send *ri* :go-pos-unsafe 1 0 0)
	  
;; move theta
(send *ri* :go-pos-unsafe 0 0 90)
					
;; sleep
(unix::sleep 5)

;; diabolo
(idle t t t)

#|
(if (eq *trigger* t)
    ;; when trigger is needed
    (progn
      (ros::subscribe "diabolo_demo/trigger" std_msgs::Bool
		      #'(lambda (msg) (progn
          ;; solve IK
          (solve-ik)
	  
	  ;; move y
	  (send *ri* :go-pos-unsafe 0 -1 0)
	  
	  ;; move theta
	  (send *ri* :go-pos-unsafe 0 0 -90)
					
          ;; sleep
          (unix::sleep 3)

	  ;; diabolo
	  (idle t t t)
	  )))
      (do-until-key
       (ros::spin-once)
       )
      )
  ;; when trigger is not needed
  (progn
    (if (eq *demo* t)
	(progn
	  (require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
	  (send *ri* :wait-interpolation)

	  ;;(speak-jp "今からディアボロを披露します")
	  (speak-en "Now I will show you jugging")
	  (unix::usleep 2000000)
	  ;;(speak-jp "スティックとディアボロを用意してください")
	  (speak-en "Please prepare stick and chinese yoyo")

	  ;; default-pose
	  (send *ri* :angle-vector *default-pos* 3000)
	  (send *ri* :wait-interpolation)

	  ;; right hand
	  ;;(speak-jp "右手にスティックを")
	  (speak-en "give me one stick for my right hand")
	  (send *ri* :stop-grasp :rarm)
	  (unix::usleep 3000000)      
	  (wait-for-hand-impact :rarm)
	  (send *ri* :start-grasp :rarm)

	  ;; left hand
	  ;;(speak-jp "左手にスティックを")
	  (speak-en "give me another stick for my left hand")
	  (send *ri* :stop-grasp :larm)
	  (unix::usleep 3000000)
	  (wait-for-hand-impact :larm)
	  (send *ri* :start-grasp :larm)

	  ;; set diabolo
	  ;;(speak-jp "ディアボロを紐に一周巻いてください")
	  (speak-en "wind the yoyo around string")
	  (unix::usleep 2500000)      
	  ;;(speak-jp "ディアボロの中心の黒い部分が手前になるように注意してください")
	  (wait-for-hand-impact :larm)
	  (wait-for-hand-impact :rarm)      

	  ;; let's diabolo
	  ;;(speak-jp "ディアボロをします")      
	  (speak-en "Now I will start")

	  (unix::usleep 3000000)      
	  )
      (progn
	(unix::usleep 3000000)
	)
      )
    (idle t t t)      
    )
  )
|#
