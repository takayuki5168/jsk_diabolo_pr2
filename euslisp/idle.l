;; FIX
(setq *demo* t)

(init)

(if (eq *demo* t)
    (progn
      (require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
      
      (speak-jp "今からディアボロを披露します")
      (speak-jp "スティックとディアボロを用意してください")

      (send *ri* :angle-vector *default-pos* 3000)

      ;; right hand
      (speak-jp "右手にスティックを")
      (send *ri* :stop-grasp :rarm)      
      (wait-for-hand-impact :arm rarm)
      (send *ri* :start-grasp :rarm)

      ;; left hand
      (speak-jp "左手にスティックを")
      (send *ri* :stop-grasp :larm)            
      (wait-for-hand-impact :arm rarm)      
      (send *ri* :start-grasp :larm)      
      
      (speak-jp "ディアボロを紐に一周巻いてください")
      (speak-jp "ディアボロの中心の黒い部分が手前になるように注意してください")
      (wait-for-hand-impact)

      (speak-jp "ディアボロをします")      
      (unix::usleep 3000)      
      )
  (progn
    (unix::usleep 3000)
    )
  )

(idle t t t)
