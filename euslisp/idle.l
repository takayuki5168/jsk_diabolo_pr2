(load "package://pr2_juggle/juggle.l")
(init)

;; FIX
(setq *perform* t)

(if (eq *perform* t)
    (progn
      (speak-jp "今からディアボロを披露します")
      (speak-jp "スティックとディアボロを用意してください")

      ;;(send *ri* :angle-vector )
      (speak-jp "右手にスティックを")
      ;; if hand shock
      (send *ri* :start-grasp :rarm)
      
      ;;(send *ri* :angle-vector )
      (speak-jp "左手にスティックを")
      ;; if hand shock
      (send *ri* :start-grasp :larm)      
      
      (speak-jp "ディアボロを紐に一周巻いてください")
      (speak-jp "ディアボロの中心の黒い部分が手前になるように注意してください")
      ;; if hand shock

      
      )
  (progn
    (unix::usleep 3000)
    )
  )

(idle t t t)
