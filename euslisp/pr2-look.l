(load "package://virtual_force_drag/euslisp/head-look-at.l")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "look")

(defclass pr2-head-look-at
  :super head-look-at)

(defmethod pr2-head-look-at
    (:init ()
     (send-super :init)
     (setq input-file (ros::get-param "~input_file" nil))
     (setq output-file (ros::get-param "~output_file" "output.yaml"))
     (if input-file
         (unix:system (format nil "rosparam load ~A look" input-file))
         (force-calibration output-file))
     
     (pr2-init nil)
     (ros::subscribe "~input" geometry_msgs::WrenchStamped #'send self :wrench-cb))

  (:look (x y)
         (when look-enable
           (let (pan tilt)
             (setq pan (rad2deg (atan y x)))
             (setq tilt 0)
             (if (> pan 0)
                 (setq pan (- (min pan 165) 50))
                 (setq pan (+ 50 (max pan -165))))
             (send *pr2* :head :angle-vector (float-vector pan tilt))
             (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :head-controller)))))

(setq *head-look-at* (instance pr2-head-look-at :init))
(ros::rate 30)
(do-until-key
    (ros::spin-once)
  (send *head-look-at** :run)
  (ros::sleep))

