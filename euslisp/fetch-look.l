(load "package://virtual_force_drag/euslisp/head-look-at.l")
(load "package://fetcheus/fetch-interface.l")

(ros::roseus "look")


(defclass fetch-head-look-at
  :super head-look-at)

(defmethod fetch-head-look-at
    (:init ()
     (send-super :init)
     (setq input-file (ros::get-param "~input_file" nil))
     (setq output-file (ros::get-param "~output_file" "output.yaml"))
     (if input-file
         (unix:system (format nil "rosparam load ~A look" input-file))
         (force-calibration output-file))
     
     (fetch-init nil)
     (ros::subscribe "~input" geometry_msgs::WrenchStamped #'send self :wrench-cb))

  (:look (x y)
         (when look-enable
           (let (pan tilt)
             (setq pan (rad2deg (atan y x)))
             (setq tilt 0)
             (if (> pan 0)
                 (setq pan (min pan 90))
                 (setq pan (max pan -90)))
             (send *robot* :head :angle-vector (float-vector pan tilt))
             (send *ri* :angle-vector-raw (send *robot* :angle-vector) 2000 :head-controller)))))

(setq *head-look-at* (instance fetch-head-look-at :init))
(ros::rate 30)
(do-until-key
    (ros::spin-once)
  (send *head-look-at** :run)
  (ros::sleep))




