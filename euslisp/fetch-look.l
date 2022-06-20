(load "package://virtual_force_drag/euslisp/head-look-at.l")
(load "package://fetcheus/fetch-interface.l")

(ros::roseus "look")


(defclass fetch-head-look-at
  :super head-look-at
  :slots (wrench-topic))

(defmethod fetch-head-look-at
    (:init ()
     (send-super :init)
     (setq input-file (ros::get-param "~input_file" nil))
     (setq output-file (ros::get-param "~output_file" "output.yaml"))
     (if input-file
         (unix:system (format nil "rosparam load ~A look" input-file))
       (force-calibration output-file))
     (setq wrench-topic (ros::get-param "~wrench_topic"))

     (fetch-init nil)
     (ros::subscribe wrench-topic geometry_msgs::WrenchStamped #'send self :wrench-cb))

  (:look (x y)
         (when look-enable
           (let (pan tilt)
             (setq pan (* 90 (/ y (* 1.6 (cdr (assoc "max" force-th-y :test #'string=))))))
             (setq tilt 0)
             (if (> pan 0)
                 (setq pan (min pan 90))
               (setq pan (max pan -90)))
             (send *fetch* :head :angle-vector (float-vector pan tilt))
             (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 2000 :head-controller)))))

(setq *fetch-head-look-at* (instance fetch-head-look-at :init))
(ros::rate 30)
(do-until-key
    (ros::spin-once)
  (send *fetch-head-look-at* :run)
  (ros::sleep))





