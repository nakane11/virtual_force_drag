#!/usr/bin/env roseus
(require :fetch-interface "package://fetcheus/fetch-interface.l")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "std_msgs")

(setq *status* nil)

(defun status-cb (msg)
  (setq *status* (send msg :data))
  (ros::ros-info (format nil "status:~A" *status*)))

(defun joy-cb (msg)
  (setq button (elt (send msg :buttons) 8))
  (when (> button 0)
    (when (> (send (ros::time- (ros::time-now) *last-time*) :to-sec) 1)
      (unless *status*
        (ros::ros-info (format nil "start"))
        (send *ri* :angle-vector #f(22.4077 66.8381 49.9094 -124.795 128.259 -49.9694 46.1586 23.8993 0.050981 1.7321))
        (send *ri* :wait-interpolation)
        (ros::service-call "lead_fetch/start" (instance std_srvs::EmptyRequest :init)))
      (when *status*
        (ros::ros-info (format nil "stop"))
        (ros::service-call "lead_fetch/stop" (instance std_srvs::EmptyRequest :init))))
    (setq *last-time* (ros::time-now))))

(ros::roseus "lead_fetch_interface")
(setq *last-time* (ros::time-now))
(fetch-init)
(ros::subscribe "/joy" sensor_msgs::Joy #'joy-cb)
(ros::subscribe "/lead_fetch/status" std_msgs::Bool #'status-cb)

;; (send *ri* :angle-vector #f(22.4077 66.8381 49.9094 -124.795 128.259 -49.9694 46.1586 23.8993 0.050981 1.7321))
;; (send *ri* :wait-interpolation)

;; (sys::exit 0)
(ros::spin)
(exit)
