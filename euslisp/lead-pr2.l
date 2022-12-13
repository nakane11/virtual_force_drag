#!/usr/bin/env roseus
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "virtual_force_drag")

(setq *status* nil)

(defun execute-cb (server goal)
  (setq switch (send goal :goal :switch))
  (when switch
    (unless *status*
      (ros::ros-info (format nil "start"))
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (send *pr2* :larm :angle-vector #f(44.3333 36.9321 9.03022 -121.342 -133.409 -92.8499 -19.1599))
      ;; (send *pr2* :head :angle-vector #f(77.0 23.0))
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation)
      (ros::service-call "lead_pr2/start" (instance std_srvs::EmptyRequest :init))))
  (unless switch
    (when *status*
      (ros::ros-info (format nil "stop"))
      (ros::service-call "lead_pr2/stop" (instance std_srvs::EmptyRequest :init))
      (pr2-tuckarm-pose)
      (send *ri* :wait-interpolation)
      ))
  (send server :set-succeeded (send server :result)))

(defun status-cb (msg)
  (setq *status* (send msg :data))
  (when *status*
    ;; (send *ri* :speak-jp "移動可能です" :volume 0.5))
    (send *ri* :speak-en "Movable." :volume 1))
  (unless *status*
    ;; (send *ri* :speak-jp "停止しました" :volume 0.5))
    (send *ri* :speak-en "Halted." :volume 1))
  (ros::ros-info (format nil "status:~A" *status*)))

(defun joy-cb (msg)
  (setq button (elt (send msg :buttons) 8))
  (when (> button 0)
    (when (> (send (ros::time- (ros::time-now) *last-time*) :to-sec) 1)
      (unless *status*
        (ros::ros-info (format nil "start"))
        (send *pr2* :larm :angle-vector #f(44.3333 36.9321 9.03022 -121.342 -133.409 -92.8499 -19.1599))
        ;; (send *pr2* :head :angle-vector #f(77.0 23.0))
        (send *ri* :angle-vector (send *pr2* :angle-vector))
        (send *ri* :wait-interpolation)
        (ros::service-call "lead_pr2/start" (instance std_srvs::EmptyRequest :init)))
      (when *status*
        (ros::ros-info (format nil "stop"))
        (ros::service-call "lead_pr2/stop" (instance std_srvs::EmptyRequest :init))))
    (setq *last-time* (ros::time-now))))

(ros::roseus "lead_pr2_interface")
(setq *last-time* (ros::time-now))
(pr2-init)
(ros::subscribe "/joy" sensor_msgs::Joy #'joy-cb)
(ros::subscribe "/lead_pr2/status" std_msgs::Bool #'status-cb)
(setq *as*
      (instance ros::simple-action-server :init
                "/lead_pr2_action" virtual_force_drag::SwitchAction
                :execute-cb #'execute-cb))

;; (send *ri* :angle-vector #f(22.4077 66.8381 49.9094 -124.795 128.259 -49.9694 46.1586 23.8993 0.050981 1.7321))
;; (send *ri* :wait-interpolation)

;; (sys::exit 0)
(do-until-key
 (send *as* :worker)
 (ros::spin-once)
 (ros::sleep))
(exit)
