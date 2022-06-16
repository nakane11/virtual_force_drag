(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "look")
(load "package://virtual_force_drag/euslisp/head-look-at.l")

(setq *look-enable* t)

(defclass pull-arm
    :slots (input-file output-file robot-type target-arm force
                       wrench-topic
                       force-th-x force-th-y force-th-z
                       sign))

(defmethod pull-arm
    (:init ()
     (setq robot-type (ros::get-param "~robot"))
     (setq target-arm (ros::get-param "~target_arm"))
     (setq wrench-topic (ros::get-param "~wrench_topic"))
     (setq input-file (ros::get-param "~input_file" nil))
     (setq output-file (ros::get-param "~output_file" "output.yaml"))
     (if input-file
         (unix:system (format nil "rosparam load ~A pull_arm" input-file))
         (force-calibration output-file))

     (ros::subscribe wrench-topic geometry_msgs::WrenchStamped #'send self :wrench-cb)
     (setq force-th-x (ros::get-param "/pull_arm/force_th_x"))
     (setq force-th-y (ros::get-param "/pull_arm/force_th_y"))
     (setq force-th-z (ros::get-param "/pull_arm/force_th_z"))

     (cond
       ((string= robot-type "pr2")
        (if (string= target-arm "larm")
            (setq sign 1)
            (setq sign -1))
        (load "package://pr2eus/pr2-interface.l")
        (pr2-init nil)
        (setq *robot* *pr2*))
       
       ((string= robot-type "fetch")
        (setq sign 1)
        (load "package://fetcheus/fetch-interface.l")
        (fetch-init nil)
        (setq *robot* *fetch*)))
     self)

  (:wrench-cb
   (msg)
   (setq force (send (send msg :wrench) :force)))

  (:run ()
        (let (tmp-x tmp-y tmp-z)
          (when force
            (setq tmp-x (- (send force :x) (cdr (assoc "zero" force-th-x :test #'string=))))
            (setq tmp-y (* sign (- (send force :y) (cdr (assoc "zero" force-th-y :test #'string=)))))
            (setq tmp-z (- (send force :z) (cdr (assoc "zero" force-th-z :test #'string=))))
            
            (head-look-at robot-type tmp-x (* sign tmp-y))))))

(defun start-look-at (req)
  (setq *look-enable* t)
  (instance std_srvs::EmptyResponse :init))

(defun stop-look-at (req)
  (setq *look-enable* nil)
  (instance std_srvs::EmptyResponse :init))

(ros::advertise-service "~start" std_srvs::Empty #'start-look-at)
(ros::advertise-service "~stop" std_srvs::Empty #'stop-look-at)

(setq *pa* (instance pull-arm :init))
(ros::rate 50)
(do-until-key
    (ros::spin-once)
  (send *pa* :run)
  (ros::sleep))
