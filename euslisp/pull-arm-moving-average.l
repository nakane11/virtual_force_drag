(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "pull_arm")

(defclass pull-arm
    :slots (input-file output-file robot-type target-arm force shoulder-pan
                       vel-topic wrench-topic
                       force-th-x force-th-y force-th-z
                       vel-th-x vel-th-y
                       func-max-x func-min-x func-max-y func-min-y
                       sign))

(defmethod pull-arm
    (:init ()
     (setq robot-type (ros::get-param "~robot"))
     (setq target-arm (ros::get-param "~target_arm"))
     (setq vel-topic (ros::get-param "~vel_topic" "/input_vel"))
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
     (setq vel-th-x (ros::get-param "/pull_arm/vel_th_x"))
     (setq vel-th-y (ros::get-param "/pull_arm/vel_th_y"))

     (cond
       ((string= robot-type "pr2")
        
        (load "package://pr2eus/pr2-interface.l")
        (pr2-init nil)
        (setq *robot* *pr2*)
        (if (string= target-arm "larm")
            (progn
              (setq sign 1)
              (send *robot* :larm :angle-vector #f(44.3333 36.9321 9.03022 -121.342 -133.409 -92.8499 -19.1599))
              (send *robot* :head :angle-vector #f(77.0 23.0)))
            (progn
              (setq sign -1)
              (send *robot* :rarm :angle-vector #f(-44.3333 36.9321 9.03022 -121.342 133.409 -92.8499 -19.1599))
              (send *robot* :head :angle-vector #f(-77.0 23.0)))))
     
       ((string= robot-type "fetch")

        (load "package://fetcheus/fetch-interface.l")
        (fetch-init nil)
        (setq *robot* *fetch*)
        (setq sign 1)
        (send *robot* :angle-vector #f(22.4077 66.8381 49.9094 -124.795 128.259 -49.9694 46.1586 23.8993 0.050981 1.7321))))
  
     (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
     (send *ri* :wait-interpolation)
     
     self)

  (:wrench-cb
   (msg)
   (setq force (send (send msg :wrench) :force)))

  (:check (tmp force-th)
          (block max
                 (when (> tmp (cdr (assoc "max" force-th :test #'string=)))
                   (return-from :check :max)))
          (block min
                 (when (< tmp (cdr (assoc "min" force-th :test #'string=)))
                   (return-from :check :min)))
          nil)

  (:rad-to-vel (rad)
               (* 0.4 (tan (/ (- rad (/ pi 4)) 1.1))))

  (:run ()
        (let (tmp-x tmp-y tmp-z)

          (setq func-max-x #'(lambda (x) (/ (log (abs x)) (cdr (assoc "max" vel-th-x :test #'string=)))))
          (setq func-min-x #'(lambda (x) (/ (log (abs x)) (cdr (assoc "min" vel-th-x :test #'string=)))))
          (setq func-max-y #'(lambda (y) (/ (log (abs y)) (cdr (assoc "max" vel-th-y :test #'string=)))))
          (setq func-min-y #'(lambda (y) (/ (log (abs y)) (cdr (assoc "min" vel-th-y :test #'string=)))))

          (when force
            (when (string= robot-type "pr2")
              (send *ri* :state :potentio-vector)
              (when (string= target-arm "larm")
                (setq shoulder-pan (deg2rad (send *ri* :robot :l_shoulder_pan_joint :joint-angle)))
                (when (or (< shoulder-pan 0.2) (> shoulder-pan 1.37))
                  (setq rotate (send self :rad-to-vel shoulder-pan))
                  (send *ri* :send-cmd-vel-raw 0 0 rotate :topic-name vel-topic)
                  (return-from :run)))
              (when (string= target-arm "rarm")
                (setq shoulder-pan (* sign (deg2rad (send *ri* :robot :r_shoulder_pan_joint :joint-angle))))
                (when (or (< shoulder-pan 0.2) (> shoulder-pan 1.37))
                  (setq rotate (send self :rad-to-vel shoulder-pan))
                  (send *ri* :send-cmd-vel-raw 0 0 (- rotate) :topic-name vel-topic)
                  (return-from :run))))

            (setq tmp-x (- (send force :x) (cdr (assoc "zero" force-th-x :test #'string=))))
            (setq tmp-y (* sign (- (send force :y) (cdr (assoc "zero" force-th-y :test #'string=)))))
            (setq tmp-z (- (send force :z) (cdr (assoc "zero" force-th-z :test #'string=))))
            
            (let ((vel-x 0.0) (vel-y 0.0))
              (case (send self :check tmp-y force-th-y)
                (:max
                 (setq vel-y (funcall func-max-y tmp-y)))
                (:min
                 (setq vel-y (- (funcall func-min-y tmp-y)))))

              (case (send self :check tmp-x force-th-x)
                (:max
                 (setq vel-x (funcall func-max-x tmp-x)))
                (:min
                 (setq vel-x (- (funcall func-min-x tmp-x)))))
              (when (or (not (eq vel-x 0.0)) (not (eq vel-y 0.0)))
                (when (> (abs vel-y) 0)
                    (setq vel-x 0))
                (cond
                  ((string= robot-type "pr2")
                   (send *ri* :send-cmd-vel-raw vel-x (* sign vel-y) 0 :topic-name vel-topic)
                   )
                  ((string= robot-type "fetch")
                   (send *ri* :send-cmd-vel-raw vel-x vel-y :topic-name vel-topic))))
              )))))


(setq *pa* (instance pull-arm :init))
(ros::rate 50)
(do-until-key
 (ros::spin-once)
 (send *pa* :run)
 (ros::sleep))
