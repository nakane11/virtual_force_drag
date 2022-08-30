(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "pull_arm")

(defclass pull-arm
    :slots (input-file output-file force
                       vel-topic wrench-topic
                       force-th-x force-th-y force-th-z
                       vel-th-x vel-th-y
                       continuity-x continuity-y
                       func-max-x func-min-x func-max-y func-min-y
                       x-list y-list sign
                       x-acc y-acc))

(defmethod pull-arm
    (:init ()
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
     (setq continuity-x (ros::get-param "/pull_arm/continuity_x"))
     (setq continuity-y (ros::get-param "/pull_arm/continuity_y"))

     (load "package://fetcheus/fetch-interface.l")
     (fetch-init nil)
     (setq sign 1)
     (send *fetch* :angle-vector #f(22.4077 66.8381 49.9094 -124.795 128.259 -49.9694 46.1586 23.8993 0.050981 1.7321))

     (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
     (send *ri* :wait-interpolation)
    
     (setq x-list (list 0))
     (setq y-list (list 0))

     (setq x-acc 0)
     (setq y-acc 0)
     (setq state 
     self)

  (:wrench-cb
   (msg)
   (setq force (send (send msg :wrench) :force)))

  (:check (li force-th)
          (block max
            (dolist (i li)
              (unless (> i (cdr (assoc "max" force-th :test #'string=)))
                (return-from max)))
            (return-from :check :max))
          (block min
            (dolist (i li)
              (unless (< i (cdr (assoc "min" force-th :test #'string=)))
                (return-from min)))
            (return-from :check :min))
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
            (setq tmp-x (- (send force :x) (cdr (assoc "zero" force-th-x :test #'string=))))
            (setq tmp-y (* sign (- (send force :y) (cdr (assoc "zero" force-th-y :test #'string=)))))
            (setq tmp-z (- (send force :z) (cdr (assoc "zero" force-th-z :test #'string=))))

            (when (> (length x-list) continuity-x)
              (setq x-list (cdr x-list)))
            (setq x-list (append x-list (list tmp-x)))

            (when (> (length y-list) continuity-y)
              (setq y-list (cdr y-list)))
            (setq y-list (append y-list (list tmp-y)))
            
            (let ((vel-x 0.0) (vel-y 0.0))
              (case (send self :check y-list force-th-y)
                (:max
                 (setq vel-y (funcall func-max-y tmp-y)))
                (:min
                 (setq vel-y (- (funcall func-min-y tmp-y)))))

              (case (send self :check x-list force-th-x)
                (:max
                 (setq vel-x (funcall func-max-x tmp-x)))
                (:min
                 (setq vel-x (- (funcall func-min-x tmp-x)))))
              (when (or (not (eq vel-x 0.0)) (not (eq vel-y 0.0)))
                (when (> (abs vel-y) 0)
                    (setq vel-x 0))
                (send *ri* :send-cmd-vel-raw vel-x vel-y :topic-name vel-topic)))))))

(setq *pa* (instance pull-arm :init))
(ros::rate 50)
(do-until-key
 (ros::spin-once)
 (send *pa* :run)
 (ros::sleep))
