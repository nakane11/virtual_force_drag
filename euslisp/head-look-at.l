(ros::roseus-add-msgs "geometry_msgs")

(defclass head-look-at
     :slots (force look-enable
                   force-th-x force-th-y force-th-z))

(defmethod head-look-at
    (:init ()
     (setq force-th-x (ros::get-param "/look/force_th_x"))
     (setq force-th-y (ros::get-param "/look/force_th_y"))
     (setq force-th-z (ros::get-param "/look/force_th_z"))
     (setq look-enable t)

     (ros::advertise-service "~start" std_srvs::Empty #'send self :start-look-at)
     (ros::advertise-service "~stop" std_srvs::Empty #'send self :stop-look-at))

  (:wrench-cb
   (msg)
   (setq force (send (send msg :wrench) :force)))

  (:start-look-at (req)
                  (setq look-enable t)
                  (instance std_srvs::EmptyResponse :init))

  (:stop-look-at (req)
                 (setq look-enable nil)
                 (instance std_srvs::EmptyResponse :init))

  (:look (x y)
         nil)
  
  (:run ()
        (let (tmp-x tmp-y tmp-z)
          (when force
            (setq tmp-x (- (send force :x) (cdr (assoc "zero" force-th-x :test #'string=))))
            (setq tmp-y (- (send force :y) (cdr (assoc "zero" force-th-y :test #'string=))))
            (setq tmp-z (- (send force :z) (cdr (assoc "zero" force-th-z :test #'string=))))
            
            (send self :look tmp-x tmp-y)))))
