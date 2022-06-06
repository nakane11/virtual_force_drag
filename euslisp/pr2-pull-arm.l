(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(load "package://pr2eus/pr2-interface.l")
(load "package://hand_navigation_pr2/euslisp/give-hand-pose.l")

(ros::roseus "pull_arm")
(pr2-init nil)
  
(defclass pull-arm
  :slots (tmp-x tmp-y tmp-z x-list y-list l-force r-force l-shoulder-pan r-shoulder-pan))

(defmethod pull-arm
    (:init ()
     (when (ros::has-param "/target_arm")
       (ros::delete-param "/target_arm"))
     (setq x-list (list 0))
     (setq y-list (list 0))
     (pr2-reset-pose)
     (let (hand-coords)
       (send *pr2* :head :angle-vector #f(0.0 14.694))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
       (setq l nil)
       (while (null l)
         (setq hand-coords nil)
         (while (null hand-coords)
           (ros::spin-once)
           (ros::ros-info "wait for hand...")
           (setq hand-coords (hand-pose-subscriber)))
         (setq l (raise-hand hand-coords))
         (ros::ros-info "hand found")))

     (ros::subscribe "/left_endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :left-cb)
     (ros::subscribe "/right_endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :right-cb)
     (ros::subscribe "/joint_states" sensor_msgs::Jointstate #'send self :joint-cb)
     self)

  (:left-cb
   (msg)
   (setq l-force (send (send msg :wrench) :force)))

  (:right-cb
   (msg)
   (setq r-force (send (send msg :wrench) :force)))

  (:joint-cb
   (msg)
   (setq l-shoulder-pan (elt (send msg :position) 32))
   (setq r-shoulder-pan (elt (send msg :position) 18)))

  (:check (axis)
   ()
   (when (eq axis :x)
     (dolist (i x-list)
       (unless (> (abs i) 1.5)
         (return-from :check nil))))
   (when (eq axis :y)
     (dolist (i y-list)
       (unless (> (abs i) 0.8)
         (return-from :check nil))))
   t)

  (:rad-to-vel (rad)
               (* 0.35 (tan (/ (- rad (/ pi 4)) 1.1))))

  (:run ()
        (let (arm tmp-x tmp-y tmp-z)
          (when (ros::has-param "/target_arm")
            (setq arm (ros::get-param "/target_arm"))

            (cond
              ((and (string-equal arm "larm") (not (null l-force)) (not (null l-shoulder-pan)))
               (progn
                 (if (or (< l-shoulder-pan 0.2) (> l-shoulder-pan 1.37))
                     (setq rotate (send self :rad-to-vel l-shoulder-pan))
                     (setq rotate nil))
                 (setq tmp-x (send l-force :x))
                 (setq tmp-y (send l-force :y))
                 (setq tmp-z (send l-force :z))))

              ((and (string-equal arm "rarm") (not (null r-force)) (not (null r-shoulder-pan)))
               (progn
                 (if (or (< r-shoulder-pan -1.37) (> r-shoulder-pan -0.2))
                     (setq rotate (send self :rad-to-vel r-shoulder-pan))
                     (setq rotate nil))
                 (setq tmp-x (send r-force :x))
                 (setq tmp-y (send r-force :y))
                 (setq tmp-z (send r-force :z))))
              (t (return-from :run nil)))

            ;; (ros::ros-info "x: ~A  y: ~A z: ~A" tmp-x tmp-y tmp-z)

            (block send-cmd-vel
              (unless (null rotate)
                (send *ri* :send-cmd-vel-raw 0 0 rotate)
                (return-from send-cmd-vel))

              (let ((vel-x 0.0) (vel-y 0.0))
                (when (send self :check :y)
                  (when (> tmp-y 2.6)
                    (setq vel-y (/ (log tmp-y) 4.0)))
                  (when (< tmp-y -2.7)
                    (setq vel-y (- (/ (log (abs tmp-y)) 4.0)))))

                (when (send self :check :x)
                  (when (> tmp-x 2.2)
                    (setq vel-x (/ (log tmp-x) 3.0)))
                  (when (< tmp-x -2.5)
                    (setq vel-x (- (/ (log (abs tmp-x)) 3.0)))))

                (when (or (not (eq vel-x 0.0)) (not (eq vel-y 0.0)))
                  (send *ri* :send-cmd-vel-raw vel-x vel-y 0)))
              ))

        (when (> (length x-list) 10)
          (setq x-list (cdr x-list)))
        (setq x-list (append x-list (list tmp-x)))

        (when (> (length y-list) 13)
          (setq y-list (cdr y-list)))
        (setq y-list (append y-list (list tmp-y))))))



(setq *pa* (instance pull-arm :init))
(ros::rate 50)
(do-until-key
 (ros::spin-once)
 (send *pa* :run)
 (ros::sleep))
