(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "std_srvs")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "head_look_at")
(pr2-init nil)
(setq *look-enable* t)
  
(defclass look-in-force
  :slots (arm tmp-x tmp-y tmp-z l-force r-force))

(defmethod look-in-force
  (:init ()
   (ros::subscribe "/left_endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :left-cb)
   (ros::subscribe "/right_endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :right-cb)
   self)

  (:left-cb
   (msg)
   (setq l-force (send (send msg :wrench) :force)))

  (:right-cb
   (msg)
   (setq r-force (send (send msg :wrench) :force)))

  (:look-at (arm)
   (let ((pan (rad2deg (atan tmp-y tmp-x))) (tilt (+ 25 (* 2 tmp-z))))
     (if (> pan 0)
         (setq pan (min pan 168))
         (setq pan (max pan -168)))
     (if (> tilt 0)
         (setq tilt (min tilt 60))
         (setq tilt (max tilt -30)))
     (send *pr2* :head :angle-vector (float-vector pan tilt)))
 
  ;; (:look-at-hand
  ;;  (arm)
  ;;  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;  (setq end-pos (send *pr2* arm :end-coords :worldpos))
  ;;  (setq wrench-pos (float-vector (+ (elt end-pos 0) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2)))) tmp-x))
  ;;                                 (+ (elt end-pos 1) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2)))) tmp-y))
  ;;                                 1400))
  ;;  (send *pr2* :head :look-at wrench-pos)
   (send *ri* :angle-vector (send *pr2* :angle-vector) 3000 :head-controller))
       
  (:run ()
   (when *look-enable*
     ;; (when (ros::has-param "/target_arm")
     ;;   (setq arm (ros::get-param "/target_arm"))
     (setq arm "larm")

       (cond
         ((and (string-equal arm "larm") (not (null l-force)))
          (progn
            (setq tmp-x (send l-force :x))
            (setq tmp-y (send l-force :y))
            (setq tmp-z (send l-force :z))))

         ((and (string-equal arm "rarm") (not (null r-force)))
          (progn
            (setq tmp-x (send r-force :x))
            (setq tmp-y (send r-force :y))
            (setq tmp-z (send r-force :z))))
         (t (return-from :run nil)))

       (if (string-equal arm "larm")
           (send self :look-at :larm)
           (send self :look-at :rarm)))))

(defun start-look-at (req)
  (setq *look-enable* t)
  (instance std_srvs::EmptyResponse :init))

(defun stop-look-at (req)
  (setq *look-enable* nil)
  (instance std_srvs::EmptyResponse :init))

(setq *l* (instance look-in-force :init))
(ros::advertise-service "~start" std_srvs::Empty #'start-look-at)
(ros::advertise-service "~stop" std_srvs::Empty #'stop-look-at)

(ros::rate 5)
(do-until-key
 (ros::spin-once)
 (send *l* :run)
 (ros::sleep))
