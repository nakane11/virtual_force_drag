(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(load "package://fetcheus/fetch-interface.l")

(defun wrenchstamped-to-list (wrench)
  (let ((force (send (send wrench :wrench) :force)))
    (list (send force :x) (send force :y)  (send force :z))))

(defun force-calibration (threshold &key timeout)
  (let ((std (list 0 0 0)) tmp wrench (count 0))
    (while t
      (setq tmp (wrenchstamped-to-list (one-shot-subscribe "/endeffector/wrench_transformed" geometry_msgs::WrenchStamped :unsubscribe nil)))
      (block check
        (dotimes (i 3)
          (when (> (abs (- (elt tmp i) (elt std i))) threshold)
            (return-from check nil)))
        (ros::unsubscribe "/endeffector/wrench_transformed")
        (return-from force-calibration std))
      (setq std (list (/ (+ (elt std 0) (elt tmp 0)) 2)
                      (/ (+ (elt std 1) (elt tmp 1)) 2)
                      (/ (+ (elt std 2) (elt tmp 2)) 2)))
      (when timeout
        (if (> count timeout)
            (return-from force-calibration std)
            (incf count))))))
              
(defclass pull-arm
  :slots (std force x-list y-list tmp-x tmp-y move-x move-y))

(defmethod pull-arm
    (:init ()
     (setq std (force-calibration 5.0 :timeout 100))
     (ros::ros-info "std: ~A" std)
     (send *ri* :speak-jp "キャリブレーションが終了しました")
     (ros::subscribe "/endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :wrench-cb)
     (setq x-list (list 0))
     (setq y-list (list 0)))

  (:wrench-cb
   (msg)
   (setq force (send (send msg :wrench) :force)))

  (:check (axis)
   ()
   (when (eq axis :x)
     (dolist (i x-list)
       (unless (> (abs i) 10)
         (return-from :check nil))))
   (when (eq axis :y)
     (dolist (i y-list)
       (unless (> (abs i) 8)
         (return-from :check nil))))
   t)

  (:look-at-hand()
   (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
   (setq end-pos (send *fetch* :rarm :end-coords :worldpos))
   (setq wrench-pos (float-vector (+ (elt end-pos 0) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2)))) tmp-x))
                                  (+ (elt end-pos 1) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2)))) tmp-y))
                                  1400))                                  
   (send *fetch* :head :look-at wrench-pos :revert-if-fail nil)
   (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 900 :head-controller))
  
  (:run ()
        (unless (null force)
          (setq tmp-x (- (send force :x) (elt std 0)))
          (setq tmp-y (- (send force :y) (elt std 1)))
          (setq tmp-z (- (send force :z) (elt std 2)))
          ;; (ros::ros-info "x: ~A  y: ~A z: ~A" tmp-x tmp-y tmp-z)

          (send self :look-at-hand)
          (block move
            (when (send self :check :y)
              (when (> tmp-y 10)
                (send *ri* :send-cmd-vel-raw 0 (/ (log tmp-y) 6.0) :topic-name "/base_controller/cmd_vel"))
              (when (< tmp-y -10)
                (send *ri* :send-cmd-vel-raw 0 (- (/ (log (abs tmp-y)) 6.0)) :topic-name "/base_controller/cmd_vel"))
              (return-from move nil))

            (when (send self :check :x)
              (when (> tmp-x 16)
                (send *ri* :send-cmd-vel-raw (/ (log tmp-x) 6.50) 0 :topic-name "/base_controller/cmd_vel"))
              (when (< tmp-x -18)
                (send *ri* :send-cmd-vel-raw (- (/ (log (abs tmp-x)) 10.0)) 0 :topic-name "/base_controller/cmd_vel"))))

          (when (> (length x-list) 18)
            (setq x-list (cdr x-list)))
          (setq x-list (append x-list (list tmp-x)))

          (when (> (length y-list) 10)
            (setq y-list (cdr y-list)))
          (setq y-list (append y-list (list tmp-y))))))

(ros::roseus "pull_arm")
(fetch-init nil)
;; (send *ri* :angle-vector #f(22.4077 66.8381 49.9094 -124.795 128.259 -49.9694 46.1586 23.8993 0.050981 1.7321))
(send *ri* :angle-vector-raw #f(22.4077 66.8381 49.9094 -124.795 128.259 -49.9694 46.1586 23.8993 0.050981 1.7321) 900 :arm-controller)
(send *ri* :wait-interpolation)
(unix::sleep 1)

(setq *pa* (instance pull-arm :init))
(ros::rate 60)
(do-until-key
  (ros::spin-once)
  (send *pa* :run)
  (ros::sleep))
         
   
