(defun sum (l)
  (if (null l)
      0
    (+ (car l) (sum (cdr l)))))

(defclass moving-average
  :slots (q len))

(defmethod moving-average
  (:init (l)
         (setq q (instance queue :init))
         (setq len l))

  (:update (a)
           (send q :enqueue a)
           (when
               (< (send q :length) len)
             (return-from :update nil))
           (when (> (send q :length) len)
             (send q :dequeue))
           (/ (sum (car q)) (float len)))

  (:queue ()
          (car q)))
          
