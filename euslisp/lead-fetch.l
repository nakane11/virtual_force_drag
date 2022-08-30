#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(ros::roseus-add-srvs "std_srvs")

(ros::roseus "lead_fetch_interface")
(fetch-init)
(send *ri* :angle-vector #f(22.4077 66.8381 49.9094 -124.795 128.259 -49.9694 46.1586 23.8993 0.050981 1.7321))
(send *ri* :wait-interpolation)

;; (ros::wait-for-service "lead_fetch/start")
;; (ros::wait-for-service "lead_fetch/stop")
;; (ros::service-call "lead_fetch/start" (instance std_srvs::EmptyRequest :init))

(sys::exit 0)
