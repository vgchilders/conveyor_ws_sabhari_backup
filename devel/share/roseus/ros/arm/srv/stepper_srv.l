;; Auto-generated. Do not edit!


(when (boundp 'arm::stepper_srv)
  (if (not (find-package "ARM"))
    (make-package "ARM"))
  (shadow 'stepper_srv (find-package "ARM")))
(unless (find-package "ARM::STEPPER_SRV")
  (make-package "ARM::STEPPER_SRV"))
(unless (find-package "ARM::STEPPER_SRVREQUEST")
  (make-package "ARM::STEPPER_SRVREQUEST"))
(unless (find-package "ARM::STEPPER_SRVRESPONSE")
  (make-package "ARM::STEPPER_SRVRESPONSE"))

(in-package "ROS")





(defclass arm::stepper_srvRequest
  :super ros::object
  :slots (_data1 _data2 ))

(defmethod arm::stepper_srvRequest
  (:init
   (&key
    ((:data1 __data1) 0.0)
    ((:data2 __data2) 0.0)
    )
   (send-super :init)
   (setq _data1 (float __data1))
   (setq _data2 (float __data2))
   self)
  (:data1
   (&optional __data1)
   (if __data1 (setq _data1 __data1)) _data1)
  (:data2
   (&optional __data2)
   (if __data2 (setq _data2 __data2)) _data2)
  (:serialization-length
   ()
   (+
    ;; float64 _data1
    8
    ;; float64 _data2
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _data1
       (sys::poke _data1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _data2
       (sys::poke _data2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _data1
     (setq _data1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _data2
     (setq _data2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass arm::stepper_srvResponse
  :super ros::object
  :slots (_result ))

(defmethod arm::stepper_srvResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional (__result :null))
   (if (not (eq __result :null)) (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass arm::stepper_srv
  :super ros::object
  :slots ())

(setf (get arm::stepper_srv :md5sum-) "1a060548b1bc718fe8b44339268f4d29")
(setf (get arm::stepper_srv :datatype-) "arm/stepper_srv")
(setf (get arm::stepper_srv :request) arm::stepper_srvRequest)
(setf (get arm::stepper_srv :response) arm::stepper_srvResponse)

(defmethod arm::stepper_srvRequest
  (:response () (instance arm::stepper_srvResponse :init)))

(setf (get arm::stepper_srvRequest :md5sum-) "1a060548b1bc718fe8b44339268f4d29")
(setf (get arm::stepper_srvRequest :datatype-) "arm/stepper_srvRequest")
(setf (get arm::stepper_srvRequest :definition-)
      "float64 data1
float64 data2
---
bool result
")

(setf (get arm::stepper_srvResponse :md5sum-) "1a060548b1bc718fe8b44339268f4d29")
(setf (get arm::stepper_srvResponse :datatype-) "arm/stepper_srvResponse")
(setf (get arm::stepper_srvResponse :definition-)
      "float64 data1
float64 data2
---
bool result
")



(provide :arm/stepper_srv "1a060548b1bc718fe8b44339268f4d29")


