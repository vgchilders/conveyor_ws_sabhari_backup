;; Auto-generated. Do not edit!


(when (boundp 'arm::Float_srv)
  (if (not (find-package "ARM"))
    (make-package "ARM"))
  (shadow 'Float_srv (find-package "ARM")))
(unless (find-package "ARM::FLOAT_SRV")
  (make-package "ARM::FLOAT_SRV"))
(unless (find-package "ARM::FLOAT_SRVREQUEST")
  (make-package "ARM::FLOAT_SRVREQUEST"))
(unless (find-package "ARM::FLOAT_SRVRESPONSE")
  (make-package "ARM::FLOAT_SRVRESPONSE"))

(in-package "ROS")





(defclass arm::Float_srvRequest
  :super ros::object
  :slots (_data ))

(defmethod arm::Float_srvRequest
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float64 _data
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _data
       (sys::poke _data (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _data
     (setq _data (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass arm::Float_srvResponse
  :super ros::object
  :slots (_result ))

(defmethod arm::Float_srvResponse
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

(defclass arm::Float_srv
  :super ros::object
  :slots ())

(setf (get arm::Float_srv :md5sum-) "e914a2ad5395f6f66880d5aed68f5700")
(setf (get arm::Float_srv :datatype-) "arm/Float_srv")
(setf (get arm::Float_srv :request) arm::Float_srvRequest)
(setf (get arm::Float_srv :response) arm::Float_srvResponse)

(defmethod arm::Float_srvRequest
  (:response () (instance arm::Float_srvResponse :init)))

(setf (get arm::Float_srvRequest :md5sum-) "e914a2ad5395f6f66880d5aed68f5700")
(setf (get arm::Float_srvRequest :datatype-) "arm/Float_srvRequest")
(setf (get arm::Float_srvRequest :definition-)
      "float64 data
---
bool result
")

(setf (get arm::Float_srvResponse :md5sum-) "e914a2ad5395f6f66880d5aed68f5700")
(setf (get arm::Float_srvResponse :datatype-) "arm/Float_srvResponse")
(setf (get arm::Float_srvResponse :definition-)
      "float64 data
---
bool result
")



(provide :arm/Float_srv "e914a2ad5395f6f66880d5aed68f5700")


