; Auto-generated. Do not edit!


(cl:in-package arm-srv)


;//! \htmlinclude dynamixel_srv-request.msg.html

(cl:defclass <dynamixel_srv-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass dynamixel_srv-request (<dynamixel_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <dynamixel_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'dynamixel_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm-srv:<dynamixel_srv-request> is deprecated: use arm-srv:dynamixel_srv-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <dynamixel_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm-srv:data-val is deprecated.  Use arm-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <dynamixel_srv-request>) ostream)
  "Serializes a message object of type '<dynamixel_srv-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <dynamixel_srv-request>) istream)
  "Deserializes a message object of type '<dynamixel_srv-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<dynamixel_srv-request>)))
  "Returns string type for a service object of type '<dynamixel_srv-request>"
  "arm/dynamixel_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'dynamixel_srv-request)))
  "Returns string type for a service object of type 'dynamixel_srv-request"
  "arm/dynamixel_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<dynamixel_srv-request>)))
  "Returns md5sum for a message object of type '<dynamixel_srv-request>"
  "e914a2ad5395f6f66880d5aed68f5700")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'dynamixel_srv-request)))
  "Returns md5sum for a message object of type 'dynamixel_srv-request"
  "e914a2ad5395f6f66880d5aed68f5700")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<dynamixel_srv-request>)))
  "Returns full string definition for message of type '<dynamixel_srv-request>"
  (cl:format cl:nil "float64 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'dynamixel_srv-request)))
  "Returns full string definition for message of type 'dynamixel_srv-request"
  (cl:format cl:nil "float64 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <dynamixel_srv-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <dynamixel_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'dynamixel_srv-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude dynamixel_srv-response.msg.html

(cl:defclass <dynamixel_srv-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass dynamixel_srv-response (<dynamixel_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <dynamixel_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'dynamixel_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm-srv:<dynamixel_srv-response> is deprecated: use arm-srv:dynamixel_srv-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <dynamixel_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm-srv:result-val is deprecated.  Use arm-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <dynamixel_srv-response>) ostream)
  "Serializes a message object of type '<dynamixel_srv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <dynamixel_srv-response>) istream)
  "Deserializes a message object of type '<dynamixel_srv-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<dynamixel_srv-response>)))
  "Returns string type for a service object of type '<dynamixel_srv-response>"
  "arm/dynamixel_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'dynamixel_srv-response)))
  "Returns string type for a service object of type 'dynamixel_srv-response"
  "arm/dynamixel_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<dynamixel_srv-response>)))
  "Returns md5sum for a message object of type '<dynamixel_srv-response>"
  "e914a2ad5395f6f66880d5aed68f5700")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'dynamixel_srv-response)))
  "Returns md5sum for a message object of type 'dynamixel_srv-response"
  "e914a2ad5395f6f66880d5aed68f5700")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<dynamixel_srv-response>)))
  "Returns full string definition for message of type '<dynamixel_srv-response>"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'dynamixel_srv-response)))
  "Returns full string definition for message of type 'dynamixel_srv-response"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <dynamixel_srv-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <dynamixel_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'dynamixel_srv-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'dynamixel_srv)))
  'dynamixel_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'dynamixel_srv)))
  'dynamixel_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'dynamixel_srv)))
  "Returns string type for a service object of type '<dynamixel_srv>"
  "arm/dynamixel_srv")