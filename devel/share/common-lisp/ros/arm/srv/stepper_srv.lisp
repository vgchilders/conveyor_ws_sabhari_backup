; Auto-generated. Do not edit!


(cl:in-package arm-srv)


;//! \htmlinclude stepper_srv-request.msg.html

(cl:defclass <stepper_srv-request> (roslisp-msg-protocol:ros-message)
  ((data1
    :reader data1
    :initarg :data1
    :type cl:float
    :initform 0.0)
   (data2
    :reader data2
    :initarg :data2
    :type cl:float
    :initform 0.0))
)

(cl:defclass stepper_srv-request (<stepper_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <stepper_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'stepper_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm-srv:<stepper_srv-request> is deprecated: use arm-srv:stepper_srv-request instead.")))

(cl:ensure-generic-function 'data1-val :lambda-list '(m))
(cl:defmethod data1-val ((m <stepper_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm-srv:data1-val is deprecated.  Use arm-srv:data1 instead.")
  (data1 m))

(cl:ensure-generic-function 'data2-val :lambda-list '(m))
(cl:defmethod data2-val ((m <stepper_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm-srv:data2-val is deprecated.  Use arm-srv:data2 instead.")
  (data2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <stepper_srv-request>) ostream)
  "Serializes a message object of type '<stepper_srv-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <stepper_srv-request>) istream)
  "Deserializes a message object of type '<stepper_srv-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data2) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<stepper_srv-request>)))
  "Returns string type for a service object of type '<stepper_srv-request>"
  "arm/stepper_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'stepper_srv-request)))
  "Returns string type for a service object of type 'stepper_srv-request"
  "arm/stepper_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<stepper_srv-request>)))
  "Returns md5sum for a message object of type '<stepper_srv-request>"
  "1a060548b1bc718fe8b44339268f4d29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'stepper_srv-request)))
  "Returns md5sum for a message object of type 'stepper_srv-request"
  "1a060548b1bc718fe8b44339268f4d29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<stepper_srv-request>)))
  "Returns full string definition for message of type '<stepper_srv-request>"
  (cl:format cl:nil "float64 data1~%float64 data2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'stepper_srv-request)))
  "Returns full string definition for message of type 'stepper_srv-request"
  (cl:format cl:nil "float64 data1~%float64 data2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <stepper_srv-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <stepper_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'stepper_srv-request
    (cl:cons ':data1 (data1 msg))
    (cl:cons ':data2 (data2 msg))
))
;//! \htmlinclude stepper_srv-response.msg.html

(cl:defclass <stepper_srv-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass stepper_srv-response (<stepper_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <stepper_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'stepper_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm-srv:<stepper_srv-response> is deprecated: use arm-srv:stepper_srv-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <stepper_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm-srv:result-val is deprecated.  Use arm-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <stepper_srv-response>) ostream)
  "Serializes a message object of type '<stepper_srv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <stepper_srv-response>) istream)
  "Deserializes a message object of type '<stepper_srv-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<stepper_srv-response>)))
  "Returns string type for a service object of type '<stepper_srv-response>"
  "arm/stepper_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'stepper_srv-response)))
  "Returns string type for a service object of type 'stepper_srv-response"
  "arm/stepper_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<stepper_srv-response>)))
  "Returns md5sum for a message object of type '<stepper_srv-response>"
  "1a060548b1bc718fe8b44339268f4d29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'stepper_srv-response)))
  "Returns md5sum for a message object of type 'stepper_srv-response"
  "1a060548b1bc718fe8b44339268f4d29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<stepper_srv-response>)))
  "Returns full string definition for message of type '<stepper_srv-response>"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'stepper_srv-response)))
  "Returns full string definition for message of type 'stepper_srv-response"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <stepper_srv-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <stepper_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'stepper_srv-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'stepper_srv)))
  'stepper_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'stepper_srv)))
  'stepper_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'stepper_srv)))
  "Returns string type for a service object of type '<stepper_srv>"
  "arm/stepper_srv")