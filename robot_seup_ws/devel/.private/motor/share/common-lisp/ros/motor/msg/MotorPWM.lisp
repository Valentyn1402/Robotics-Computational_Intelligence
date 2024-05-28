; Auto-generated. Do not edit!


(cl:in-package motor-msg)


;//! \htmlinclude MotorPWM.msg.html

(cl:defclass <MotorPWM> (roslisp-msg-protocol:ros-message)
  ((pwm_left
    :reader pwm_left
    :initarg :pwm_left
    :type cl:float
    :initform 0.0)
   (pwm_right
    :reader pwm_right
    :initarg :pwm_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass MotorPWM (<MotorPWM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorPWM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorPWM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor-msg:<MotorPWM> is deprecated: use motor-msg:MotorPWM instead.")))

(cl:ensure-generic-function 'pwm_left-val :lambda-list '(m))
(cl:defmethod pwm_left-val ((m <MotorPWM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor-msg:pwm_left-val is deprecated.  Use motor-msg:pwm_left instead.")
  (pwm_left m))

(cl:ensure-generic-function 'pwm_right-val :lambda-list '(m))
(cl:defmethod pwm_right-val ((m <MotorPWM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor-msg:pwm_right-val is deprecated.  Use motor-msg:pwm_right instead.")
  (pwm_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorPWM>) ostream)
  "Serializes a message object of type '<MotorPWM>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pwm_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pwm_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorPWM>) istream)
  "Deserializes a message object of type '<MotorPWM>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pwm_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pwm_right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorPWM>)))
  "Returns string type for a message object of type '<MotorPWM>"
  "motor/MotorPWM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorPWM)))
  "Returns string type for a message object of type 'MotorPWM"
  "motor/MotorPWM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorPWM>)))
  "Returns md5sum for a message object of type '<MotorPWM>"
  "d7a5f6b78fec2b5366e39d282d33bb64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorPWM)))
  "Returns md5sum for a message object of type 'MotorPWM"
  "d7a5f6b78fec2b5366e39d282d33bb64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorPWM>)))
  "Returns full string definition for message of type '<MotorPWM>"
  (cl:format cl:nil "# Represents the PWM value of the left and right motors 0...1~%float32 pwm_left~%float32 pwm_right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorPWM)))
  "Returns full string definition for message of type 'MotorPWM"
  (cl:format cl:nil "# Represents the PWM value of the left and right motors 0...1~%float32 pwm_left~%float32 pwm_right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorPWM>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorPWM>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorPWM
    (cl:cons ':pwm_left (pwm_left msg))
    (cl:cons ':pwm_right (pwm_right msg))
))
