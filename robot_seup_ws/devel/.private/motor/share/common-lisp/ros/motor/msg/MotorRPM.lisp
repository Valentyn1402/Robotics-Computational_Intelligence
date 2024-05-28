; Auto-generated. Do not edit!


(cl:in-package motor-msg)


;//! \htmlinclude MotorRPM.msg.html

(cl:defclass <MotorRPM> (roslisp-msg-protocol:ros-message)
  ((rpm_left
    :reader rpm_left
    :initarg :rpm_left
    :type cl:float
    :initform 0.0)
   (rpm_right
    :reader rpm_right
    :initarg :rpm_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass MotorRPM (<MotorRPM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorRPM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorRPM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor-msg:<MotorRPM> is deprecated: use motor-msg:MotorRPM instead.")))

(cl:ensure-generic-function 'rpm_left-val :lambda-list '(m))
(cl:defmethod rpm_left-val ((m <MotorRPM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor-msg:rpm_left-val is deprecated.  Use motor-msg:rpm_left instead.")
  (rpm_left m))

(cl:ensure-generic-function 'rpm_right-val :lambda-list '(m))
(cl:defmethod rpm_right-val ((m <MotorRPM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor-msg:rpm_right-val is deprecated.  Use motor-msg:rpm_right instead.")
  (rpm_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorRPM>) ostream)
  "Serializes a message object of type '<MotorRPM>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rpm_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rpm_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorRPM>) istream)
  "Deserializes a message object of type '<MotorRPM>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rpm_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rpm_right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorRPM>)))
  "Returns string type for a message object of type '<MotorRPM>"
  "motor/MotorRPM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorRPM)))
  "Returns string type for a message object of type 'MotorRPM"
  "motor/MotorRPM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorRPM>)))
  "Returns md5sum for a message object of type '<MotorRPM>"
  "811f89cd5e54718d6b2167dcea919c32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorRPM)))
  "Returns md5sum for a message object of type 'MotorRPM"
  "811f89cd5e54718d6b2167dcea919c32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorRPM>)))
  "Returns full string definition for message of type '<MotorRPM>"
  (cl:format cl:nil "# Represents the RPM value of the left and right motors 0...1~%float32 rpm_left~%float32 rpm_right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorRPM)))
  "Returns full string definition for message of type 'MotorRPM"
  (cl:format cl:nil "# Represents the RPM value of the left and right motors 0...1~%float32 rpm_left~%float32 rpm_right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorRPM>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorRPM>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorRPM
    (cl:cons ':rpm_left (rpm_left msg))
    (cl:cons ':rpm_right (rpm_right msg))
))
