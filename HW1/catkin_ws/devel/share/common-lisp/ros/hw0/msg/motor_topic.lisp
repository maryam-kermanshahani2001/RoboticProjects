; Auto-generated. Do not edit!


(cl:in-package hw0-msg)


;//! \htmlinclude motor_topic.msg.html

(cl:defclass <motor_topic> (roslisp-msg-protocol:ros-message)
  ((rotation
    :reader rotation
    :initarg :rotation
    :type cl:integer
    :initform 0)
   (clockwise
    :reader clockwise
    :initarg :clockwise
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass motor_topic (<motor_topic>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_topic>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_topic)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hw0-msg:<motor_topic> is deprecated: use hw0-msg:motor_topic instead.")))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <motor_topic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hw0-msg:rotation-val is deprecated.  Use hw0-msg:rotation instead.")
  (rotation m))

(cl:ensure-generic-function 'clockwise-val :lambda-list '(m))
(cl:defmethod clockwise-val ((m <motor_topic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hw0-msg:clockwise-val is deprecated.  Use hw0-msg:clockwise instead.")
  (clockwise m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_topic>) ostream)
  "Serializes a message object of type '<motor_topic>"
  (cl:let* ((signed (cl:slot-value msg 'rotation)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clockwise) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_topic>) istream)
  "Deserializes a message object of type '<motor_topic>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rotation) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:setf (cl:slot-value msg 'clockwise) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_topic>)))
  "Returns string type for a message object of type '<motor_topic>"
  "hw0/motor_topic")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_topic)))
  "Returns string type for a message object of type 'motor_topic"
  "hw0/motor_topic")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_topic>)))
  "Returns md5sum for a message object of type '<motor_topic>"
  "3f09a113fbdc065e6685b77041855ae5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_topic)))
  "Returns md5sum for a message object of type 'motor_topic"
  "3f09a113fbdc065e6685b77041855ae5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_topic>)))
  "Returns full string definition for message of type '<motor_topic>"
  (cl:format cl:nil "int64 rotation~%bool clockwise~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_topic)))
  "Returns full string definition for message of type 'motor_topic"
  (cl:format cl:nil "int64 rotation~%bool clockwise~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_topic>))
  (cl:+ 0
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_topic>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_topic
    (cl:cons ':rotation (rotation msg))
    (cl:cons ':clockwise (clockwise msg))
))
