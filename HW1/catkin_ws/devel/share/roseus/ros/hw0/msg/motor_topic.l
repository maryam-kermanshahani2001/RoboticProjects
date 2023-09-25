;; Auto-generated. Do not edit!


(when (boundp 'hw0::motor_topic)
  (if (not (find-package "HW0"))
    (make-package "HW0"))
  (shadow 'motor_topic (find-package "HW0")))
(unless (find-package "HW0::MOTOR_TOPIC")
  (make-package "HW0::MOTOR_TOPIC"))

(in-package "ROS")
;;//! \htmlinclude motor_topic.msg.html


(defclass hw0::motor_topic
  :super ros::object
  :slots (_rotation _clockwise ))

(defmethod hw0::motor_topic
  (:init
   (&key
    ((:rotation __rotation) 0)
    ((:clockwise __clockwise) nil)
    )
   (send-super :init)
   (setq _rotation (round __rotation))
   (setq _clockwise __clockwise)
   self)
  (:rotation
   (&optional __rotation)
   (if __rotation (setq _rotation __rotation)) _rotation)
  (:clockwise
   (&optional (__clockwise :null))
   (if (not (eq __clockwise :null)) (setq _clockwise __clockwise)) _clockwise)
  (:serialization-length
   ()
   (+
    ;; int64 _rotation
    8
    ;; bool _clockwise
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _rotation
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _rotation (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _rotation) (= (length (_rotation . bv)) 2)) ;; bignum
              (write-long (ash (elt (_rotation . bv) 0) 0) s)
              (write-long (ash (elt (_rotation . bv) 1) -1) s))
             ((and (class _rotation) (= (length (_rotation . bv)) 1)) ;; big1
              (write-long (elt (_rotation . bv) 0) s)
              (write-long (if (>= _rotation 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _rotation s)(write-long (if (>= _rotation 0) 0 #xffffffff) s)))
     ;; bool _clockwise
       (if _clockwise (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _rotation
#+(or :alpha :irix6 :x86_64)
      (setf _rotation (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _rotation (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; bool _clockwise
     (setq _clockwise (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get hw0::motor_topic :md5sum-) "3f09a113fbdc065e6685b77041855ae5")
(setf (get hw0::motor_topic :datatype-) "hw0/motor_topic")
(setf (get hw0::motor_topic :definition-)
      "int64 rotation
bool clockwise
")



(provide :hw0/motor_topic "3f09a113fbdc065e6685b77041855ae5")


