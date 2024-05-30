;; Auto-generated. Do not edit!


(when (boundp 'motor::MotorPWM)
  (if (not (find-package "MOTOR"))
    (make-package "MOTOR"))
  (shadow 'MotorPWM (find-package "MOTOR")))
(unless (find-package "MOTOR::MOTORPWM")
  (make-package "MOTOR::MOTORPWM"))

(in-package "ROS")
;;//! \htmlinclude MotorPWM.msg.html


(defclass motor::MotorPWM
  :super ros::object
  :slots (_pwm_left _pwm_right ))

(defmethod motor::MotorPWM
  (:init
   (&key
    ((:pwm_left __pwm_left) 0.0)
    ((:pwm_right __pwm_right) 0.0)
    )
   (send-super :init)
   (setq _pwm_left (float __pwm_left))
   (setq _pwm_right (float __pwm_right))
   self)
  (:pwm_left
   (&optional __pwm_left)
   (if __pwm_left (setq _pwm_left __pwm_left)) _pwm_left)
  (:pwm_right
   (&optional __pwm_right)
   (if __pwm_right (setq _pwm_right __pwm_right)) _pwm_right)
  (:serialization-length
   ()
   (+
    ;; float32 _pwm_left
    4
    ;; float32 _pwm_right
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pwm_left
       (sys::poke _pwm_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pwm_right
       (sys::poke _pwm_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pwm_left
     (setq _pwm_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pwm_right
     (setq _pwm_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get motor::MotorPWM :md5sum-) "d7a5f6b78fec2b5366e39d282d33bb64")
(setf (get motor::MotorPWM :datatype-) "motor/MotorPWM")
(setf (get motor::MotorPWM :definition-)
      "# Represents the PWM value of the left and right motors 0...1
float32 pwm_left
float32 pwm_right
")



(provide :motor/MotorPWM "d7a5f6b78fec2b5366e39d282d33bb64")


