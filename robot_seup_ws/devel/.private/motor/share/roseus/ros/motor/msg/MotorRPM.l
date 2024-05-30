;; Auto-generated. Do not edit!


(when (boundp 'motor::MotorRPM)
  (if (not (find-package "MOTOR"))
    (make-package "MOTOR"))
  (shadow 'MotorRPM (find-package "MOTOR")))
(unless (find-package "MOTOR::MOTORRPM")
  (make-package "MOTOR::MOTORRPM"))

(in-package "ROS")
;;//! \htmlinclude MotorRPM.msg.html


(defclass motor::MotorRPM
  :super ros::object
  :slots (_rpm_left _rpm_right ))

(defmethod motor::MotorRPM
  (:init
   (&key
    ((:rpm_left __rpm_left) 0.0)
    ((:rpm_right __rpm_right) 0.0)
    )
   (send-super :init)
   (setq _rpm_left (float __rpm_left))
   (setq _rpm_right (float __rpm_right))
   self)
  (:rpm_left
   (&optional __rpm_left)
   (if __rpm_left (setq _rpm_left __rpm_left)) _rpm_left)
  (:rpm_right
   (&optional __rpm_right)
   (if __rpm_right (setq _rpm_right __rpm_right)) _rpm_right)
  (:serialization-length
   ()
   (+
    ;; float32 _rpm_left
    4
    ;; float32 _rpm_right
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _rpm_left
       (sys::poke _rpm_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rpm_right
       (sys::poke _rpm_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _rpm_left
     (setq _rpm_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rpm_right
     (setq _rpm_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get motor::MotorRPM :md5sum-) "811f89cd5e54718d6b2167dcea919c32")
(setf (get motor::MotorRPM :datatype-) "motor/MotorRPM")
(setf (get motor::MotorRPM :definition-)
      "# Represents the RPM value of the left and right motors 0...1
float32 rpm_left
float32 rpm_right
")



(provide :motor/MotorRPM "811f89cd5e54718d6b2167dcea919c32")


