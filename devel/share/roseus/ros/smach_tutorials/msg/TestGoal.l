;; Auto-generated. Do not edit!


(when (boundp 'smach_tutorials::TestGoal)
  (if (not (find-package "SMACH_TUTORIALS"))
    (make-package "SMACH_TUTORIALS"))
  (shadow 'TestGoal (find-package "SMACH_TUTORIALS")))
(unless (find-package "SMACH_TUTORIALS::TESTGOAL")
  (make-package "SMACH_TUTORIALS::TESTGOAL"))

(in-package "ROS")
;;//! \htmlinclude TestGoal.msg.html


(defclass smach_tutorials::TestGoal
  :super ros::object
  :slots (_goal ))

(defmethod smach_tutorials::TestGoal
  (:init
   (&key
    ((:goal __goal) 0.0)
    )
   (send-super :init)
   (setq _goal (float __goal))
   self)
  (:goal
   (&optional __goal)
   (if __goal (setq _goal __goal)) _goal)
  (:serialization-length
   ()
   (+
    ;; float64 _goal
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _goal
       (sys::poke _goal (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _goal
     (setq _goal (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get smach_tutorials::TestGoal :md5sum-) "96f1fc969cebfe9056357b5db1aa501e")
(setf (get smach_tutorials::TestGoal :datatype-) "smach_tutorials/TestGoal")
(setf (get smach_tutorials::TestGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
float64 goal

")



(provide :smach_tutorials/TestGoal "96f1fc969cebfe9056357b5db1aa501e")

