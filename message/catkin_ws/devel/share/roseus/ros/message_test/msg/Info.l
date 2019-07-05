;; Auto-generated. Do not edit!


(when (boundp 'message_test::Info)
  (if (not (find-package "MESSAGE_TEST"))
    (make-package "MESSAGE_TEST"))
  (shadow 'Info (find-package "MESSAGE_TEST")))
(unless (find-package "MESSAGE_TEST::INFO")
  (make-package "MESSAGE_TEST::INFO"))

(in-package "ROS")
;;//! \htmlinclude Info.msg.html


(defclass message_test::Info
  :super ros::object
  :slots (_scores _boxes _classes _detected ))

(defmethod message_test::Info
  (:init
   (&key
    ((:scores __scores) (make-array 0 :initial-element 0 :element-type :integer))
    ((:boxes __boxes) (make-array 0 :initial-element 0 :element-type :integer))
    ((:classes __classes) (make-array 0 :initial-element 0 :element-type :integer))
    ((:detected __detected) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _scores __scores)
   (setq _boxes __boxes)
   (setq _classes __classes)
   (setq _detected __detected)
   self)
  (:scores
   (&optional __scores)
   (if __scores (setq _scores __scores)) _scores)
  (:boxes
   (&optional __boxes)
   (if __boxes (setq _boxes __boxes)) _boxes)
  (:classes
   (&optional __classes)
   (if __classes (setq _classes __classes)) _classes)
  (:detected
   (&optional __detected)
   (if __detected (setq _detected __detected)) _detected)
  (:serialization-length
   ()
   (+
    ;; int32[] _scores
    (* 4    (length _scores)) 4
    ;; int32[] _boxes
    (* 4    (length _boxes)) 4
    ;; int32[] _classes
    (* 4    (length _classes)) 4
    ;; int32[] _detected
    (* 4    (length _detected)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _scores
     (write-long (length _scores) s)
     (dotimes (i (length _scores))
       (write-long (elt _scores i) s)
       )
     ;; int32[] _boxes
     (write-long (length _boxes) s)
     (dotimes (i (length _boxes))
       (write-long (elt _boxes i) s)
       )
     ;; int32[] _classes
     (write-long (length _classes) s)
     (dotimes (i (length _classes))
       (write-long (elt _classes i) s)
       )
     ;; int32[] _detected
     (write-long (length _detected) s)
     (dotimes (i (length _detected))
       (write-long (elt _detected i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _scores
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _scores (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _scores i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _boxes (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _boxes i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _classes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _classes (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _classes i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _detected
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detected (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _detected i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get message_test::Info :md5sum-) "c873532b9ae33423f8d1b3d0ddf4f322")
(setf (get message_test::Info :datatype-) "message_test/Info")
(setf (get message_test::Info :definition-)
      "int32[] scores
int32[] boxes
int32[] classes
int32[] detected
")



(provide :message_test/Info "c873532b9ae33423f8d1b3d0ddf4f322")


