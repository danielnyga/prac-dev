; Auto-generated. Do not edit!


(cl:in-package mlnpredicates-msg)


;//! \htmlinclude MLNPredicate.msg.html

(cl:defclass <MLNPredicate> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (domain
    :reader domain
    :initarg :domain
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass MLNPredicate (<MLNPredicate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MLNPredicate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MLNPredicate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mlnpredicates-msg:<MLNPredicate> is deprecated: use mlnpredicates-msg:MLNPredicate instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <MLNPredicate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mlnpredicates-msg:name-val is deprecated.  Use mlnpredicates-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'domain-val :lambda-list '(m))
(cl:defmethod domain-val ((m <MLNPredicate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mlnpredicates-msg:domain-val is deprecated.  Use mlnpredicates-msg:domain instead.")
  (domain m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MLNPredicate>) ostream)
  "Serializes a message object of type '<MLNPredicate>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'domain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'domain))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MLNPredicate>) istream)
  "Deserializes a message object of type '<MLNPredicate>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'domain) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'domain)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MLNPredicate>)))
  "Returns string type for a message object of type '<MLNPredicate>"
  "mlnpredicates/MLNPredicate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MLNPredicate)))
  "Returns string type for a message object of type 'MLNPredicate"
  "mlnpredicates/MLNPredicate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MLNPredicate>)))
  "Returns md5sum for a message object of type '<MLNPredicate>"
  "9992c37cb772d59ee22a4bf22c84177b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MLNPredicate)))
  "Returns md5sum for a message object of type 'MLNPredicate"
  "9992c37cb772d59ee22a4bf22c84177b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MLNPredicate>)))
  "Returns full string definition for message of type '<MLNPredicate>"
  (cl:format cl:nil "string name~%string[] domain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MLNPredicate)))
  "Returns full string definition for message of type 'MLNPredicate"
  (cl:format cl:nil "string name~%string[] domain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MLNPredicate>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'domain) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MLNPredicate>))
  "Converts a ROS message object to a list"
  (cl:list 'MLNPredicate
    (cl:cons ':name (name msg))
    (cl:cons ':domain (domain msg))
))
