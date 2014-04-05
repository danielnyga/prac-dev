; Auto-generated. Do not edit!


(cl:in-package mlnpredicates-srv)


;//! \htmlinclude MLNPredicates-request.msg.html

(cl:defclass <MLNPredicates-request> (roslisp-msg-protocol:ros-message)
  ((dummy
    :reader dummy
    :initarg :dummy
    :type cl:string
    :initform ""))
)

(cl:defclass MLNPredicates-request (<MLNPredicates-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MLNPredicates-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MLNPredicates-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mlnpredicates-srv:<MLNPredicates-request> is deprecated: use mlnpredicates-srv:MLNPredicates-request instead.")))

(cl:ensure-generic-function 'dummy-val :lambda-list '(m))
(cl:defmethod dummy-val ((m <MLNPredicates-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mlnpredicates-srv:dummy-val is deprecated.  Use mlnpredicates-srv:dummy instead.")
  (dummy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MLNPredicates-request>) ostream)
  "Serializes a message object of type '<MLNPredicates-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'dummy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'dummy))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MLNPredicates-request>) istream)
  "Deserializes a message object of type '<MLNPredicates-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dummy) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'dummy) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MLNPredicates-request>)))
  "Returns string type for a service object of type '<MLNPredicates-request>"
  "mlnpredicates/MLNPredicatesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MLNPredicates-request)))
  "Returns string type for a service object of type 'MLNPredicates-request"
  "mlnpredicates/MLNPredicatesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MLNPredicates-request>)))
  "Returns md5sum for a message object of type '<MLNPredicates-request>"
  "723d9aac36a8f458c6439a8f71c782b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MLNPredicates-request)))
  "Returns md5sum for a message object of type 'MLNPredicates-request"
  "723d9aac36a8f458c6439a8f71c782b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MLNPredicates-request>)))
  "Returns full string definition for message of type '<MLNPredicates-request>"
  (cl:format cl:nil "string dummy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MLNPredicates-request)))
  "Returns full string definition for message of type 'MLNPredicates-request"
  (cl:format cl:nil "string dummy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MLNPredicates-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'dummy))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MLNPredicates-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MLNPredicates-request
    (cl:cons ':dummy (dummy msg))
))
;//! \htmlinclude MLNPredicates-response.msg.html

(cl:defclass <MLNPredicates-response> (roslisp-msg-protocol:ros-message)
  ((predicates
    :reader predicates
    :initarg :predicates
    :type (cl:vector mlnpredicates-msg:MLNPredicate)
   :initform (cl:make-array 0 :element-type 'mlnpredicates-msg:MLNPredicate :initial-element (cl:make-instance 'mlnpredicates-msg:MLNPredicate))))
)

(cl:defclass MLNPredicates-response (<MLNPredicates-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MLNPredicates-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MLNPredicates-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mlnpredicates-srv:<MLNPredicates-response> is deprecated: use mlnpredicates-srv:MLNPredicates-response instead.")))

(cl:ensure-generic-function 'predicates-val :lambda-list '(m))
(cl:defmethod predicates-val ((m <MLNPredicates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mlnpredicates-srv:predicates-val is deprecated.  Use mlnpredicates-srv:predicates instead.")
  (predicates m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MLNPredicates-response>) ostream)
  "Serializes a message object of type '<MLNPredicates-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'predicates))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'predicates))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MLNPredicates-response>) istream)
  "Deserializes a message object of type '<MLNPredicates-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'predicates) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'predicates)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'mlnpredicates-msg:MLNPredicate))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MLNPredicates-response>)))
  "Returns string type for a service object of type '<MLNPredicates-response>"
  "mlnpredicates/MLNPredicatesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MLNPredicates-response)))
  "Returns string type for a service object of type 'MLNPredicates-response"
  "mlnpredicates/MLNPredicatesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MLNPredicates-response>)))
  "Returns md5sum for a message object of type '<MLNPredicates-response>"
  "723d9aac36a8f458c6439a8f71c782b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MLNPredicates-response)))
  "Returns md5sum for a message object of type 'MLNPredicates-response"
  "723d9aac36a8f458c6439a8f71c782b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MLNPredicates-response>)))
  "Returns full string definition for message of type '<MLNPredicates-response>"
  (cl:format cl:nil "MLNPredicate[] predicates~%~%~%================================================================================~%MSG: mlnpredicates/MLNPredicate~%string name~%string[] domain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MLNPredicates-response)))
  "Returns full string definition for message of type 'MLNPredicates-response"
  (cl:format cl:nil "MLNPredicate[] predicates~%~%~%================================================================================~%MSG: mlnpredicates/MLNPredicate~%string name~%string[] domain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MLNPredicates-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'predicates) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MLNPredicates-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MLNPredicates-response
    (cl:cons ':predicates (predicates msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MLNPredicates)))
  'MLNPredicates-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MLNPredicates)))
  'MLNPredicates-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MLNPredicates)))
  "Returns string type for a service object of type '<MLNPredicates>"
  "mlnpredicates/MLNPredicates")