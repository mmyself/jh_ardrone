; Auto-generated. Do not edit!


(cl:in-package ar_localizer-srv)


;//! \htmlinclude GenerateLocations-request.msg.html

(cl:defclass <GenerateLocations-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GenerateLocations-request (<GenerateLocations-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GenerateLocations-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GenerateLocations-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ar_localizer-srv:<GenerateLocations-request> is deprecated: use ar_localizer-srv:GenerateLocations-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GenerateLocations-request>) ostream)
  "Serializes a message object of type '<GenerateLocations-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GenerateLocations-request>) istream)
  "Deserializes a message object of type '<GenerateLocations-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GenerateLocations-request>)))
  "Returns string type for a service object of type '<GenerateLocations-request>"
  "ar_localizer/GenerateLocationsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateLocations-request)))
  "Returns string type for a service object of type 'GenerateLocations-request"
  "ar_localizer/GenerateLocationsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GenerateLocations-request>)))
  "Returns md5sum for a message object of type '<GenerateLocations-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GenerateLocations-request)))
  "Returns md5sum for a message object of type 'GenerateLocations-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GenerateLocations-request>)))
  "Returns full string definition for message of type '<GenerateLocations-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GenerateLocations-request)))
  "Returns full string definition for message of type 'GenerateLocations-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GenerateLocations-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GenerateLocations-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GenerateLocations-request
))
;//! \htmlinclude GenerateLocations-response.msg.html

(cl:defclass <GenerateLocations-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GenerateLocations-response (<GenerateLocations-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GenerateLocations-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GenerateLocations-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ar_localizer-srv:<GenerateLocations-response> is deprecated: use ar_localizer-srv:GenerateLocations-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GenerateLocations-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_localizer-srv:success-val is deprecated.  Use ar_localizer-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GenerateLocations-response>) ostream)
  "Serializes a message object of type '<GenerateLocations-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GenerateLocations-response>) istream)
  "Deserializes a message object of type '<GenerateLocations-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GenerateLocations-response>)))
  "Returns string type for a service object of type '<GenerateLocations-response>"
  "ar_localizer/GenerateLocationsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateLocations-response)))
  "Returns string type for a service object of type 'GenerateLocations-response"
  "ar_localizer/GenerateLocationsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GenerateLocations-response>)))
  "Returns md5sum for a message object of type '<GenerateLocations-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GenerateLocations-response)))
  "Returns md5sum for a message object of type 'GenerateLocations-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GenerateLocations-response>)))
  "Returns full string definition for message of type '<GenerateLocations-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GenerateLocations-response)))
  "Returns full string definition for message of type 'GenerateLocations-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GenerateLocations-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GenerateLocations-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GenerateLocations-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GenerateLocations)))
  'GenerateLocations-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GenerateLocations)))
  'GenerateLocations-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateLocations)))
  "Returns string type for a service object of type '<GenerateLocations>"
  "ar_localizer/GenerateLocations")
