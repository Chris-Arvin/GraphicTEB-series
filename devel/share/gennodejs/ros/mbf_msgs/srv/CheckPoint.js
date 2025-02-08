// Auto-generated. Do not edit!

// (in-package mbf_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class CheckPointRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.point = null;
      this.costmap = null;
    }
    else {
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = new geometry_msgs.msg.PointStamped();
      }
      if (initObj.hasOwnProperty('costmap')) {
        this.costmap = initObj.costmap
      }
      else {
        this.costmap = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheckPointRequest
    // Serialize message field [point]
    bufferOffset = geometry_msgs.msg.PointStamped.serialize(obj.point, buffer, bufferOffset);
    // Serialize message field [costmap]
    bufferOffset = _serializer.uint8(obj.costmap, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckPointRequest
    let len;
    let data = new CheckPointRequest(null);
    // Deserialize message field [point]
    data.point = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [costmap]
    data.costmap = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PointStamped.getMessageSize(object.point);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mbf_msgs/CheckPointRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '36e9c2f425eee0a2ebd8c4b0aae9f573';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8                      LOCAL_COSTMAP  = 1
    uint8                      GLOBAL_COSTMAP = 2
    
    geometry_msgs/PointStamped point             # the point to be checked after transforming to costmap frame
    uint8                      costmap           # costmap in which to check the point
    
    ================================================================================
    MSG: geometry_msgs/PointStamped
    # This represents a Point with reference coordinate frame and timestamp
    Header header
    Point point
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CheckPointRequest(null);
    if (msg.point !== undefined) {
      resolved.point = geometry_msgs.msg.PointStamped.Resolve(msg.point)
    }
    else {
      resolved.point = new geometry_msgs.msg.PointStamped()
    }

    if (msg.costmap !== undefined) {
      resolved.costmap = msg.costmap;
    }
    else {
      resolved.costmap = 0
    }

    return resolved;
    }
};

// Constants for message
CheckPointRequest.Constants = {
  LOCAL_COSTMAP: 1,
  GLOBAL_COSTMAP: 2,
}

class CheckPointResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
      this.cost = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('cost')) {
        this.cost = initObj.cost
      }
      else {
        this.cost = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheckPointResponse
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [cost]
    bufferOffset = _serializer.uint32(obj.cost, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckPointResponse
    let len;
    let data = new CheckPointResponse(null);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cost]
    data.cost = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mbf_msgs/CheckPointResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd74139e1f7169aa4fb64b44c3a698692';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8                      FREE      =  0    # point is in traversable space
    uint8                      INSCRIBED =  1    # point is in inscribed space
    uint8                      LETHAL    =  2    # point is in collision
    uint8                      UNKNOWN   =  3    # point is in unknown space
    uint8                      OUTSIDE   =  4    # point is outside the map
    
    uint8                      state             # point state: FREE, INFLATED, LETHAL, UNKNOWN or OUTSIDE
    uint32                     cost              # cost of the cell at point
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CheckPointResponse(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.cost !== undefined) {
      resolved.cost = msg.cost;
    }
    else {
      resolved.cost = 0
    }

    return resolved;
    }
};

// Constants for message
CheckPointResponse.Constants = {
  FREE: 0,
  INSCRIBED: 1,
  LETHAL: 2,
  UNKNOWN: 3,
  OUTSIDE: 4,
}

module.exports = {
  Request: CheckPointRequest,
  Response: CheckPointResponse,
  md5sum() { return '098ffffe56a4f2f59d33aa09df0749c1'; },
  datatype() { return 'mbf_msgs/CheckPoint'; }
};
