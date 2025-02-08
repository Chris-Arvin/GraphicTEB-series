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

class CheckPoseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
      this.safety_dist = null;
      this.lethal_cost_mult = null;
      this.inscrib_cost_mult = null;
      this.unknown_cost_mult = null;
      this.costmap = null;
      this.current_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('safety_dist')) {
        this.safety_dist = initObj.safety_dist
      }
      else {
        this.safety_dist = 0.0;
      }
      if (initObj.hasOwnProperty('lethal_cost_mult')) {
        this.lethal_cost_mult = initObj.lethal_cost_mult
      }
      else {
        this.lethal_cost_mult = 0.0;
      }
      if (initObj.hasOwnProperty('inscrib_cost_mult')) {
        this.inscrib_cost_mult = initObj.inscrib_cost_mult
      }
      else {
        this.inscrib_cost_mult = 0.0;
      }
      if (initObj.hasOwnProperty('unknown_cost_mult')) {
        this.unknown_cost_mult = initObj.unknown_cost_mult
      }
      else {
        this.unknown_cost_mult = 0.0;
      }
      if (initObj.hasOwnProperty('costmap')) {
        this.costmap = initObj.costmap
      }
      else {
        this.costmap = 0;
      }
      if (initObj.hasOwnProperty('current_pose')) {
        this.current_pose = initObj.current_pose
      }
      else {
        this.current_pose = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheckPoseRequest
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [safety_dist]
    bufferOffset = _serializer.float32(obj.safety_dist, buffer, bufferOffset);
    // Serialize message field [lethal_cost_mult]
    bufferOffset = _serializer.float32(obj.lethal_cost_mult, buffer, bufferOffset);
    // Serialize message field [inscrib_cost_mult]
    bufferOffset = _serializer.float32(obj.inscrib_cost_mult, buffer, bufferOffset);
    // Serialize message field [unknown_cost_mult]
    bufferOffset = _serializer.float32(obj.unknown_cost_mult, buffer, bufferOffset);
    // Serialize message field [costmap]
    bufferOffset = _serializer.uint8(obj.costmap, buffer, bufferOffset);
    // Serialize message field [current_pose]
    bufferOffset = _serializer.bool(obj.current_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckPoseRequest
    let len;
    let data = new CheckPoseRequest(null);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [safety_dist]
    data.safety_dist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lethal_cost_mult]
    data.lethal_cost_mult = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [inscrib_cost_mult]
    data.inscrib_cost_mult = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [unknown_cost_mult]
    data.unknown_cost_mult = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [costmap]
    data.costmap = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [current_pose]
    data.current_pose = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.pose);
    return length + 18;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mbf_msgs/CheckPoseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '265e0591fcb39074b9d918fb13f423f4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8                      LOCAL_COSTMAP  = 1
    uint8                      GLOBAL_COSTMAP = 2
    
    geometry_msgs/PoseStamped  pose              # the pose to be checked after transforming to costmap frame
    float32                    safety_dist       # minimum distance allowed to the closest obstacle
    float32                    lethal_cost_mult  # cost multiplier for cells marked as lethal obstacle (zero is ignored)
    float32                    inscrib_cost_mult # cost multiplier for cells marked as inscribed obstacle (zero is ignored)
    float32                    unknown_cost_mult # cost multiplier for cells marked as unknown space (zero is ignored)
    uint8                      costmap           # costmap in which to check the pose
    bool                       current_pose      # check current robot pose instead (ignores pose field)
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CheckPoseRequest(null);
    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.PoseStamped.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.safety_dist !== undefined) {
      resolved.safety_dist = msg.safety_dist;
    }
    else {
      resolved.safety_dist = 0.0
    }

    if (msg.lethal_cost_mult !== undefined) {
      resolved.lethal_cost_mult = msg.lethal_cost_mult;
    }
    else {
      resolved.lethal_cost_mult = 0.0
    }

    if (msg.inscrib_cost_mult !== undefined) {
      resolved.inscrib_cost_mult = msg.inscrib_cost_mult;
    }
    else {
      resolved.inscrib_cost_mult = 0.0
    }

    if (msg.unknown_cost_mult !== undefined) {
      resolved.unknown_cost_mult = msg.unknown_cost_mult;
    }
    else {
      resolved.unknown_cost_mult = 0.0
    }

    if (msg.costmap !== undefined) {
      resolved.costmap = msg.costmap;
    }
    else {
      resolved.costmap = 0
    }

    if (msg.current_pose !== undefined) {
      resolved.current_pose = msg.current_pose;
    }
    else {
      resolved.current_pose = false
    }

    return resolved;
    }
};

// Constants for message
CheckPoseRequest.Constants = {
  LOCAL_COSTMAP: 1,
  GLOBAL_COSTMAP: 2,
}

class CheckPoseResponse {
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
    // Serializes a message object of type CheckPoseResponse
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [cost]
    bufferOffset = _serializer.uint32(obj.cost, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckPoseResponse
    let len;
    let data = new CheckPoseResponse(null);
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
    return 'mbf_msgs/CheckPoseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd74139e1f7169aa4fb64b44c3a698692';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8                      FREE      =  0    # robot is completely in traversable space
    uint8                      INSCRIBED =  1    # robot is partially in inscribed space
    uint8                      LETHAL    =  2    # robot is partially in collision
    uint8                      UNKNOWN   =  3    # robot is partially in unknown space
    uint8                      OUTSIDE   =  4    # robot is partially outside the map
    
    uint8                      state             # pose state: FREE, INFLATED, LETHAL, UNKNOWN or OUTSIDE
    uint32                     cost              # total cost of all cells within footprint padded by safety_dist
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CheckPoseResponse(null);
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
CheckPoseResponse.Constants = {
  FREE: 0,
  INSCRIBED: 1,
  LETHAL: 2,
  UNKNOWN: 3,
  OUTSIDE: 4,
}

module.exports = {
  Request: CheckPoseRequest,
  Response: CheckPoseResponse,
  md5sum() { return '540ccc67006cf1c2c5502427e26c7f21'; },
  datatype() { return 'mbf_msgs/CheckPose'; }
};
