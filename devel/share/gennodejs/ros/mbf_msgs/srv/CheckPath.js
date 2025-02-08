// Auto-generated. Do not edit!

// (in-package mbf_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let nav_msgs = _finder('nav_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class CheckPathRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.path = null;
      this.safety_dist = null;
      this.lethal_cost_mult = null;
      this.inscrib_cost_mult = null;
      this.unknown_cost_mult = null;
      this.costmap = null;
      this.return_on = null;
      this.skip_poses = null;
      this.path_cells_only = null;
    }
    else {
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = new nav_msgs.msg.Path();
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
      if (initObj.hasOwnProperty('return_on')) {
        this.return_on = initObj.return_on
      }
      else {
        this.return_on = 0;
      }
      if (initObj.hasOwnProperty('skip_poses')) {
        this.skip_poses = initObj.skip_poses
      }
      else {
        this.skip_poses = 0;
      }
      if (initObj.hasOwnProperty('path_cells_only')) {
        this.path_cells_only = initObj.path_cells_only
      }
      else {
        this.path_cells_only = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheckPathRequest
    // Serialize message field [path]
    bufferOffset = nav_msgs.msg.Path.serialize(obj.path, buffer, bufferOffset);
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
    // Serialize message field [return_on]
    bufferOffset = _serializer.uint8(obj.return_on, buffer, bufferOffset);
    // Serialize message field [skip_poses]
    bufferOffset = _serializer.uint8(obj.skip_poses, buffer, bufferOffset);
    // Serialize message field [path_cells_only]
    bufferOffset = _serializer.bool(obj.path_cells_only, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckPathRequest
    let len;
    let data = new CheckPathRequest(null);
    // Deserialize message field [path]
    data.path = nav_msgs.msg.Path.deserialize(buffer, bufferOffset);
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
    // Deserialize message field [return_on]
    data.return_on = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [skip_poses]
    data.skip_poses = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [path_cells_only]
    data.path_cells_only = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += nav_msgs.msg.Path.getMessageSize(object.path);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mbf_msgs/CheckPathRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '16f70020b17f5c034724ed8fb518b9f5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8                      LOCAL_COSTMAP  = 1
    uint8                      GLOBAL_COSTMAP = 2
    
    nav_msgs/Path              path              # the path to be checked after transforming to costmap frame
    float32                    safety_dist       # minimum distance allowed to the closest obstacle (footprint padding)
    float32                    lethal_cost_mult  # cost multiplier for cells marked as lethal obstacle (zero is ignored)
    float32                    inscrib_cost_mult # cost multiplier for cells marked as inscribed obstacle (zero is ignored)
    float32                    unknown_cost_mult # cost multiplier for cells marked as unknown space (zero is ignored)
    uint8                      costmap           # costmap in which to check the pose
    uint8                      return_on         # abort check on finding a pose with this state or worse (zero is ignored)
    uint8                      skip_poses        # skip this number of poses between checks, to speedup processing
    bool                       path_cells_only   # check only cells directly traversed by the path, ignoring robot footprint
                                                 # (if true, safety_dist is ignored)
    
    ================================================================================
    MSG: nav_msgs/Path
    #An array of poses that represents a Path for a robot to follow
    Header header
    geometry_msgs/PoseStamped[] poses
    
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
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    const resolved = new CheckPathRequest(null);
    if (msg.path !== undefined) {
      resolved.path = nav_msgs.msg.Path.Resolve(msg.path)
    }
    else {
      resolved.path = new nav_msgs.msg.Path()
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

    if (msg.return_on !== undefined) {
      resolved.return_on = msg.return_on;
    }
    else {
      resolved.return_on = 0
    }

    if (msg.skip_poses !== undefined) {
      resolved.skip_poses = msg.skip_poses;
    }
    else {
      resolved.skip_poses = 0
    }

    if (msg.path_cells_only !== undefined) {
      resolved.path_cells_only = msg.path_cells_only;
    }
    else {
      resolved.path_cells_only = false
    }

    return resolved;
    }
};

// Constants for message
CheckPathRequest.Constants = {
  LOCAL_COSTMAP: 1,
  GLOBAL_COSTMAP: 2,
}

class CheckPathResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.last_checked = null;
      this.state = null;
      this.cost = null;
    }
    else {
      if (initObj.hasOwnProperty('last_checked')) {
        this.last_checked = initObj.last_checked
      }
      else {
        this.last_checked = 0;
      }
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
    // Serializes a message object of type CheckPathResponse
    // Serialize message field [last_checked]
    bufferOffset = _serializer.uint32(obj.last_checked, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [cost]
    bufferOffset = _serializer.uint32(obj.cost, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckPathResponse
    let len;
    let data = new CheckPathResponse(null);
    // Deserialize message field [last_checked]
    data.last_checked = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cost]
    data.cost = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mbf_msgs/CheckPathResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '420eb6a13d128bba3770710452ea1c17';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8                      FREE      =  0    # path is completely in traversable space
    uint8                      INSCRIBED =  1    # path is partially in inscribed space
    uint8                      LETHAL    =  2    # path is partially in collision
    uint8                      UNKNOWN   =  3    # path is partially in unknown space
    uint8                      OUTSIDE   =  4    # path is partially outside the map
    
    uint32                     last_checked      # index of the first pose along the path with return_on state or worse
    uint8                      state             # path worst state (until last_checked): FREE, INFLATED, LETHAL, UNKNOWN...
    uint32                     cost              # cost of all cells traversed by path within footprint padded by safety_dist
                                                 # or just by the path if path_cells_only is true
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CheckPathResponse(null);
    if (msg.last_checked !== undefined) {
      resolved.last_checked = msg.last_checked;
    }
    else {
      resolved.last_checked = 0
    }

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
CheckPathResponse.Constants = {
  FREE: 0,
  INSCRIBED: 1,
  LETHAL: 2,
  UNKNOWN: 3,
  OUTSIDE: 4,
}

module.exports = {
  Request: CheckPathRequest,
  Response: CheckPathResponse,
  md5sum() { return '59ef27f5ce7e3cc3c7f7a0f823bca55c'; },
  datatype() { return 'mbf_msgs/CheckPath'; }
};
