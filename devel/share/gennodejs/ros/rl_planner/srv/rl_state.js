// Auto-generated. Do not edit!

// (in-package rl_planner.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let visualization_msgs = _finder('visualization_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class rl_stateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.map_static_position = null;
      this.map_dynamic_velocity_x = null;
      this.map_dynamic_velocity_y = null;
      this.trajectories = null;
      this.vx = null;
      this.vy = null;
      this.last_static_safety_margin = null;
      this.last_dynamic_safety_margin = null;
    }
    else {
      if (initObj.hasOwnProperty('map_static_position')) {
        this.map_static_position = initObj.map_static_position
      }
      else {
        this.map_static_position = [];
      }
      if (initObj.hasOwnProperty('map_dynamic_velocity_x')) {
        this.map_dynamic_velocity_x = initObj.map_dynamic_velocity_x
      }
      else {
        this.map_dynamic_velocity_x = [];
      }
      if (initObj.hasOwnProperty('map_dynamic_velocity_y')) {
        this.map_dynamic_velocity_y = initObj.map_dynamic_velocity_y
      }
      else {
        this.map_dynamic_velocity_y = [];
      }
      if (initObj.hasOwnProperty('trajectories')) {
        this.trajectories = initObj.trajectories
      }
      else {
        this.trajectories = new visualization_msgs.msg.MarkerArray();
      }
      if (initObj.hasOwnProperty('vx')) {
        this.vx = initObj.vx
      }
      else {
        this.vx = 0.0;
      }
      if (initObj.hasOwnProperty('vy')) {
        this.vy = initObj.vy
      }
      else {
        this.vy = 0.0;
      }
      if (initObj.hasOwnProperty('last_static_safety_margin')) {
        this.last_static_safety_margin = initObj.last_static_safety_margin
      }
      else {
        this.last_static_safety_margin = 0.0;
      }
      if (initObj.hasOwnProperty('last_dynamic_safety_margin')) {
        this.last_dynamic_safety_margin = initObj.last_dynamic_safety_margin
      }
      else {
        this.last_dynamic_safety_margin = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rl_stateRequest
    // Serialize message field [map_static_position]
    bufferOffset = _arraySerializer.int32(obj.map_static_position, buffer, bufferOffset, null);
    // Serialize message field [map_dynamic_velocity_x]
    bufferOffset = _arraySerializer.float32(obj.map_dynamic_velocity_x, buffer, bufferOffset, null);
    // Serialize message field [map_dynamic_velocity_y]
    bufferOffset = _arraySerializer.float32(obj.map_dynamic_velocity_y, buffer, bufferOffset, null);
    // Serialize message field [trajectories]
    bufferOffset = visualization_msgs.msg.MarkerArray.serialize(obj.trajectories, buffer, bufferOffset);
    // Serialize message field [vx]
    bufferOffset = _serializer.float32(obj.vx, buffer, bufferOffset);
    // Serialize message field [vy]
    bufferOffset = _serializer.float32(obj.vy, buffer, bufferOffset);
    // Serialize message field [last_static_safety_margin]
    bufferOffset = _serializer.float32(obj.last_static_safety_margin, buffer, bufferOffset);
    // Serialize message field [last_dynamic_safety_margin]
    bufferOffset = _serializer.float32(obj.last_dynamic_safety_margin, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rl_stateRequest
    let len;
    let data = new rl_stateRequest(null);
    // Deserialize message field [map_static_position]
    data.map_static_position = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [map_dynamic_velocity_x]
    data.map_dynamic_velocity_x = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [map_dynamic_velocity_y]
    data.map_dynamic_velocity_y = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [trajectories]
    data.trajectories = visualization_msgs.msg.MarkerArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [vx]
    data.vx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vy]
    data.vy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [last_static_safety_margin]
    data.last_static_safety_margin = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [last_dynamic_safety_margin]
    data.last_dynamic_safety_margin = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.map_static_position.length;
    length += 4 * object.map_dynamic_velocity_x.length;
    length += 4 * object.map_dynamic_velocity_y.length;
    length += visualization_msgs.msg.MarkerArray.getMessageSize(object.trajectories);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rl_planner/rl_stateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '73d26f25021558402f8110e4b5bc55bc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request 部分
    int32[]  map_static_position
    float32[]  map_dynamic_velocity_x
    float32[]  map_dynamic_velocity_y
    visualization_msgs/MarkerArray trajectories
    float32 vx
    float32 vy
    float32 last_static_safety_margin
    float32 last_dynamic_safety_margin
    
    ================================================================================
    MSG: visualization_msgs/MarkerArray
    Marker[] markers
    
    ================================================================================
    MSG: visualization_msgs/Marker
    # See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz
    
    uint8 ARROW=0
    uint8 CUBE=1
    uint8 SPHERE=2
    uint8 CYLINDER=3
    uint8 LINE_STRIP=4
    uint8 LINE_LIST=5
    uint8 CUBE_LIST=6
    uint8 SPHERE_LIST=7
    uint8 POINTS=8
    uint8 TEXT_VIEW_FACING=9
    uint8 MESH_RESOURCE=10
    uint8 TRIANGLE_LIST=11
    
    uint8 ADD=0
    uint8 MODIFY=0
    uint8 DELETE=2
    uint8 DELETEALL=3
    
    Header header                        # header for time/frame information
    string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
    int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
    int32 type 		                       # Type of object
    int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
    geometry_msgs/Pose pose                 # Pose of the object
    geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
    std_msgs/ColorRGBA color             # Color [0.0-1.0]
    duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
    bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep
    
    #Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
    geometry_msgs/Point[] points
    #Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
    #number of colors must either be 0 or equal to the number of points
    #NOTE: alpha is not yet used
    std_msgs/ColorRGBA[] colors
    
    # NOTE: only used for text markers
    string text
    
    # NOTE: only used for MESH_RESOURCE markers
    string mesh_resource
    bool mesh_use_embedded_materials
    
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
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: std_msgs/ColorRGBA
    float32 r
    float32 g
    float32 b
    float32 a
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rl_stateRequest(null);
    if (msg.map_static_position !== undefined) {
      resolved.map_static_position = msg.map_static_position;
    }
    else {
      resolved.map_static_position = []
    }

    if (msg.map_dynamic_velocity_x !== undefined) {
      resolved.map_dynamic_velocity_x = msg.map_dynamic_velocity_x;
    }
    else {
      resolved.map_dynamic_velocity_x = []
    }

    if (msg.map_dynamic_velocity_y !== undefined) {
      resolved.map_dynamic_velocity_y = msg.map_dynamic_velocity_y;
    }
    else {
      resolved.map_dynamic_velocity_y = []
    }

    if (msg.trajectories !== undefined) {
      resolved.trajectories = visualization_msgs.msg.MarkerArray.Resolve(msg.trajectories)
    }
    else {
      resolved.trajectories = new visualization_msgs.msg.MarkerArray()
    }

    if (msg.vx !== undefined) {
      resolved.vx = msg.vx;
    }
    else {
      resolved.vx = 0.0
    }

    if (msg.vy !== undefined) {
      resolved.vy = msg.vy;
    }
    else {
      resolved.vy = 0.0
    }

    if (msg.last_static_safety_margin !== undefined) {
      resolved.last_static_safety_margin = msg.last_static_safety_margin;
    }
    else {
      resolved.last_static_safety_margin = 0.0
    }

    if (msg.last_dynamic_safety_margin !== undefined) {
      resolved.last_dynamic_safety_margin = msg.last_dynamic_safety_margin;
    }
    else {
      resolved.last_dynamic_safety_margin = 0.0
    }

    return resolved;
    }
};

class rl_stateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.static_safety_margin = null;
      this.dynamic_safety_margin = null;
    }
    else {
      if (initObj.hasOwnProperty('static_safety_margin')) {
        this.static_safety_margin = initObj.static_safety_margin
      }
      else {
        this.static_safety_margin = 0.0;
      }
      if (initObj.hasOwnProperty('dynamic_safety_margin')) {
        this.dynamic_safety_margin = initObj.dynamic_safety_margin
      }
      else {
        this.dynamic_safety_margin = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rl_stateResponse
    // Serialize message field [static_safety_margin]
    bufferOffset = _serializer.float32(obj.static_safety_margin, buffer, bufferOffset);
    // Serialize message field [dynamic_safety_margin]
    bufferOffset = _serializer.float32(obj.dynamic_safety_margin, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rl_stateResponse
    let len;
    let data = new rl_stateResponse(null);
    // Deserialize message field [static_safety_margin]
    data.static_safety_margin = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dynamic_safety_margin]
    data.dynamic_safety_margin = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rl_planner/rl_stateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '69c674084bab219ec028bde8f55acfab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response 部分
    float32 static_safety_margin
    float32 dynamic_safety_margin
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rl_stateResponse(null);
    if (msg.static_safety_margin !== undefined) {
      resolved.static_safety_margin = msg.static_safety_margin;
    }
    else {
      resolved.static_safety_margin = 0.0
    }

    if (msg.dynamic_safety_margin !== undefined) {
      resolved.dynamic_safety_margin = msg.dynamic_safety_margin;
    }
    else {
      resolved.dynamic_safety_margin = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: rl_stateRequest,
  Response: rl_stateResponse,
  md5sum() { return 'e0541c399b3508e509e9e93d705eab77'; },
  datatype() { return 'rl_planner/rl_state'; }
};
