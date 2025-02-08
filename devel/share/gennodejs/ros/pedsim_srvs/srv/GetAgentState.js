// Auto-generated. Do not edit!

// (in-package pedsim_srvs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let pedsim_msgs = _finder('pedsim_msgs');

//-----------------------------------------------------------

class GetAgentStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.agent_id = null;
    }
    else {
      if (initObj.hasOwnProperty('agent_id')) {
        this.agent_id = initObj.agent_id
      }
      else {
        this.agent_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAgentStateRequest
    // Serialize message field [agent_id]
    bufferOffset = _serializer.int16(obj.agent_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAgentStateRequest
    let len;
    let data = new GetAgentStateRequest(null);
    // Deserialize message field [agent_id]
    data.agent_id = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pedsim_srvs/GetAgentStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0c54969886c310134f80d7787e2397f3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 agent_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAgentStateRequest(null);
    if (msg.agent_id !== undefined) {
      resolved.agent_id = msg.agent_id;
    }
    else {
      resolved.agent_id = 0
    }

    return resolved;
    }
};

class GetAgentStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = new pedsim_msgs.msg.AgentState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAgentStateResponse
    // Serialize message field [state]
    bufferOffset = pedsim_msgs.msg.AgentState.serialize(obj.state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAgentStateResponse
    let len;
    let data = new GetAgentStateResponse(null);
    // Deserialize message field [state]
    data.state = pedsim_msgs.msg.AgentState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += pedsim_msgs.msg.AgentState.getMessageSize(object.state);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pedsim_srvs/GetAgentStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b3e40c6ab2052e0c1b449d6d78af2454';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    pedsim_msgs/AgentState state
    
    ================================================================================
    MSG: pedsim_msgs/AgentState
    Header header
    uint64 id
    uint16 type
    string social_state
    geometry_msgs/Pose pose
    geometry_msgs/Twist twist
    pedsim_msgs/AgentForce forces
    
    # Use sensors package to control observability
    
    # Social State string constants
    string      TYPE_STANDING = "standing"
    string      TYPE_INDIVIDUAL_MOVING = "individual_moving"
    string      TYPE_WAITING_IN_QUEUE = "waiting_in_queue"
    string      TYPE_GROUP_MOVING = "group_moving"
    
    
    # Agent types
    # 0, 1 -> ordinary agents
    # 2 -> Robot
    # 3 -> standing/elderly agents
    
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
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
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
    MSG: pedsim_msgs/AgentForce
    # Forces acting on an agent.
    
    # Basic SFM forces.
    geometry_msgs/Vector3 desired_force
    geometry_msgs/Vector3 obstacle_force
    geometry_msgs/Vector3 social_force
    
    # Additional Group Forces
    geometry_msgs/Vector3 group_coherence_force
    geometry_msgs/Vector3 group_gaze_force
    geometry_msgs/Vector3 group_repulsion_force
    
    # Extra stabilization/custom forces.
    geometry_msgs/Vector3 random_force
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAgentStateResponse(null);
    if (msg.state !== undefined) {
      resolved.state = pedsim_msgs.msg.AgentState.Resolve(msg.state)
    }
    else {
      resolved.state = new pedsim_msgs.msg.AgentState()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAgentStateRequest,
  Response: GetAgentStateResponse,
  md5sum() { return '506aed4cf0fa361a55600b1ac6b1f978'; },
  datatype() { return 'pedsim_srvs/GetAgentState'; }
};
