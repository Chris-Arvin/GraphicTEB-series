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

class GetAllAgentsStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.agent_ids = null;
    }
    else {
      if (initObj.hasOwnProperty('agent_ids')) {
        this.agent_ids = initObj.agent_ids
      }
      else {
        this.agent_ids = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAllAgentsStateRequest
    // Serialize message field [agent_ids]
    bufferOffset = _arraySerializer.int16(obj.agent_ids, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllAgentsStateRequest
    let len;
    let data = new GetAllAgentsStateRequest(null);
    // Deserialize message field [agent_ids]
    data.agent_ids = _arrayDeserializer.int16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.agent_ids.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pedsim_srvs/GetAllAgentsStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6ae2c87e95de54a27053702cd22ee2c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16[] agent_ids
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAllAgentsStateRequest(null);
    if (msg.agent_ids !== undefined) {
      resolved.agent_ids = msg.agent_ids;
    }
    else {
      resolved.agent_ids = []
    }

    return resolved;
    }
};

class GetAllAgentsStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.agent_states = null;
    }
    else {
      if (initObj.hasOwnProperty('agent_states')) {
        this.agent_states = initObj.agent_states
      }
      else {
        this.agent_states = new pedsim_msgs.msg.AgentStates();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAllAgentsStateResponse
    // Serialize message field [agent_states]
    bufferOffset = pedsim_msgs.msg.AgentStates.serialize(obj.agent_states, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllAgentsStateResponse
    let len;
    let data = new GetAllAgentsStateResponse(null);
    // Deserialize message field [agent_states]
    data.agent_states = pedsim_msgs.msg.AgentStates.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += pedsim_msgs.msg.AgentStates.getMessageSize(object.agent_states);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pedsim_srvs/GetAllAgentsStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '24869d0a05b36ba71cbe2bf6556f2fc9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    pedsim_msgs/AgentStates agent_states
    
    
    ================================================================================
    MSG: pedsim_msgs/AgentStates
    Header header
    pedsim_msgs/AgentState[] agent_states
    
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
    const resolved = new GetAllAgentsStateResponse(null);
    if (msg.agent_states !== undefined) {
      resolved.agent_states = pedsim_msgs.msg.AgentStates.Resolve(msg.agent_states)
    }
    else {
      resolved.agent_states = new pedsim_msgs.msg.AgentStates()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAllAgentsStateRequest,
  Response: GetAllAgentsStateResponse,
  md5sum() { return '03ab07dfda7ff01d05feae6d271a6b89'; },
  datatype() { return 'pedsim_srvs/GetAllAgentsState'; }
};
