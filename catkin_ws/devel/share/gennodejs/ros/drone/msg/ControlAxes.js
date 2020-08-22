// Auto-generated. Do not edit!

// (in-package drone.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ControlAxes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlAxes
    // Check that the constant length array field [data] has the right length
    if (obj.data.length !== 4) {
      throw new Error('Unable to serialize array field data - length must be 4')
    }
    // Serialize message field [data]
    bufferOffset = _arraySerializer.uint16(obj.data, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlAxes
    let len;
    let data = new ControlAxes(null);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.uint16(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone/ControlAxes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '073fbf0245def3a5b07ad0a112777420';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # represents the channels that the FC accepts
    # [Roll, Pitch, Throttle, Yaw]
    uint16[4] data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlAxes(null);
    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = ControlAxes;
