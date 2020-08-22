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

class JoyControlAxes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.axes = null;
    }
    else {
      if (initObj.hasOwnProperty('axes')) {
        this.axes = initObj.axes
      }
      else {
        this.axes = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JoyControlAxes
    // Check that the constant length array field [axes] has the right length
    if (obj.axes.length !== 4) {
      throw new Error('Unable to serialize array field axes - length must be 4')
    }
    // Serialize message field [axes]
    bufferOffset = _arraySerializer.uint16(obj.axes, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JoyControlAxes
    let len;
    let data = new JoyControlAxes(null);
    // Deserialize message field [axes]
    data.axes = _arrayDeserializer.uint16(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone/JoyControlAxes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c1b78b75a49603d998b276567ad61bb1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # represents the channels that the FC accepts
    # [Roll, Pitch, Throttle, Yaw]
    uint16[4] axes
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JoyControlAxes(null);
    if (msg.axes !== undefined) {
      resolved.axes = msg.axes;
    }
    else {
      resolved.axes = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = JoyControlAxes;
