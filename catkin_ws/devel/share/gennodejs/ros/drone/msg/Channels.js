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

class Channels {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.channel = null;
    }
    else {
      if (initObj.hasOwnProperty('channel')) {
        this.channel = initObj.channel
      }
      else {
        this.channel = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Channels
    // Check that the constant length array field [channel] has the right length
    if (obj.channel.length !== 6) {
      throw new Error('Unable to serialize array field channel - length must be 6')
    }
    // Serialize message field [channel]
    bufferOffset = _arraySerializer.uint16(obj.channel, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Channels
    let len;
    let data = new Channels(null);
    // Deserialize message field [channel]
    data.channel = _arrayDeserializer.uint16(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone/Channels';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '31b508c75ec65d5aaa1d5c71970654be';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # represents the channels that the FC accepts
    # [Roll, Pitch, Throttle, Yaw, AUX1, AUX2]
    uint16[6] channel
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Channels(null);
    if (msg.channel !== undefined) {
      resolved.channel = msg.channel;
    }
    else {
      resolved.channel = new Array(6).fill(0)
    }

    return resolved;
    }
};

module.exports = Channels;
