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

class SonarAltitude {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ground_distance = null;
    }
    else {
      if (initObj.hasOwnProperty('ground_distance')) {
        this.ground_distance = initObj.ground_distance
      }
      else {
        this.ground_distance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SonarAltitude
    // Serialize message field [ground_distance]
    bufferOffset = _serializer.float32(obj.ground_distance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SonarAltitude
    let len;
    let data = new SonarAltitude(null);
    // Deserialize message field [ground_distance]
    data.ground_distance = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone/SonarAltitude';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8f146d4e34a63c94036de552a9e31ac8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # from sonar
    float32 ground_distance
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SonarAltitude(null);
    if (msg.ground_distance !== undefined) {
      resolved.ground_distance = msg.ground_distance;
    }
    else {
      resolved.ground_distance = 0.0
    }

    return resolved;
    }
};

module.exports = SonarAltitude;
