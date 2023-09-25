// Auto-generated. Do not edit!

// (in-package hw0.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class motor_topic {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rotation = null;
      this.clockwise = null;
    }
    else {
      if (initObj.hasOwnProperty('rotation')) {
        this.rotation = initObj.rotation
      }
      else {
        this.rotation = 0;
      }
      if (initObj.hasOwnProperty('clockwise')) {
        this.clockwise = initObj.clockwise
      }
      else {
        this.clockwise = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type motor_topic
    // Serialize message field [rotation]
    bufferOffset = _serializer.int64(obj.rotation, buffer, bufferOffset);
    // Serialize message field [clockwise]
    bufferOffset = _serializer.bool(obj.clockwise, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type motor_topic
    let len;
    let data = new motor_topic(null);
    // Deserialize message field [rotation]
    data.rotation = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [clockwise]
    data.clockwise = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hw0/motor_topic';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3f09a113fbdc065e6685b77041855ae5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 rotation
    bool clockwise
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new motor_topic(null);
    if (msg.rotation !== undefined) {
      resolved.rotation = msg.rotation;
    }
    else {
      resolved.rotation = 0
    }

    if (msg.clockwise !== undefined) {
      resolved.clockwise = msg.clockwise;
    }
    else {
      resolved.clockwise = false
    }

    return resolved;
    }
};

module.exports = motor_topic;
