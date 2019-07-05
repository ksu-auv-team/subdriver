// Auto-generated. Do not edit!

// (in-package message_test.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.scores = null;
      this.boxes = null;
      this.classes = null;
      this.detected = null;
    }
    else {
      if (initObj.hasOwnProperty('scores')) {
        this.scores = initObj.scores
      }
      else {
        this.scores = [];
      }
      if (initObj.hasOwnProperty('boxes')) {
        this.boxes = initObj.boxes
      }
      else {
        this.boxes = [];
      }
      if (initObj.hasOwnProperty('classes')) {
        this.classes = initObj.classes
      }
      else {
        this.classes = [];
      }
      if (initObj.hasOwnProperty('detected')) {
        this.detected = initObj.detected
      }
      else {
        this.detected = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Info
    // Serialize message field [scores]
    bufferOffset = _arraySerializer.int32(obj.scores, buffer, bufferOffset, null);
    // Serialize message field [boxes]
    bufferOffset = _arraySerializer.int32(obj.boxes, buffer, bufferOffset, null);
    // Serialize message field [classes]
    bufferOffset = _arraySerializer.int32(obj.classes, buffer, bufferOffset, null);
    // Serialize message field [detected]
    bufferOffset = _arraySerializer.int32(obj.detected, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Info
    let len;
    let data = new Info(null);
    // Deserialize message field [scores]
    data.scores = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [boxes]
    data.boxes = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [classes]
    data.classes = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [detected]
    data.detected = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.scores.length;
    length += 4 * object.boxes.length;
    length += 4 * object.classes.length;
    length += 4 * object.detected.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'message_test/Info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c873532b9ae33423f8d1b3d0ddf4f322';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] scores
    int32[] boxes
    int32[] classes
    int32[] detected
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Info(null);
    if (msg.scores !== undefined) {
      resolved.scores = msg.scores;
    }
    else {
      resolved.scores = []
    }

    if (msg.boxes !== undefined) {
      resolved.boxes = msg.boxes;
    }
    else {
      resolved.boxes = []
    }

    if (msg.classes !== undefined) {
      resolved.classes = msg.classes;
    }
    else {
      resolved.classes = []
    }

    if (msg.detected !== undefined) {
      resolved.detected = msg.detected;
    }
    else {
      resolved.detected = []
    }

    return resolved;
    }
};

module.exports = Info;
