// Auto-generated. Do not edit!

// (in-package stepper_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Stepper_Target {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position_steps = null;
      this.top_speed_steps_per_second = null;
    }
    else {
      if (initObj.hasOwnProperty('position_steps')) {
        this.position_steps = initObj.position_steps
      }
      else {
        this.position_steps = 0;
      }
      if (initObj.hasOwnProperty('top_speed_steps_per_second')) {
        this.top_speed_steps_per_second = initObj.top_speed_steps_per_second
      }
      else {
        this.top_speed_steps_per_second = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Stepper_Target
    // Serialize message field [position_steps]
    bufferOffset = _serializer.uint32(obj.position_steps, buffer, bufferOffset);
    // Serialize message field [top_speed_steps_per_second]
    bufferOffset = _serializer.uint32(obj.top_speed_steps_per_second, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Stepper_Target
    let len;
    let data = new Stepper_Target(null);
    // Deserialize message field [position_steps]
    data.position_steps = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [top_speed_steps_per_second]
    data.top_speed_steps_per_second = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'stepper_msg/Stepper_Target';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '737f4dc6071f1d903de2da2901e5c3ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 position_steps
    uint32 top_speed_steps_per_second
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Stepper_Target(null);
    if (msg.position_steps !== undefined) {
      resolved.position_steps = msg.position_steps;
    }
    else {
      resolved.position_steps = 0
    }

    if (msg.top_speed_steps_per_second !== undefined) {
      resolved.top_speed_steps_per_second = msg.top_speed_steps_per_second;
    }
    else {
      resolved.top_speed_steps_per_second = 0
    }

    return resolved;
    }
};

module.exports = Stepper_Target;
