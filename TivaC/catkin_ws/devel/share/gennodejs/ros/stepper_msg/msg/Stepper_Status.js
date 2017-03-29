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

class Stepper_Status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position_steps = null;
      this.speed_steps_per_second = null;
      this.direction_forward = null;
      this.enabled = null;
      this.errors = null;
    }
    else {
      if (initObj.hasOwnProperty('position_steps')) {
        this.position_steps = initObj.position_steps
      }
      else {
        this.position_steps = 0;
      }
      if (initObj.hasOwnProperty('speed_steps_per_second')) {
        this.speed_steps_per_second = initObj.speed_steps_per_second
      }
      else {
        this.speed_steps_per_second = 0;
      }
      if (initObj.hasOwnProperty('direction_forward')) {
        this.direction_forward = initObj.direction_forward
      }
      else {
        this.direction_forward = false;
      }
      if (initObj.hasOwnProperty('enabled')) {
        this.enabled = initObj.enabled
      }
      else {
        this.enabled = false;
      }
      if (initObj.hasOwnProperty('errors')) {
        this.errors = initObj.errors
      }
      else {
        this.errors = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Stepper_Status
    // Serialize message field [position_steps]
    bufferOffset = _serializer.int64(obj.position_steps, buffer, bufferOffset);
    // Serialize message field [speed_steps_per_second]
    bufferOffset = _serializer.int32(obj.speed_steps_per_second, buffer, bufferOffset);
    // Serialize message field [direction_forward]
    bufferOffset = _serializer.bool(obj.direction_forward, buffer, bufferOffset);
    // Serialize message field [enabled]
    bufferOffset = _serializer.bool(obj.enabled, buffer, bufferOffset);
    // Serialize message field [errors]
    bufferOffset = _serializer.string(obj.errors, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Stepper_Status
    let len;
    let data = new Stepper_Status(null);
    // Deserialize message field [position_steps]
    data.position_steps = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [speed_steps_per_second]
    data.speed_steps_per_second = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [direction_forward]
    data.direction_forward = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [enabled]
    data.enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [errors]
    data.errors = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.errors.length;
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'stepper_msg/Stepper_Status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8b977b7753d25b4757850d088d3939b1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 position_steps
    int32 speed_steps_per_second
    bool direction_forward
    bool enabled
    string errors
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Stepper_Status(null);
    if (msg.position_steps !== undefined) {
      resolved.position_steps = msg.position_steps;
    }
    else {
      resolved.position_steps = 0
    }

    if (msg.speed_steps_per_second !== undefined) {
      resolved.speed_steps_per_second = msg.speed_steps_per_second;
    }
    else {
      resolved.speed_steps_per_second = 0
    }

    if (msg.direction_forward !== undefined) {
      resolved.direction_forward = msg.direction_forward;
    }
    else {
      resolved.direction_forward = false
    }

    if (msg.enabled !== undefined) {
      resolved.enabled = msg.enabled;
    }
    else {
      resolved.enabled = false
    }

    if (msg.errors !== undefined) {
      resolved.errors = msg.errors;
    }
    else {
      resolved.errors = ''
    }

    return resolved;
    }
};

module.exports = Stepper_Status;
