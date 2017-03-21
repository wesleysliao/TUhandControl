// Auto-generated. Do not edit!

// (in-package adc_joystick_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ADC_Joystick {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.select = null;
      this.x_axis_raw = null;
      this.y_axis_raw = null;
    }
    else {
      if (initObj.hasOwnProperty('select')) {
        this.select = initObj.select
      }
      else {
        this.select = false;
      }
      if (initObj.hasOwnProperty('x_axis_raw')) {
        this.x_axis_raw = initObj.x_axis_raw
      }
      else {
        this.x_axis_raw = 0;
      }
      if (initObj.hasOwnProperty('y_axis_raw')) {
        this.y_axis_raw = initObj.y_axis_raw
      }
      else {
        this.y_axis_raw = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ADC_Joystick
    // Serialize message field [select]
    bufferOffset = _serializer.bool(obj.select, buffer, bufferOffset);
    // Serialize message field [x_axis_raw]
    bufferOffset = _serializer.uint16(obj.x_axis_raw, buffer, bufferOffset);
    // Serialize message field [y_axis_raw]
    bufferOffset = _serializer.uint16(obj.y_axis_raw, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ADC_Joystick
    let len;
    let data = new ADC_Joystick(null);
    // Deserialize message field [select]
    data.select = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [x_axis_raw]
    data.x_axis_raw = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [y_axis_raw]
    data.y_axis_raw = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'adc_joystick_msg/ADC_Joystick';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '832227e0bdded7eb16fd560e3102fbf2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool select
    uint16 x_axis_raw
    uint16 y_axis_raw
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ADC_Joystick(null);
    if (msg.select !== undefined) {
      resolved.select = msg.select;
    }
    else {
      resolved.select = false
    }

    if (msg.x_axis_raw !== undefined) {
      resolved.x_axis_raw = msg.x_axis_raw;
    }
    else {
      resolved.x_axis_raw = 0
    }

    if (msg.y_axis_raw !== undefined) {
      resolved.y_axis_raw = msg.y_axis_raw;
    }
    else {
      resolved.y_axis_raw = 0
    }

    return resolved;
    }
};

module.exports = ADC_Joystick;
