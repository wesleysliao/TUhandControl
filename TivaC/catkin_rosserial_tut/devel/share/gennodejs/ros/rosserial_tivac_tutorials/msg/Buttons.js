// Auto-generated. Do not edit!

// (in-package rosserial_tivac_tutorials.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Buttons {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sw1 = null;
      this.sw2 = null;
    }
    else {
      if (initObj.hasOwnProperty('sw1')) {
        this.sw1 = initObj.sw1
      }
      else {
        this.sw1 = new std_msgs.msg.Bool();
      }
      if (initObj.hasOwnProperty('sw2')) {
        this.sw2 = initObj.sw2
      }
      else {
        this.sw2 = new std_msgs.msg.Bool();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Buttons
    // Serialize message field [sw1]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.sw1, buffer, bufferOffset);
    // Serialize message field [sw2]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.sw2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Buttons
    let len;
    let data = new Buttons(null);
    // Deserialize message field [sw1]
    data.sw1 = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    // Deserialize message field [sw2]
    data.sw2 = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rosserial_tivac_tutorials/Buttons';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a78ccaade8fa723d1ebeb7b099042085';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Bool sw1
    std_msgs/Bool sw2
    
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Buttons(null);
    if (msg.sw1 !== undefined) {
      resolved.sw1 = std_msgs.msg.Bool.Resolve(msg.sw1)
    }
    else {
      resolved.sw1 = new std_msgs.msg.Bool()
    }

    if (msg.sw2 !== undefined) {
      resolved.sw2 = std_msgs.msg.Bool.Resolve(msg.sw2)
    }
    else {
      resolved.sw2 = new std_msgs.msg.Bool()
    }

    return resolved;
    }
};

module.exports = Buttons;
