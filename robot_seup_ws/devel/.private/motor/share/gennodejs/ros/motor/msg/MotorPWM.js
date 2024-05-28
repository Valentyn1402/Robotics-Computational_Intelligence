// Auto-generated. Do not edit!

// (in-package motor.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorPWM {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pwm_left = null;
      this.pwm_right = null;
    }
    else {
      if (initObj.hasOwnProperty('pwm_left')) {
        this.pwm_left = initObj.pwm_left
      }
      else {
        this.pwm_left = 0.0;
      }
      if (initObj.hasOwnProperty('pwm_right')) {
        this.pwm_right = initObj.pwm_right
      }
      else {
        this.pwm_right = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorPWM
    // Serialize message field [pwm_left]
    bufferOffset = _serializer.float32(obj.pwm_left, buffer, bufferOffset);
    // Serialize message field [pwm_right]
    bufferOffset = _serializer.float32(obj.pwm_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorPWM
    let len;
    let data = new MotorPWM(null);
    // Deserialize message field [pwm_left]
    data.pwm_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pwm_right]
    data.pwm_right = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'motor/MotorPWM';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd7a5f6b78fec2b5366e39d282d33bb64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Represents the PWM value of the left and right motors 0...1
    float32 pwm_left
    float32 pwm_right
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorPWM(null);
    if (msg.pwm_left !== undefined) {
      resolved.pwm_left = msg.pwm_left;
    }
    else {
      resolved.pwm_left = 0.0
    }

    if (msg.pwm_right !== undefined) {
      resolved.pwm_right = msg.pwm_right;
    }
    else {
      resolved.pwm_right = 0.0
    }

    return resolved;
    }
};

module.exports = MotorPWM;
