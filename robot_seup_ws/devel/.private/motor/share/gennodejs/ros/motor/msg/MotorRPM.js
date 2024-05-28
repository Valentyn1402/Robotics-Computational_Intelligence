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

class MotorRPM {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rpm_left = null;
      this.rpm_right = null;
    }
    else {
      if (initObj.hasOwnProperty('rpm_left')) {
        this.rpm_left = initObj.rpm_left
      }
      else {
        this.rpm_left = 0.0;
      }
      if (initObj.hasOwnProperty('rpm_right')) {
        this.rpm_right = initObj.rpm_right
      }
      else {
        this.rpm_right = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorRPM
    // Serialize message field [rpm_left]
    bufferOffset = _serializer.float32(obj.rpm_left, buffer, bufferOffset);
    // Serialize message field [rpm_right]
    bufferOffset = _serializer.float32(obj.rpm_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorRPM
    let len;
    let data = new MotorRPM(null);
    // Deserialize message field [rpm_left]
    data.rpm_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rpm_right]
    data.rpm_right = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'motor/MotorRPM';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '811f89cd5e54718d6b2167dcea919c32';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Represents the RPM value of the left and right motors 0...1
    float32 rpm_left
    float32 rpm_right
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorRPM(null);
    if (msg.rpm_left !== undefined) {
      resolved.rpm_left = msg.rpm_left;
    }
    else {
      resolved.rpm_left = 0.0
    }

    if (msg.rpm_right !== undefined) {
      resolved.rpm_right = msg.rpm_right;
    }
    else {
      resolved.rpm_right = 0.0
    }

    return resolved;
    }
};

module.exports = MotorRPM;
