// Auto-generated. Do not edit!

// (in-package ucar_qr_decoder.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ZBarDecodeResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.job_id = null;
      this.decoded = null;
      this.decode_ms = null;
      this.stage = null;
    }
    else {
      if (initObj.hasOwnProperty('job_id')) {
        this.job_id = initObj.job_id
      }
      else {
        this.job_id = 0;
      }
      if (initObj.hasOwnProperty('decoded')) {
        this.decoded = initObj.decoded
      }
      else {
        this.decoded = [];
      }
      if (initObj.hasOwnProperty('decode_ms')) {
        this.decode_ms = initObj.decode_ms
      }
      else {
        this.decode_ms = 0.0;
      }
      if (initObj.hasOwnProperty('stage')) {
        this.stage = initObj.stage
      }
      else {
        this.stage = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ZBarDecodeResult
    // Serialize message field [job_id]
    bufferOffset = _serializer.uint64(obj.job_id, buffer, bufferOffset);
    // Serialize message field [decoded]
    bufferOffset = _arraySerializer.string(obj.decoded, buffer, bufferOffset, null);
    // Serialize message field [decode_ms]
    bufferOffset = _serializer.float32(obj.decode_ms, buffer, bufferOffset);
    // Serialize message field [stage]
    bufferOffset = _serializer.string(obj.stage, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ZBarDecodeResult
    let len;
    let data = new ZBarDecodeResult(null);
    // Deserialize message field [job_id]
    data.job_id = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [decoded]
    data.decoded = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [decode_ms]
    data.decode_ms = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [stage]
    data.stage = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.decoded.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += _getByteLength(object.stage);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ucar_qr_decoder/ZBarDecodeResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41ac4872f56934370169b3f1087ab96f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint64 job_id
    string[] decoded
    float32 decode_ms
    string stage
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ZBarDecodeResult(null);
    if (msg.job_id !== undefined) {
      resolved.job_id = msg.job_id;
    }
    else {
      resolved.job_id = 0
    }

    if (msg.decoded !== undefined) {
      resolved.decoded = msg.decoded;
    }
    else {
      resolved.decoded = []
    }

    if (msg.decode_ms !== undefined) {
      resolved.decode_ms = msg.decode_ms;
    }
    else {
      resolved.decode_ms = 0.0
    }

    if (msg.stage !== undefined) {
      resolved.stage = msg.stage;
    }
    else {
      resolved.stage = ''
    }

    return resolved;
    }
};

module.exports = ZBarDecodeResult;
