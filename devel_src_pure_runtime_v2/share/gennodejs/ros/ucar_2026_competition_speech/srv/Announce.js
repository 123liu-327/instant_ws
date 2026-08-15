// Auto-generated. Do not edit!

// (in-package ucar_2026_competition_speech.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class AnnounceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.event = null;
      this.item = null;
      this.workshop = null;
      this.decision = null;
      this.text = null;
      this.wait = null;
    }
    else {
      if (initObj.hasOwnProperty('event')) {
        this.event = initObj.event
      }
      else {
        this.event = '';
      }
      if (initObj.hasOwnProperty('item')) {
        this.item = initObj.item
      }
      else {
        this.item = '';
      }
      if (initObj.hasOwnProperty('workshop')) {
        this.workshop = initObj.workshop
      }
      else {
        this.workshop = '';
      }
      if (initObj.hasOwnProperty('decision')) {
        this.decision = initObj.decision
      }
      else {
        this.decision = '';
      }
      if (initObj.hasOwnProperty('text')) {
        this.text = initObj.text
      }
      else {
        this.text = '';
      }
      if (initObj.hasOwnProperty('wait')) {
        this.wait = initObj.wait
      }
      else {
        this.wait = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnnounceRequest
    // Serialize message field [event]
    bufferOffset = _serializer.string(obj.event, buffer, bufferOffset);
    // Serialize message field [item]
    bufferOffset = _serializer.string(obj.item, buffer, bufferOffset);
    // Serialize message field [workshop]
    bufferOffset = _serializer.string(obj.workshop, buffer, bufferOffset);
    // Serialize message field [decision]
    bufferOffset = _serializer.string(obj.decision, buffer, bufferOffset);
    // Serialize message field [text]
    bufferOffset = _serializer.string(obj.text, buffer, bufferOffset);
    // Serialize message field [wait]
    bufferOffset = _serializer.bool(obj.wait, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnnounceRequest
    let len;
    let data = new AnnounceRequest(null);
    // Deserialize message field [event]
    data.event = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [item]
    data.item = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [workshop]
    data.workshop = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [decision]
    data.decision = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [text]
    data.text = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [wait]
    data.wait = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.event);
    length += _getByteLength(object.item);
    length += _getByteLength(object.workshop);
    length += _getByteLength(object.decision);
    length += _getByteLength(object.text);
    return length + 21;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ucar_2026_competition_speech/AnnounceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c5c01b62f0b7bc5ddbf1469bdf6b9ea0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Competition event: task1, task2, task3, task4, task5, or custom.
    string event
    # Used by task2 and task3.
    string item
    string workshop
    # Used by task4. Accepted aliases include left/right/straight/stop and Chinese names.
    string decision
    # Required by task1 and custom. Ignored by the fixed task2-task5 templates.
    string text
    # Wait until the conservative estimated playback duration has elapsed.
    bool wait
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AnnounceRequest(null);
    if (msg.event !== undefined) {
      resolved.event = msg.event;
    }
    else {
      resolved.event = ''
    }

    if (msg.item !== undefined) {
      resolved.item = msg.item;
    }
    else {
      resolved.item = ''
    }

    if (msg.workshop !== undefined) {
      resolved.workshop = msg.workshop;
    }
    else {
      resolved.workshop = ''
    }

    if (msg.decision !== undefined) {
      resolved.decision = msg.decision;
    }
    else {
      resolved.decision = ''
    }

    if (msg.text !== undefined) {
      resolved.text = msg.text;
    }
    else {
      resolved.text = ''
    }

    if (msg.wait !== undefined) {
      resolved.wait = msg.wait;
    }
    else {
      resolved.wait = false
    }

    return resolved;
    }
};

class AnnounceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.speech_text = null;
      this.estimated_duration = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('speech_text')) {
        this.speech_text = initObj.speech_text
      }
      else {
        this.speech_text = '';
      }
      if (initObj.hasOwnProperty('estimated_duration')) {
        this.estimated_duration = initObj.estimated_duration
      }
      else {
        this.estimated_duration = 0.0;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnnounceResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [speech_text]
    bufferOffset = _serializer.string(obj.speech_text, buffer, bufferOffset);
    // Serialize message field [estimated_duration]
    bufferOffset = _serializer.float32(obj.estimated_duration, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnnounceResponse
    let len;
    let data = new AnnounceResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [speech_text]
    data.speech_text = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [estimated_duration]
    data.estimated_duration = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.speech_text);
    length += _getByteLength(object.message);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ucar_2026_competition_speech/AnnounceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd69777e02a5c8cab2cb57cba1ade6ebe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string speech_text
    float32 estimated_duration
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AnnounceResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.speech_text !== undefined) {
      resolved.speech_text = msg.speech_text;
    }
    else {
      resolved.speech_text = ''
    }

    if (msg.estimated_duration !== undefined) {
      resolved.estimated_duration = msg.estimated_duration;
    }
    else {
      resolved.estimated_duration = 0.0
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: AnnounceRequest,
  Response: AnnounceResponse,
  md5sum() { return 'f3261cee1e2a84f216672d4d9f69791a'; },
  datatype() { return 'ucar_2026_competition_speech/Announce'; }
};
