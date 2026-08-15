// Auto-generated. Do not edit!

// (in-package ucar_2026_smart_factory_llm.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ReasonPickupOrderRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.item_a = null;
      this.item_b = null;
      this.item_c = null;
      this.voice_instruction = null;
    }
    else {
      if (initObj.hasOwnProperty('item_a')) {
        this.item_a = initObj.item_a
      }
      else {
        this.item_a = '';
      }
      if (initObj.hasOwnProperty('item_b')) {
        this.item_b = initObj.item_b
      }
      else {
        this.item_b = '';
      }
      if (initObj.hasOwnProperty('item_c')) {
        this.item_c = initObj.item_c
      }
      else {
        this.item_c = '';
      }
      if (initObj.hasOwnProperty('voice_instruction')) {
        this.voice_instruction = initObj.voice_instruction
      }
      else {
        this.voice_instruction = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReasonPickupOrderRequest
    // Serialize message field [item_a]
    bufferOffset = _serializer.string(obj.item_a, buffer, bufferOffset);
    // Serialize message field [item_b]
    bufferOffset = _serializer.string(obj.item_b, buffer, bufferOffset);
    // Serialize message field [item_c]
    bufferOffset = _serializer.string(obj.item_c, buffer, bufferOffset);
    // Serialize message field [voice_instruction]
    bufferOffset = _serializer.string(obj.voice_instruction, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReasonPickupOrderRequest
    let len;
    let data = new ReasonPickupOrderRequest(null);
    // Deserialize message field [item_a]
    data.item_a = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [item_b]
    data.item_b = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [item_c]
    data.item_c = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [voice_instruction]
    data.voice_instruction = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.item_a);
    length += _getByteLength(object.item_b);
    length += _getByteLength(object.item_c);
    length += _getByteLength(object.voice_instruction);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ucar_2026_smart_factory_llm/ReasonPickupOrderRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '69a5ea5bde9f7428407b0f91b89d76ab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 三个二维码解析得到的子类名称（与扫描顺序一致即可）
    string item_a
    string item_b
    string item_c
    # 语音唤醒后的完整指令文本（含目标大类）
    string voice_instruction
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReasonPickupOrderRequest(null);
    if (msg.item_a !== undefined) {
      resolved.item_a = msg.item_a;
    }
    else {
      resolved.item_a = ''
    }

    if (msg.item_b !== undefined) {
      resolved.item_b = msg.item_b;
    }
    else {
      resolved.item_b = ''
    }

    if (msg.item_c !== undefined) {
      resolved.item_c = msg.item_c;
    }
    else {
      resolved.item_c = ''
    }

    if (msg.voice_instruction !== undefined) {
      resolved.voice_instruction = msg.voice_instruction;
    }
    else {
      resolved.voice_instruction = ''
    }

    return resolved;
    }
};

class ReasonPickupOrderResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.error_message = null;
      this.announcement_physical = null;
      this.announcement_simulation = null;
      this.announcement_full = null;
      this.pickup_item = null;
      this.pickup_major = null;
      this.pickup_workshop = null;
      this.sim_item = null;
      this.sim_major = null;
      this.sim_workshop = null;
      this.raw_model_reply = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('error_message')) {
        this.error_message = initObj.error_message
      }
      else {
        this.error_message = '';
      }
      if (initObj.hasOwnProperty('announcement_physical')) {
        this.announcement_physical = initObj.announcement_physical
      }
      else {
        this.announcement_physical = '';
      }
      if (initObj.hasOwnProperty('announcement_simulation')) {
        this.announcement_simulation = initObj.announcement_simulation
      }
      else {
        this.announcement_simulation = '';
      }
      if (initObj.hasOwnProperty('announcement_full')) {
        this.announcement_full = initObj.announcement_full
      }
      else {
        this.announcement_full = '';
      }
      if (initObj.hasOwnProperty('pickup_item')) {
        this.pickup_item = initObj.pickup_item
      }
      else {
        this.pickup_item = '';
      }
      if (initObj.hasOwnProperty('pickup_major')) {
        this.pickup_major = initObj.pickup_major
      }
      else {
        this.pickup_major = '';
      }
      if (initObj.hasOwnProperty('pickup_workshop')) {
        this.pickup_workshop = initObj.pickup_workshop
      }
      else {
        this.pickup_workshop = '';
      }
      if (initObj.hasOwnProperty('sim_item')) {
        this.sim_item = initObj.sim_item
      }
      else {
        this.sim_item = '';
      }
      if (initObj.hasOwnProperty('sim_major')) {
        this.sim_major = initObj.sim_major
      }
      else {
        this.sim_major = '';
      }
      if (initObj.hasOwnProperty('sim_workshop')) {
        this.sim_workshop = initObj.sim_workshop
      }
      else {
        this.sim_workshop = '';
      }
      if (initObj.hasOwnProperty('raw_model_reply')) {
        this.raw_model_reply = initObj.raw_model_reply
      }
      else {
        this.raw_model_reply = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReasonPickupOrderResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [error_message]
    bufferOffset = _serializer.string(obj.error_message, buffer, bufferOffset);
    // Serialize message field [announcement_physical]
    bufferOffset = _serializer.string(obj.announcement_physical, buffer, bufferOffset);
    // Serialize message field [announcement_simulation]
    bufferOffset = _serializer.string(obj.announcement_simulation, buffer, bufferOffset);
    // Serialize message field [announcement_full]
    bufferOffset = _serializer.string(obj.announcement_full, buffer, bufferOffset);
    // Serialize message field [pickup_item]
    bufferOffset = _serializer.string(obj.pickup_item, buffer, bufferOffset);
    // Serialize message field [pickup_major]
    bufferOffset = _serializer.string(obj.pickup_major, buffer, bufferOffset);
    // Serialize message field [pickup_workshop]
    bufferOffset = _serializer.string(obj.pickup_workshop, buffer, bufferOffset);
    // Serialize message field [sim_item]
    bufferOffset = _serializer.string(obj.sim_item, buffer, bufferOffset);
    // Serialize message field [sim_major]
    bufferOffset = _serializer.string(obj.sim_major, buffer, bufferOffset);
    // Serialize message field [sim_workshop]
    bufferOffset = _serializer.string(obj.sim_workshop, buffer, bufferOffset);
    // Serialize message field [raw_model_reply]
    bufferOffset = _serializer.string(obj.raw_model_reply, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReasonPickupOrderResponse
    let len;
    let data = new ReasonPickupOrderResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [error_message]
    data.error_message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [announcement_physical]
    data.announcement_physical = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [announcement_simulation]
    data.announcement_simulation = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [announcement_full]
    data.announcement_full = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pickup_item]
    data.pickup_item = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pickup_major]
    data.pickup_major = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pickup_workshop]
    data.pickup_workshop = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [sim_item]
    data.sim_item = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [sim_major]
    data.sim_major = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [sim_workshop]
    data.sim_workshop = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [raw_model_reply]
    data.raw_model_reply = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.error_message);
    length += _getByteLength(object.announcement_physical);
    length += _getByteLength(object.announcement_simulation);
    length += _getByteLength(object.announcement_full);
    length += _getByteLength(object.pickup_item);
    length += _getByteLength(object.pickup_major);
    length += _getByteLength(object.pickup_workshop);
    length += _getByteLength(object.sim_item);
    length += _getByteLength(object.sim_major);
    length += _getByteLength(object.sim_workshop);
    length += _getByteLength(object.raw_model_reply);
    return length + 45;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ucar_2026_smart_factory_llm/ReasonPickupOrderResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e787d6899f55d92dc505ef388cf2bfb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string error_message
    # 赛方要求的播报格式（可拆成两句给 TTS）
    string announcement_physical
    string announcement_simulation
    string announcement_full
    # 结构化结果，便于下游记录 / 调试
    string pickup_item
    string pickup_major
    string pickup_workshop
    string sim_item
    string sim_major
    string sim_workshop
    string raw_model_reply
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReasonPickupOrderResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.error_message !== undefined) {
      resolved.error_message = msg.error_message;
    }
    else {
      resolved.error_message = ''
    }

    if (msg.announcement_physical !== undefined) {
      resolved.announcement_physical = msg.announcement_physical;
    }
    else {
      resolved.announcement_physical = ''
    }

    if (msg.announcement_simulation !== undefined) {
      resolved.announcement_simulation = msg.announcement_simulation;
    }
    else {
      resolved.announcement_simulation = ''
    }

    if (msg.announcement_full !== undefined) {
      resolved.announcement_full = msg.announcement_full;
    }
    else {
      resolved.announcement_full = ''
    }

    if (msg.pickup_item !== undefined) {
      resolved.pickup_item = msg.pickup_item;
    }
    else {
      resolved.pickup_item = ''
    }

    if (msg.pickup_major !== undefined) {
      resolved.pickup_major = msg.pickup_major;
    }
    else {
      resolved.pickup_major = ''
    }

    if (msg.pickup_workshop !== undefined) {
      resolved.pickup_workshop = msg.pickup_workshop;
    }
    else {
      resolved.pickup_workshop = ''
    }

    if (msg.sim_item !== undefined) {
      resolved.sim_item = msg.sim_item;
    }
    else {
      resolved.sim_item = ''
    }

    if (msg.sim_major !== undefined) {
      resolved.sim_major = msg.sim_major;
    }
    else {
      resolved.sim_major = ''
    }

    if (msg.sim_workshop !== undefined) {
      resolved.sim_workshop = msg.sim_workshop;
    }
    else {
      resolved.sim_workshop = ''
    }

    if (msg.raw_model_reply !== undefined) {
      resolved.raw_model_reply = msg.raw_model_reply;
    }
    else {
      resolved.raw_model_reply = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: ReasonPickupOrderRequest,
  Response: ReasonPickupOrderResponse,
  md5sum() { return '6f2e4969829f8723b74f2ea5cfb7ffc4'; },
  datatype() { return 'ucar_2026_smart_factory_llm/ReasonPickupOrder'; }
};
