// Auto-generated. Do not edit!

// (in-package line_follower.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class LineTrack {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.valid = null;
      this.confidence = null;
      this.image_width = null;
      this.image_height = null;
      this.center_x_px = null;
      this.center_y_px = null;
      this.lookahead_x_px = null;
      this.lookahead_y_px = null;
      this.heading_error_rad = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('valid')) {
        this.valid = initObj.valid
      }
      else {
        this.valid = false;
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('image_width')) {
        this.image_width = initObj.image_width
      }
      else {
        this.image_width = 0;
      }
      if (initObj.hasOwnProperty('image_height')) {
        this.image_height = initObj.image_height
      }
      else {
        this.image_height = 0;
      }
      if (initObj.hasOwnProperty('center_x_px')) {
        this.center_x_px = initObj.center_x_px
      }
      else {
        this.center_x_px = [];
      }
      if (initObj.hasOwnProperty('center_y_px')) {
        this.center_y_px = initObj.center_y_px
      }
      else {
        this.center_y_px = [];
      }
      if (initObj.hasOwnProperty('lookahead_x_px')) {
        this.lookahead_x_px = initObj.lookahead_x_px
      }
      else {
        this.lookahead_x_px = 0.0;
      }
      if (initObj.hasOwnProperty('lookahead_y_px')) {
        this.lookahead_y_px = initObj.lookahead_y_px
      }
      else {
        this.lookahead_y_px = 0.0;
      }
      if (initObj.hasOwnProperty('heading_error_rad')) {
        this.heading_error_rad = initObj.heading_error_rad
      }
      else {
        this.heading_error_rad = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LineTrack
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _serializer.bool(obj.valid, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [image_width]
    bufferOffset = _serializer.uint32(obj.image_width, buffer, bufferOffset);
    // Serialize message field [image_height]
    bufferOffset = _serializer.uint32(obj.image_height, buffer, bufferOffset);
    // Serialize message field [center_x_px]
    bufferOffset = _arraySerializer.float32(obj.center_x_px, buffer, bufferOffset, null);
    // Serialize message field [center_y_px]
    bufferOffset = _arraySerializer.float32(obj.center_y_px, buffer, bufferOffset, null);
    // Serialize message field [lookahead_x_px]
    bufferOffset = _serializer.float32(obj.lookahead_x_px, buffer, bufferOffset);
    // Serialize message field [lookahead_y_px]
    bufferOffset = _serializer.float32(obj.lookahead_y_px, buffer, bufferOffset);
    // Serialize message field [heading_error_rad]
    bufferOffset = _serializer.float32(obj.heading_error_rad, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LineTrack
    let len;
    let data = new LineTrack(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [image_width]
    data.image_width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [image_height]
    data.image_height = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [center_x_px]
    data.center_x_px = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [center_y_px]
    data.center_y_px = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [lookahead_x_px]
    data.lookahead_x_px = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lookahead_y_px]
    data.lookahead_y_px = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_error_rad]
    data.heading_error_rad = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.center_x_px.length;
    length += 4 * object.center_y_px.length;
    return length + 33;
  }

  static datatype() {
    // Returns string type for a message object
    return 'line_follower/LineTrack';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8f2dc59e6af2e2fcb8e42affc92dd20';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool valid
    float32 confidence
    uint32 image_width
    uint32 image_height
    float32[] center_x_px
    float32[] center_y_px
    float32 lookahead_x_px
    float32 lookahead_y_px
    float32 heading_error_rad
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LineTrack(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.valid !== undefined) {
      resolved.valid = msg.valid;
    }
    else {
      resolved.valid = false
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.image_width !== undefined) {
      resolved.image_width = msg.image_width;
    }
    else {
      resolved.image_width = 0
    }

    if (msg.image_height !== undefined) {
      resolved.image_height = msg.image_height;
    }
    else {
      resolved.image_height = 0
    }

    if (msg.center_x_px !== undefined) {
      resolved.center_x_px = msg.center_x_px;
    }
    else {
      resolved.center_x_px = []
    }

    if (msg.center_y_px !== undefined) {
      resolved.center_y_px = msg.center_y_px;
    }
    else {
      resolved.center_y_px = []
    }

    if (msg.lookahead_x_px !== undefined) {
      resolved.lookahead_x_px = msg.lookahead_x_px;
    }
    else {
      resolved.lookahead_x_px = 0.0
    }

    if (msg.lookahead_y_px !== undefined) {
      resolved.lookahead_y_px = msg.lookahead_y_px;
    }
    else {
      resolved.lookahead_y_px = 0.0
    }

    if (msg.heading_error_rad !== undefined) {
      resolved.heading_error_rad = msg.heading_error_rad;
    }
    else {
      resolved.heading_error_rad = 0.0
    }

    return resolved;
    }
};

module.exports = LineTrack;
