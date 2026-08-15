
"use strict";

let SetLEDMode = require('./SetLEDMode.js')
let SetSensorTF = require('./SetSensorTF.js')
let SetMaxVel = require('./SetMaxVel.js')
let GetMaxVel = require('./GetMaxVel.js')
let GetBatteryInfo = require('./GetBatteryInfo.js')
let GetSensorTF = require('./GetSensorTF.js')

module.exports = {
  SetLEDMode: SetLEDMode,
  SetSensorTF: SetSensorTF,
  SetMaxVel: SetMaxVel,
  GetMaxVel: GetMaxVel,
  GetBatteryInfo: GetBatteryInfo,
  GetSensorTF: GetSensorTF,
};
