#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Trigger simulation only after the real car parks at the sim workshop."""

import json
import threading

import rospy

from xunfei2026_room_delivery_manager_v1 import canonical_workshop
from xunfei2026_simulation_handoff_v1 import SimulationHandoff


class SecondParkingSimulationHandoff(SimulationHandoff):
    def __init__(self):
        self.second_parking_payload = None
        super(SecondParkingSimulationHandoff, self).__init__()
        self.publish_state(
            "WAITING_SECOND_TARGET_PARKING",
            trigger_state="SIM_TARGET_PARKED",
            first_delivery_tts_triggers_simulation=False)

    def result_callback(self, msg):
        super(SecondParkingSimulationHandoff, self).result_callback(msg)
        self.try_start_second_parking_handoff()

    def real_status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if (payload.get("state") != "SIM_TARGET_PARKED" or
                not bool(payload.get("parking_success", False)) or
                not bool(payload.get("simulation_trigger_authorized", False))):
            return
        with self.lock:
            self.second_parking_payload = payload
        self.try_start_second_parking_handoff()

    def try_start_second_parking_handoff(self):
        with self.lock:
            if self.handoff_started or self.second_parking_payload is None:
                return
            payload = dict(self.second_parking_payload)
            required_order = (
                self.sim_target_key, self.sim_selected_item,
                self.sim_target_category, self.sim_target_warehouse,
                self.sim_mission_id)
            if not all(required_order):
                self.publish_state(
                    "SIMULATION_TRIGGER_WAITING_ORDER",
                    reason="simulation order is incomplete")
                return
            parked_warehouse = canonical_workshop(
                payload.get("sim_warehouse", payload.get("warehouse", "")))
            expected_warehouse = canonical_workshop(self.sim_target_warehouse)
            parked_item = str(payload.get("sim_item", "")).strip()
            if (parked_warehouse != expected_warehouse or
                    parked_item != self.sim_selected_item):
                self.publish_state(
                    "SIMULATION_TRIGGER_REJECTED_SECOND_TARGET_MISMATCH",
                    parked_item=parked_item,
                    expected_item=self.sim_selected_item,
                    parked_warehouse=parked_warehouse,
                    expected_warehouse=expected_warehouse)
                return
            self.handoff_started = True
        self.publish_state(
            "SIMULATION_TRIGGERED_BY_SECOND_PARKING",
            parked_item=parked_item,
            parked_warehouse=parked_warehouse,
            parking_result_required=True)
        worker = threading.Thread(target=self.start_simulation_handoff)
        worker.daemon = True
        worker.start()


if __name__ == "__main__":
    SecondParkingSimulationHandoff()
    rospy.spin()
