# !/usr/bin/env python3
# -*- coding: utf-8 -*-

from signage.autoware_diagnostic import AutowareDiagnostic
from diagnostic_msgs.msg import DiagnosticStatus

class Heartbeat:
    def __init__(self, node):
        self._node = node

        self._diagnostic_updater = AutowareDiagnostic().init_updater(
            self._node,
            "/system/signage_connection : signage heartbeat",
            self.handle_heartbeat_diagnostics,
            "none",
        )

    def handle_heartbeat_diagnostics(self, stat):
        stat.summary(DiagnosticStatus.OK, "controller change to keyboard")
        return stat