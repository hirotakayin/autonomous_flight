#!/bin/bash
# Opens the mission dashboard in the default browser (roslaunch passes extra args; they are ignored).
sleep 3
xdg-open "file://$(rospack find autonomous_flight)/gui/mission_control.html" >/dev/null 2>&1
