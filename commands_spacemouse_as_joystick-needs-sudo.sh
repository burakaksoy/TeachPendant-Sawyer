#!/bin/bash

# cat /proc/bus/input/devices
# sudo ~/relabsd/build/relabsd /dev/input/event14 ~/relabsd/conf/space_navigator.conf &

# sleep 2s

# xinput list
# xinput disable 10 # 3Dconnexion SpaceMouse Pro 
# xinput disable 17 # relabsd: 3Dconnexion SpaceMouse Pro

gnome-terminal --tab --title="Run As Joystick" -e "bash -c 'cat /proc/bus/input/devices; sudo /home/burak/relabsd/build/relabsd /dev/input/event14 /home/burak/relabsd/conf/space_navigator.conf;'bash";
# sleep 10s;

gnome-terminal --tab --title="Disable Mouse Function" -e "bash -c 'xinput list; xinput disable 10; xinput disable 17;'bash";
