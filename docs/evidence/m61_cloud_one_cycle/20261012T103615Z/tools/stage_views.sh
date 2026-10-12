#!/usr/bin/env bash
# View-only GUI camera moves for extra screenshots (does not touch the simulation state).
RUN=$1
EVT=$(dirname "$RUN")/tools
source $EVT/env.sh
C=$RUN/captures; mkdir -p $C
timeout 20 ign service -l > $C/ign_services.txt 2>&1
if grep -q "^/gui/move_to$" $C/ign_services.txt; then
  timeout 20 ign service -s /gui/move_to --reqtype ignition.msgs.StringMsg \
    --reptype ignition.msgs.Boolean --timeout 5000 --req 'data: "spiderx"' > $C/gui_move_to.txt 2>&1
  sleep 4
  import -window root -display :77 $C/screenshot_gui_move_to_spiderx.png > /dev/null 2>&1
  echo "move_to spiderx screenshot exit=$?" >> $C/screenshot.txt
fi
if grep -q "^/gui/move_to/pose$" $C/ign_services.txt; then
  # side view from +y, level with the body, looking toward -y (leg/ground clearance)
  timeout 20 ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req 'pose: {position: {x: 0.0, y: 0.9, z: 0.14}, orientation: {x: 0, y: 0, z: -0.7071068, w: 0.7071068}}' \
    > $C/gui_move_to_pose.txt 2>&1
  sleep 4
  import -window root -display :77 $C/screenshot_gui_side.png > /dev/null 2>&1
  echo "side view screenshot exit=$?" >> $C/screenshot.txt
  # foot-level view: eye 8 mm above the ground, 0.4 m to the side, looking along -y; a foot that
  # touched the ground would sit on the ground line, a hanging foot shows a gap below it
  timeout 20 ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req 'pose: {position: {x: 0.05, y: 0.40, z: 0.008}, orientation: {x: 0, y: 0, z: -0.7071068, w: 0.7071068}}' \
    > $C/gui_move_to_pose_footlevel.txt 2>&1
  sleep 4
  import -window root -display :77 $C/screenshot_gui_foot_level.png > /dev/null 2>&1
  echo "foot-level view screenshot exit=$?" >> $C/screenshot.txt
fi
cat $C/screenshot.txt
