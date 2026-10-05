#!/bin/bash
while pgrep -f "git clone.*IQ.Pilot.HondaBoschA" > /dev/null; do
  echo "Still cloning..."
  sleep 5
done
echo "Clone finished! Starting scons build..."
ssh comma 'cd /data/openpilot && scons -j4' > scons_output.log 2>&1
echo "Scons build finished! Check scons_output.log"
