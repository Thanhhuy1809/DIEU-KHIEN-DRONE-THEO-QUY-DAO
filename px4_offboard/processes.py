
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    "cd ~/PX4-Autopilot && make px4_sitl gz_x500"

    # Run QGroundControl
    # "cd ~/QGroundControl && ./QGroundControl.AppImage"
]


for command in commands:
   
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    time.sleep(1)
