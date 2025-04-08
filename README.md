# How PS5 Controller Input was Achieved

  - The controller was connected via USB on a computer using Arch Linux.
  - Device rules were set to allow USB device read/write per https://pypi.org/project/dualsense-controller/#getting-started---simple-example .
  - The python script in handles the USB input from the controller, and writes the pertinent information to a file in the same directory.
  - The MATLAB script reads the file for control input.

