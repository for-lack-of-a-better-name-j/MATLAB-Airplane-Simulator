from time import sleep
from dualsense_controller import DualSenseController as ds

global is_running
# get the default device, check that there are not None.
assert ds.enumerate_devices() != [], (
    "Could not find a PS5 controller. Did you follow setup instructions per https://pypi.org/project/dualsense-controller/ ?"
)


def on_cross_btn_pressed():
    print("cross button pressed!")
    with open("control_file", "w") as f:
        f.write("cross button pressed!")


def stop_on_o_btn_pressed():
    global is_running
    is_running = False
    print("stopping capture!")


def on_left_stick_y_changed(left_stick_y):
    print(f"left stick y:{left_stick_y}")
    with open("left_stick_y", "w") as f:
        # f.write(str(left_stick_y))
        if left_stick_y < 0:
            f.write(str(abs(0.5 * left_stick_y)))
        else:
            f.write(str(0.5 * left_stick_y + 0.5))


def on_left_stick_x_changed(left_stick_x):
    print(f"left stick x:{left_stick_x}")
    with open("left_stick_x", "w") as f:
        f.write(str(left_stick_x))
        # if left_stick_x < 0:
        #     f.write(str(abs(0.5 * left_stick_x)))
        # else:
        #     f.write(str(0.5 * left_stick_x + 0.5))
        #


def on_right_stick_y_changed(right_stick_y):
    print(f"right stick y:{right_stick_y}")
    with open("right_stick_y", "w") as f:
        f.write(str(right_stick_y))
        # if right_stick_y < 0:
        #     f.write(str(abs(0.5 * right_stick_y)))
        # else:
        #     f.write(str(0.5 * right_stick_y + 0.5))
        #


def on_right_stick_x_changed(right_stick_x):
    print(f"right stick x:{right_stick_x}")
    with open("right_stick_x", "w") as f:
        f.write(str(right_stick_x))
        # if right_stick_x < 0:
        #     f.write(str(abs(0.5 * right_stick_x)))
        # else:
        #     f.write(str(0.5 * right_stick_x + 0.5))
        #


def on_error(error):
    global is_running
    is_running = False
    print(f"stopping capture due to error {error}!")


# instantiate a controller object for the first PS5 controller found
ctl = ds()

ctl.btn_cross.on_down(on_cross_btn_pressed)
ctl.btn_circle.on_down(stop_on_o_btn_pressed)

ctl.left_stick_x.on_change(on_left_stick_x_changed)
ctl.left_stick_y.on_change(on_left_stick_y_changed)
ctl.right_stick_x.on_change(on_right_stick_x_changed)
ctl.right_stick_y.on_change(on_right_stick_y_changed)
ctl.activate()
for i in range(1, 10):
    sleep(0.1)
    ctl.lightbar.set_color_blue()
    sleep(0.1)
    ctl.lightbar.set_color_green()
# set up a little event loop.
print(
    "Successfully initialized PS5 controller input capture.\n    - To stop capture, press circle. \n    - The controller's light should have flashed blue and green."
)
is_running = True
while is_running:
    sleep(0.001)
