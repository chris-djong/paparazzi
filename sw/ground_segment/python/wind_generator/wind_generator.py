import sys
import signal

# Location of pprz libraries are hardcoded for now
sys.path.append("/home/chris/paparazzi/var/lib/python")

import pprzlink
from pprzlink import ivy

# Keep track of current desired wind variables
w_east_desired = 0
w_north_desired = 0
w_up_desired = 0

def get_wind():
    return 7.5, 7.5, 7.5


def ivy_request_callback(sender, msg, resp, *args, **kwargs):
    """
        Ivy Callback for Paparazzi Requests
    """

    if msg.msg_class == "ground" and msg.name == "WORLD_ENV_REQ":
        return worldenv_cb(msg, resp)
    else:
        return None


def worldenv_cb(ac_id, _, msg):
    """
        Callback for paparazzi WORLD_ENV requests
    """
    # request location (in meters)
    east, north, up = float(msg.get_field(3)),\
        float(msg.get_field(4)),\
        float(msg.get_field(5))
    up *= -1
    # convert in km + translation with mesoNH origin
    weast, wnorth, wup = get_wind()

    msg_back = pprzlink.message.PprzMessage("ground", "WORLD_ENV")
    msg_back.set_value_by_name("wind_east", weast)
    msg_back.set_value_by_name("wind_north", wnorth)
    msg_back.set_value_by_name("wind_up", wup)
    msg_back.set_value_by_name("ir_contrast", 266)
    msg_back.set_value_by_name("time_scale", 1)
    msg_back.set_value_by_name("gps_availability", 1)
    ivy_interface.send(msg_back, None)

    print("Executing world_env_req cb..")

"""
def worlddesired_cb(ac_id, msg):
    east, north, up = float(msg.get_field(3)),\
        float(msg.get_field(4)),\
        float(msg.get_field(5))
    up *= -1

    # And assign to global variables
    global w_east_desired
    w_east_desired = east
    global w_north_desired
    w_north_desired= north
    global w_up_desired
    w_up_desired = up

    print("Executing world desired cb..")
"""
def signal_handler(signal, frame):
    print('\nShutting down IVY...')
    ivy_interface.shutdown()
    print("Done.")

def main():
    print("Starting wind simulator..")
    print("Registering signal handler..")
    # register signal handler for ctrl+c to stop the program
    signal.signal(signal.SIGINT, signal_handler)

    print("Creating ivy interface..")
    # init ivy and register callback for WORLD_ENV_REQ and NPS_SPEED_POS
    global ivy_interface
    ivy_interface = ivy.IvyMessagesInterface("HillFlow")
    # ivy_interface.subscribe(worlddesired_cb, '(.* WORLD_DESIRED .*)')
    ivy_interface.subscribe(worldenv_cb, '(.* WORLD_ENV_REQ .*)')

    print("Starting main loop..")
    # wait for ivy to stop
    from ivy.std_api import IvyMainLoop  # noqa

    signal.pause()


if __name__ == '__main__':
    main()