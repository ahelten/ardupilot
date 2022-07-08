Build ArduPilot:
====

    cd ~/amh_devel/ardupilot
    ./waf configure --board CubeOrange
    ./waf rover


Copy ArduPilot:
====

From VM to a bot:

    scp ./build/CubeOrange/bin/ardurover.apj pi@192.168.2.177:amh_devel

To PC from VM:

    scp ahelten@10.10.0.48:amh_devel/ardupilot/build/CubeOrange/bin/ardurover.apj .


Program ArduPilot:
====

    sudo ~/Greenfield/killit
    cd ~/amh_devel/ardupilot

    # Use correct serial device name!
    python3 Tools/scripts/uploader.py --port /dev/ttyACM0 ../ardurover.apj


Setup Pi for ArduPilot ser2net:
====

Add to `/etc/ser2net.conf` (update actual serial device name):

   5760:raw:600:/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_470041001451303039333335-if00:921600 NONE 1STOPBIT 8DATABITS LOCAL -RTSCTS


SITL
====

Setup, Build, and Run:

    cd ~/ardupilot/Rover
    sim_vehicle.py --map --console

In the console running `sim_vehicle.py`:

    GUIDED
    arm throttle


Unit Tests
====

May need to checkout a different gtest commit:

    cd modules/gtest
    git checkout 10b1902d893ea8cc43c69541d70868f91af3646b

Configure and Build (this does not *run* the tests!):

    ./waf configure --board=linux --debug
    ./waf tests

Running a specific test:

    ./build/linux/tests/test_location

Running all unit tests:

    # NO IDEA how to do this -- can't seem to find any information on this !?!?
    # So I wrote a simple bash script:
    ./run_tests.sh

