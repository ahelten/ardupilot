Retrieve the code:
====

    git clone --recursive git@github.com:ahelten/ardupilot.git
    git remote add main-ardupilot https://github.com/ArduPilot/ardupilot
    git fetch main-ardupilot
    git submodule update --init --recursive

      # Use whatever the HEAD of most recent `hpposllh_` branch:
    git branch -av|grep hpposllh_
      # For example:
    git checkout hpposllh_sync_with_4.2.3-rc1

      # Or checkout using the latest tag of the latest branch:
    git tag -l | grep ahelten

Build ArduPilot:
====

    cd <ardupilot>
    ./waf configure --board CubeOrange
    ./waf rover
    ll build/CubeOrange/bin/ardurover.apj

For Cube+ (not currently using this hardware version, but we do have one):

    cd <ardupilot>
    ./waf configure --board CubeOrangePlus
    ./waf rover
    ll build/CubeOrangePlus/bin/ardurover.apj


Copy ArduPilot:
====

From VM to a bot:

    scp ./build/CubeOrange/bin/ardurover.apj pi@192.168.2.177:amh_devel

To PC from VM:

    scp ahelten@10.10.0.48:amh_devel/ardupilot/build/CubeOrange/bin/ardurover.apj .


Program ArduPilot:
====

```
sudo ~/Greenfield/killit
cd ~/amh_devel/ardupilot

# Use correct serial device name!
python3 Tools/scripts/uploader.py --port /dev/ttyACM0 ../ardurover.apj
# Or:
python3 ./uploader.py --port $(readlink -f /dev/serial/by-id/usb-*_CubeOrange*-if00) ./ardurover.apj
```


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

