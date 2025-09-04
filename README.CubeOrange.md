Retrieve the code:
====

    git clone --recursive git@github.com:ahelten/ardupilot.git
    git remote add main-ardupilot https://github.com/ArduPilot/ardupilot
    git fetch main-ardupilot
    git submodule update --init --recursive

      # Use whatever the HEAD of most recent `hpposllh_` branch:
    git branch -av|grep hpposllh_
      # For example:
    git checkout hpposllh_sync_with_4.6.2

      # Or checkout using the latest tag of the latest branch:
    git tag -l | grep ahelten


Mavlink Submodule
----

Don't forget the Mavlink repo:

    cd modules/mavlink
    git remote add main-mavlink ssh://git@github.com/ArduPilot/mavlink
    git fetch

To check if there were changes in the verion used by ArduPilot:

1. Go to `ardupilot/modules` directory
2. Diff the submodule directory with whatever the current ArduPilot tag being synced
3. Go into the `ardupilot/modules/mavlink` directory and diff my forks' HEAD with the current
   ArduPilot version

For example:

    $ cd ardupilot/modules
    $ git diffw Rover-4.4.0 ./mavlink/
    diff --git a/modules/mavlink b/modules/mavlink
    index 806a3ba749..ad642f3bdb 160000
    --- a/modules/mavlink
    +++ b/modules/mavlink
    @@ -1 +1 @@
    -Subproject commit 806a3ba7499da677ded35226d6ff88e5a100c4c2
    +Subproject commit ad642f3bdbbedc1ddae86af27511dceeff27473d

    $ cd mavlink
    $ git diffw 806a3ba74


Build ArduPilot
====

For original Cube Orange:

    cd <ardupilot>
    ./waf configure --board CubeOrange
    ./waf rover
    ll build/CubeOrange/bin/ardurover.apj

For Cube+:

    cd <ardupilot>
    ./waf configure --board CubeOrangePlus
    ./waf rover
    ll build/CubeOrangePlus/bin/ardurover.apj


Copy ArduPilot
====

From VM to a bot:

    rsync -zaP ./build/CubeOrange/bin/ardurover.apj gfr@robot73:
    rsync -zaP ./build/CubeOrangePlus/bin/ardurover.apj gfr@robot73:


Program ArduPilot
====

Two approaches have been used to remote-update the Cube. However, the uSD method requires a 4.5+ to
work is known to *not* work with 4.4.0. The uploader.py script is known to work with 4.6.2 but not
yet known whether it works to update 4.4.0 -> 4.6.2.

If both update methods fail to update 4.4.0 to something newer, it must be updated to 4.6.2 or newer
using Mission Planner with a hard-wired USB connection from laptop to the Cube. After this USB
update, the remote update should be supported.

Using uploader.py Script
----

This method is known to work with 4.6.2 but it's not known whether it will work with 4.4.0:

```
sudo systemctl stop weedbot
cd <ardupilot>

# Use correct serial device name! (note: this will typically run on the bot!)
./Tools/scripts/uploader.py --port /dev/ttyAMA2 --baud-flightstack 921600 ~/ardurover.apj
```

**NOTE:** Don't try setting `--baud-bootloader` to something higher than the default, it
didn't work with Cube Orange and ArduPilot 4.6.2.


uSD Update
----

This method is known to work with 4.6.2 but is known to *not* work with 4.4.0:

1. Use Mission Planner or `mavproxy.py` to upload the `ardurover.abin` file to `/ardupilot.abin`
   (note the rename to `ardupilot.abin`)
   * Mission Planner: go to `Config -> MAVFtp`, select the `/` directory, right-click in the
     directory listing, and select `Upload`. Browse and select the `ardupilot.abin` file.
   * `mavproxy.py`:
```
pip install mavproxy
  # -- OR -- (sudo may or may not be required)
sudo pip install mavproxy

sudo systemctl stop weedbot
mavproxy.py --master=tcp:127.0.0.1:55760
MANUAL> ftp put ardupilot.abin
```
2. Power cycle the Cube. Power cycle is *required* -- a reset/reboot is not sufficient!!
3. After power-cycling, the Cube will take longer than normal to boot because it's reflashing the
   firmware. This typically isn't a problem with our bots given they take longer to boot than the
   reflash.
4. One verification that it worked is the file on uSD will be renamed `ardupilot-flashed.abin`


Setup Pi for ArduPilot ser2net
====

Add to `/etc/ser2net.yaml` (update actual serial device name and baudrate using
`stty -a -F /dev/ttyAMA2`):

```
connection: &pixhawk
   accepter: tcp,5760
   enable: off
   options:
     banner: *banner
     kickolduser: true
     telnet-brk-on-sync: true
   connector: serialdev,
             /dev/ttyAMA2,
             921600n81,local
```

If the `yaml` file doesn't exist, add to `/etc/ser2net.conf` (update actual serial device name):

```
5760:raw:600:/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_470041001451303039333335-if00:921600 NONE 1STOPBIT 8DATABITS LOCAL -RTSCTS
```


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

