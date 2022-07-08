Navio2 Setup Instructions are here (they are pretty good): https://docs.emlid.com/navio2/

* Login 'pi' and 'raspberry'.
* Change `pi` password to typical one
* Before running `apt update`, you may need to fix an APT key issue by running
  the following command for ROS 1 installs (and, yes, I realize we aren't using
  ROS so probably either one will work, I used the ROS 1):

      curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

  Or this for ROS 2 installs:

      sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

  - For more information, see [here](https://answers.ros.org/question/379190/apt-update-signatures-were-invalid-f42ed6fbab17c654/)
* Update APT repo and distro:

```
sudo apt update -y
sudo apt dist-upgrade -y
```

* Install some tools and optionally setup vim/bash:

```
sudo apt install -y git vim-nox exuberant-ctags screen cmake dos2unix

cd
git clone git@github.com:ahelten/dotvim .vim
cd .vim
git submodule update --init
cd
ln -s .vim/.vimrc ~/.vimrc
ln -s .vim/.screenrc ~/.screenrc
echo '. ~pi/.vim/bashrc_extras' >> ~/.bashrc
```

Building ArduRover for Navio2 on Linux
====


```
./waf configure --board navio2
./war rover
```


Running ArduRover on Linux
====

See `ardupilot/libraries/AP_HAL/HAL.h` for the list of UARTs that can be assigned from the command
line.

Running it requires sudo, a TCP IP:port for mavlink connections, `-B` for GPS1, and `-E` for GPS2.

To run with u-blox GPS heading kit (**NOTE:** different port so this can run alongside a Pixhawk):

```
sudo ./build/navio2/bin/ardurover -A tcp:0.0.0.0:5761 -B /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_*-if00-port0 -E /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00
```

To run with Septentrio GPS (**NOTE:** different port so this can run alongside a Pixhawk):

```
sudo ./build/navio2/bin/ardurover -A tcp:0.0.0.0:5761 -B /dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_3237687-if02
sudo ./build/navio2/bin/ardurover -A tcp:0.0.0.0:5761 -B /dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_*-if02
```

