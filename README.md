![ODrive Logo](https://static1.squarespace.com/static/58aff26de4fcb53b5efd2f02/t/59bf2a7959cc6872bd68be7e/1505700483663/Odrive+logo+plus+text+black.png?format=1000w)

# Unofficial fork of ODrive
This is a fork that happened at version 0.5.1. I mainly developed this for my [Milana Robot](https://github.com/helmutbuhler/milana_robot) project. But even for other projects, this fork might be useful to some people because this fork has some features independent of that project.
Since there are no version numbers left between 0.5.1 and 0.5.2, I gave this fork the version number 0.5.100 and will count up from that.

Some of the features new in this fork:
 - Better oscilloscope functionality (data compression, multiple values, multiple axes, custom trigger, faster data retrieval). You can use [Control UI](https://github.com/helmutbuhler/odrive_control_ui) to use that feature.
 - More data to debug Encoder Index issues (to detect encoder noise/slippage).
 - Optional error suppression with CUI encoders.
 - ODrivetool works with ODrive via UART (The official 0.5.2 version dropped this feature).
 - ODrivetool in this version can be called with custom UART options (baudrate and stopbits). (Call with --help for details)
 - ODrive can be configured to use a different baudrate/stopbits for UART communication.
 - Optional reading of incremental encoders via Fibre when ODrive uses SPI encoders.
 - Some minor UART communication bugfixes in Firmware.
 - There is also some functionality specific to the Milana Robot project. But you can just ignore that if you don't need it. The robot specific stuff is all in memory, nothing is persisted in the config.

In this [diff](https://github.com/helmutbuhler/odrive_milana/compare/fw-v0.5.1...fw-v0.5.100?diff=split&w=#diff-a33c1b961195dd4f4049aa164d649b95bf0420dc26572775fc1c8b04bbb52625) of `odrive-interface.yaml`, you can see the new variables available via ODrivetool and also some comments explaining them.

## Binaries
If you don't want to build it yourself, some built binaries are available [here](https://github.com/helmutbuhler/odrive_milana/releases).

## Build
To build, clone this repository and cd into it and then into the folder `Firmware`. Open `tup.config.default` and uncomment the line defining CONFIG_BOARD_VERSION. Make sure the version matches your board. Save that file as `tup.config` and execute `make`. If you are missing some dependencies, refer to "Installing Prerequisites" in the [official developer guide](https://docs.odriverobotics.com/v/0.5.6/developer-guide.html)

I verified this on a Ubuntu 18.04 on a Jetson Nano and in a VM. I also tried this on a newer version of Ubuntu, and while it built there, the resulting binary file bricked the ODrive. I was only able to recover the ODrive with the DFU switch. So I recommend Ubuntu 18.04 for building this. The resulting .hex file should be around 746500 bytes big (the faulty ones were around 776.500 bytes big).

## Flash firmware
Once you have compiled (or downloaded) the .hex file, you must flash it to your ODrive. Connect ODrive via USB and make sure you can access it with ODrivetool: (assuming you are still in the Firmware folder)
```
sudo python3 ../tools/odrivetool
```
If you are on Windows, omit the sudo. If you get python package errors, try installing them by executing this:
```
pip3 install PyYAML Jinja2 jsonschema requests pyusb intelhex appdirs ipython pyserial
```
When that works, you can flash it:
```
sudo python3 ../tools/odrivetool dfu build/ODriveFirmware.hex
``` 
(Change the path to the hex file if you downloaded it and put it somewhere else)

Once that hopefully succeeds, I'd recommend to start odrivetool again and then force a reboot:

```
sudo python3 ../tools/odrivetool
odrv0.reboot()
```

## Usage
I recommend using the odrivetool in this repository and not the one on pip:
```
python3 tools/odrivetool
```
You can also use [Control UI](https://github.com/helmutbuhler/odrive_control_ui) to connect to an ODrive with this firmware.



# Original readme of ODrive 0.5.1:

This project is all about accurately driving brushless motors, for cheap. The aim is to make it possible to use inexpensive brushless motors in high performance robotics projects, like [this](https://www.youtube.com/watch?v=WT4E5nb3KtY).

| Branch | Build Status |
|--------|--------------|
| master | [![Build Status](https://travis-ci.org/madcowswe/ODrive.png?branch=master)](https://travis-ci.org/madcowswe/ODrive) |
| devel  | [![Build Status](https://travis-ci.org/madcowswe/ODrive.png?branch=devel)](https://travis-ci.org/madcowswe/ODrive) |

[![pip install odrive (nightly)](https://github.com/madcowswe/ODrive/workflows/pip%20install%20odrive%20(nightly)/badge.svg)](https://github.com/madcowswe/ODrive/actions?query=workflow%3A%22pip+install+odrive+%28nightly%29%22)

Please refer to the [Developer Guide](https://docs.odriverobotics.com/developer-guide) to get started with ODrive firmware development.


### Repository Structure
 * **Firmware**: ODrive firmware
 * **tools**: Python library & tools
 * **docs**: Documentation

### Other Resources

 * [Main Website](https://www.odriverobotics.com/)
 * [User Guide](https://docs.odriverobotics.com/)
 * [Forum](https://discourse.odriverobotics.com/)
 * [Chat](https://discourse.odriverobotics.com/t/come-chat-with-us/281)
