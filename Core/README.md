# Micromouse Core
This contains all of the code for the Micromouse Bot

## Installation
Instead of using the Arduino IDE, we're using [PlatformIO](https://docs.platformio.org/en/latest/core/index.html) to compile and upload code to the arduino. PIO is much better than the Arduino IDE because it allows us to easily compile multiple files at one time, use a version control system, and run code on multiple devices (including embedded that are not supported by Arduino, like [this](https://www.digikey.com/en/products/detail/stmicroelectronics/STM32F103R8T6TR/2035367))

Note that there's both a CLI and a VSCode extension for PlatformIO. I reccomend installing both, as the CLI is useful for debugging while the VSCode entension has a nice GUI and allows your IDE to view builtin Arduino libraries.

To install the VSCode extension, see [this guide](https://platformio.org/install/ide?install=vscode). To install the CLI, see [this guide](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html). If you have homebrew on MacOS, you can install it with 
```bash
brew install platformio
```
## Rough structure of the project
- `/src` is where spicy things happen.
- `/include` allows spicy things to happen by linking them together.
- `platformio.ini` declares which environment we want to run the program on, and the project's dependencies.
- `/test` is not in use right now.
## Installing dependencies
This shouldn't be a step, since the building process of platformio should automatically install and resolve dependencies. But in the case this doesn't work, use:
```bash
pio pkg install
```

## Running
To upload code:

```bash
pio run --target upload
```

To read the serial output of the Arduino (note that this will only display output ***after*** code is running on the Arduino) and essentially checking if the board is connected:

```bash
pio device monitor
```

To simply test if the code will compile (note that this does NOT require that the Arduino is connected):

```bash
pio run
```

To specify the environment in any of the following commands, use the `--environment` flag:

```bash
[command] --environment nano33ble
```