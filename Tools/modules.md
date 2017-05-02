
# Modules & Commands Reference
Documentation of PX4 modules, drivers and commands. It describes the provided
functionality, high-level implementation overview and how to use the
command-line interface.

> **Note** **This list is auto-generated from the source code** and contains the
> most recent modules documentation.

This is not a complete list and NuttX provides some additional commands
as well (such as `free`). Use `help` on the console to get a list of all
available commands, and in most cases `command help` will print the usage.

Since this is generated from source, errors must be reported/fixed 
in the [Firmware](https://github.com/PX4/Firmware) repository.

# Category: Command
## param
Source: [systemcmds/param](https://github.com/PX4/Firmware/tree/master/src/systemcmds/param)

### Usage
```
param <command> [arguments...]
 Commands:
   load          Load params from a file (overwrite all)
     [<file>]    File name (use default if not given)

   import        Import params from a file
     [<file>]    File name (use default if not given)

   save          Save params to a file
     [<file>]    File name (use default if not given)

   select        Select default file
     [<file>]    File name (use <root>/eeprom/parameters if not given)

   show          Show parameter values
     [-c]        Show only changed params
     [<filter>]  Filter by param name (wildcard at end allowed, eg. sys_*)

   set           Set parameter to a value
     <param_name> <value> Parameter name and value to set
     [fail]      If provided, let the command fail if param is not found

   compare       Compare a param with a value. Command will succeed if equal
     <param_name> <value> Parameter name and value to compare

   greater       Compare a param with a value. Command will succeed if param is
                 greater than the value
     <param_name> <value> Parameter name and value to compare

   reset         Reset params to default
     [<exclude1> [<exclude2>]] Do not reset matching params (wildcard at end
                 allowed)

   reset_nostart Reset params to default, but keep SYS_AUTOSTART and
                 SYS_AUTOCONFIG
     [<exclude1> [<exclude2>]] Do not reset matching params (wildcard at end
                 allowed)

   index         Show param for a given index
     <index>     Index: an integer >= 0

   index_used    Show used param for a given index
     <index>     Index: an integer >= 0

   find          Show index of a param
     <param>     param name
```
## pwm
Source: [systemcmds/pwm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/pwm)

### Usage
```
pwm <command> [arguments...]
 Commands:
   arm           Arm output

   disarm        Disarm output

   info          Print current configuration of all channels

   forcefail     Force Failsafe mode
     on|off      Turn on or off

   terminatefail Force Termination Failsafe mode
     on|off      Turn on or off

   rate          Configure PWM rates
     -r <val>    PWM Rate in Hz (0 = Oneshot, otherwise 50 to 400Hz)

   oneshot       Configure Oneshot125 (rate is set to 0)

   failsafe      Set Failsafe PWM value

   disarmed      Set Disarmed PWM value

   min           Set Minimum PWM value

   max           Set Maximum PWM value

   test          Set Output to a specific value until 'q' or 'c' or 'ctrl-c'
                 pressed

   steps         Run 5 steps from 0 to 100%

 The commands 'failsafe', 'disarmed', 'min', 'max' and 'test' require a PWM
 value:
     -p <val>    PWM value (eg. 1100)

 The commands 'rate', 'oneshot', 'failsafe', 'disarmed', 'min', 'max', 'test'
 and 'steps' additionally require to specify the channels with one of the
 following commands:
     [-c <val>]  select channels in the form: 1234 (1 digit per channel,
                 1=first)
     [-m <val>]  Select channels via bitmask (eg. 0xF, 3)
                 default: 0
     [-g <val>]  Select channels by group (eg. 0, 1, 2. use 'pwm info' to show
                 groups)
                 default: 0
     [-a]        Select all channels

 These parameters apply to all commands:
     [-d <val>]  Select PWM output device
                 values: <file:dev>, default: /dev/pwm_output0
     [-v]        Verbose output
     [-e]        Exit with 1 instead of 0 on error
```
# Category: Communication
## mavlink
Source: [modules/mavlink](https://github.com/PX4/Firmware/tree/master/src/modules/mavlink)


### Description
This module implements the MAVLink protocol, which can be used on a Serial link or UDP network connection.
It communicates with the system via uORB: some messages are directly handled in the module (eg. mission
protocol), others are published via uORB (eg. vehicle_command).

Streams are used to send periodic messages with a specific rate, such as the vehicle attitude.
When starting the mavlink instance, a mode can be specified, which defines the set of enabled streams with their rates.
For a running instance, streams can be configured via `mavlink stream` command.

There can be multiple independent instances of the module, each connected to one serial device or network port.

### Implementation
The implementation uses 2 threads, a sending and a receiving thread. The sender runs at a fixed rate and dynamically
reduces the rates of the streams if the combined bandwidth is higher than the configured rate (`-r`) or the
physical link becomes saturated. This can be checked with `mavlink status`, see if `rate mult` is less than 1.

### Examples
Start mavlink on ttyS1 serial with baudrate 921600 and maximum sending rate of 80kB/s:
```
mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000
```

Start mavlink on UDP port 14556 and enable the HIGHRES_IMU message with 50Hz:
```
mavlink start -u 14556 -r 1000000
mavlink stream -u 14556 -s HIGHRES_IMU -r 50
```

### Usage
```
mavlink <command> [arguments...]
 Commands:
   start         Start a new instance
     [-d <val>]  Select Serial Device
                 values: <file:dev>, default: /dev/ttyS1
     [-b <val>]  Baudrate
                 default: 57600
     [-r <val>]  Maximum sending data rate in B/s (if 0, use baudrate / 20)
                 default: 0
     [-u <val>]  Select UDP Network Port (local)
                 default: 14556
     [-o <val>]  Select UDP Network Port (remote)
                 default: 14550
     [-t <val>]  Partner IP (broadcasting can be enabled via MAV_BROADCAST
                 param)
                 default: 127.0.0.1
     [-m <val>]  Mode: sets default streams and rates
                 values: custom|camera|onboard|osd|magic|config|iridium,
                 default: normal
     [-f]        Enable message forwarding to other Mavlink instances
     [-v]        Verbose output
     [-w]        Wait to send, until first message received
     [-x]        Enable FTP

   stop-all      Stop all instances

   status        Print status for all instances

   stream        Configure the sending rate of a stream for a running instance
     [-u <val>]  Select Mavlink instance via local Network Port
                 default: 0
     [-d <val>]  Select Mavlink instance via Serial Device
                 values: <file:dev>
     -s <val>    Mavlink stream to configure
     -r <val>    Rate in Hz (0 = turn off)

   boot_complete Enable sending of messages. (Must be) called as last step in
                 startup script.
```
## uorb
Source: [modules/uORB](https://github.com/PX4/Firmware/tree/master/src/modules/uORB)

### Usage
```
uorb <command> [arguments...]
 Commands:
   start

   status        Print topic statistics

   top           Monitor topic publication rates
     [-a]        print all instead of only currently publishing topics
     [<filter1> [<filter2>]] topic(s) to match (implies -a)
```
# Category: Driver
## fmu
Source: [drivers/px4fmu](https://github.com/PX4/Firmware/tree/master/src/drivers/px4fmu)


### Description
This module is responsible for driving the output and reading the input pins. For boards without a separate IO chip
(eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the
px4io driver is used for main ones.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.
In addition it does the RC input parsing and auto-selecting the method. Supported methods are:
- PPM
- SBUS
- DSM
- SUMD
- ST24

The module is configured via mode_* commands. This defines which of the first N pins the driver should occupy.
By using mode_pwm4 for example, pins 5 and 6 can be used by the camera trigger driver or by a PWM rangefinder
driver.

### Implementation
By default the module runs on the work queue, to reduce RAM usage. It can also be run in its own thread,
specified via start flag -t, to reduce latency.
When running on the work queue, it schedules at a fixed frequency, and the pwm rate limits the update rate of
the actuator_controls topics. In case of running in its own thread, the module polls on the actuator_controls topic.
Additionally the pwm rate defines the lower-level IO timer rates.

### Examples
It is typically started with:
```
fmu mode_pwm
```
To drive all available pins.

Capture input (rising and falling edges) and print on the console: start the fmu in one of the capture modes:
```
fmu mode_pwm3cap1
```
This will enable capturing on the 4th pin. Then do:
```
fmu test
```

Use the `pwm` command for further configurations (PWM rate, levels, ...), and the `mixer` command to load
mixer files.

### Usage
```
fmu <command> [arguments...]
 Commands:
   start         Start the task (without any mode set, use any of the mode_*
                 cmds)
     [-t]        Run as separate task instead of the work queue

 All of the mode_* commands will start the fmu if not running already

   mode_gpio

   mode_rcin     Only do RC input, no PWM outputs

   mode_pwm      Select all available pins as PWM

   mode_pwm1

   mode_pwm4

   mode_pwm2

   mode_pwm3

   mode_pwm3cap1

   mode_pwm2cap2

   mode_serial

   mode_gpio_serial

   mode_pwm_serial

   mode_pwm_gpio

   bind          Send a DSM bind command (module must be running)

   sensor_reset  Do a sensor reset (SPI bus)
     [<ms>]      Delay time in ms between reset and re-enabling

   peripheral_reset Reset board peripherals
     [<ms>]      Delay time in ms between reset and re-enabling

   i2c           Configure I2C clock rate
     <bus_id> <rate> Specify the bus id (>=0) and rate in Hz

   test          Test inputs and outputs

   fake          Arm and send an actuator controls command
     <roll> <pitch> <yaw> <thrust> Control values in range [-100, 100]

   stop

   status        print status info
```
# Category: System
## logger
Source: [modules/logger](https://github.com/PX4/Firmware/tree/master/src/modules/logger)


### Description
System logger which logs a configurable set of uORB topics and system printf messages
(warnings and errors) to ULog files. These can be used for system and flight performance evaluation,
tuning, replay and crash analysis.

It supports 2 backends:
- Files: write ULog files to the file system (SD card)
- MAVLink: stream ULog data via MAVLink to a client (the client must support this)

Both backends can be enabled and used at the same time.

### Implementation
The implementation uses two threads:
- The main thread, running at a fixed rate (or polling on a topic if started with -p) and checking for
  data updates
- The writer thread, writing data to the file

In between there is a write buffer with configurable size. It should be large to avoid dropouts.

### Examples
Typical usage to start logging immediately:
```
logger start -e -t
```

Or if already running:
```
logger on
```

### Usage
```
logger <command> [arguments...]
 Commands:
   start
     [-m <val>]  Backend mode
                 values: file|mavlink|all, default: all
     [-e]        Enable logging right after start until disarm (otherwise only
                 when armed)
     [-f]        Log until shutdown (implies -e)
     [-t]        Use date/time for naming log directories and files
     [-r <val>]  Log rate in Hz, 0 means unlimited rate
                 default: 280
     [-b <val>]  Log buffer size in KiB
                 default: 12
     [-q <val>]  uORB queue size for mavlink mode
                 default: 14
     [-p <val>]  Poll on a topic instead of running with fixed rate (Log rate
                 and topic intervals are ignored if this is set)
                 values: <topic_name>

   on            start logging now, override arming (logger must be running)

   off           stop logging now, override arming (logger must be running)

   stop

   status        print status info
```
## send_event
Source: [modules/events](https://github.com/PX4/Firmware/tree/master/src/modules/events)


### Description
Background process running periodically on the LP work queue to perform housekeeping tasks.
It is currently only responsible for temperature calibration.

The tasks can be started via CLI or uORB topics (vehicle_command from MAVLink, etc.).

### Usage
```
send_event <command> [arguments...]
 Commands:
   start         Start the background task

   temperature_calibration Run temperature calibration process
     [-g]        calibrate the gyro
     [-a]        calibrate the accel
     [-b]        calibrate the baro (if none of these is given, all will be
                 calibrated)

   stop

   status        print status info
```
## top
Source: [systemcmds/top](https://github.com/PX4/Firmware/tree/master/src/systemcmds/top)

Monitor running processes and their CPU, stack usage, priority and state
### Usage
```
top [arguments...]
   once          print load only once
```
