# MQTT client based on SOSS

The client wraps [soss-mqtt](../soss-mqtt) to simplify the configuration for ROS1 and ROS2.

## Setup

`mqtt-client` depends on [SOSS](https://github.com/osrf/soss) and has to be built from source.

1. Install dependencies

```bash
sudo apt-get install libboost-dev libboost-program-options-dev libyaml-cpp-dev libssl-dev python3-colcon-common-extensions python3-vcstool
```

2. Import code

```bash
vcs import --recursive < mqtt-client/soss.repos
```

3. Build

```bash
. /opt/ros/noetic/setup.bash
colcon build --packages-up-to mqtt-client-ros1
```
or
```bash
. /opt/ros/foxy/setup.bash
colcon build --packages-up-to mqtt-client-ros2
```

## Usage

```
Usage: mqtt-client [options] CONFIG_FILE
Options:
  --help                produce help message
  --address arg         MQTT broker address
  --config arg          Client mapping
  --id arg              MQTT client ID
  --scope arg           Client scope
  --debug               print soss config
  --print               print soss config only
```

`address` is required, either from the command line or the `mqtt` section of the config file.
The `scope` parameters act as MQTT topic prefix for all configured ROS interfaces.
The command line parameters take precedence over the config file.

`--debug` and `--print` will both output the effective soss configuration.

In additon the environment variable `ROS_VERSION` must be set (from `setup.bash`).

Example: `mqtt-client --address localhost --scope test test.yml`

## Configuration

### Interfaces

Example:

```yaml
publishers:
  out: { type: "std_msgs/String" }

subscribers:
  in: { type: "std_msgs/String" }

servers:
  local_service: { type: "std_srvs/SetBool" }

clients:
  remote_service: { type: "std_srvs/SetBool" }
```

Each entry can carry an additional `remap` setting, which changes the MQTT mapping.
It defaults to `$scope/$name` ( or `$name` if scope is empty).

The following expressions have been implemented:
* `$name`: name of the topic/service (from configuration)
* `$scope`: scope (from CLI)
* `$id`: client id (from CLI)

### Middleware

Middleware settings can be specified for both systems, `mqtt` (using [`soss-mqtt`](../soss-mqtt#configuration)) and `ros` (either `soss-ros1` or `soss-ros2`):

At top level:
```yaml
mqtt: { json_indent: 2 }
ros: { node_name: my_node }
```

Or per interface:

```yaml
publishers:
  out: { type: "std_msgs/String", mqtt: {qos: 1}}
```

## mqtt-client-ros1 / mqtt-client-ros2

`mqtt-client` is version agnostic, but to work with ROS all message and service definitions have to be generated.

`mqtt-client-ros1` and `mqtt-client-ros2` are wrappers, which pull in `std_msgs` and `std_srvs`.
Use case specific definitions can be added in their `CMakeLists.txt`.
