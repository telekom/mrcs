# soss-mqtt

This package implements a [MQTT](https://mqtt.org/) extension for [SOSS (System Of Systems Synthesizer
)](https://github.com/osrf/soss) using the [Eclipse Paho MQTT C++ library](https://github.com/eclipse/paho.mqtt.cpp). SOSS is configured by a YAML file (see section [Configuration](#configuration)), where the user can setup a complex system composed of different middlewares and map topics and services between them.

## Principle

`soss-mqtt` is not meant to be used with arbitrary MQTT systems, but rather to provide a (scalable) brokerage layer for SOSS as depicted in the following diagram:

![broker architecture](doc/soss-mqtt.png)

In order to support SOSS topics and services on top of MQTT topics, this plugin implements a [custom topic mapping](#mapping) to differentiate the operations and to track the meta-data. All SOSS messages are converted to JSON objects and can be transmitted as text (for debugging) or [MessagePack](https://msgpack.org/) (default).

## Installation

`soss-mqtt` depends on [SOSS](https://github.com/osrf/soss) and has to be built from source.

1. Install dependencies

```bash
sudo apt-get install libboost-program-options-dev libyaml-cpp-dev libssl-dev python3-colcon-common-extensions python3-vcstool
```

2. Import SOSS repositories

```bash
vcs import < soss-mqtt/soss.repos
```

3. Build

```bash
colcon build --packages-up-to soss-mqtt
```

## Usage

An example [`test.yml`](doc/test.yml) file is shown below:
```yaml
systems:
    mqtt_broker: {type: mqtt, address: "tcp://localhost:1883"}
    ros: {type: ros2}
topics:
    out:
        type: "std_msgs/String"
        route: {from: ros, to: mqtt_broker}
        mqtt_broker: {json_indent: 2}
```

It connects a MQTT broker (e.g. mosquitto) and ROS2.
The broker and a test client can be installed with: `sudo apt-get mosquitto mosquitto-clients`.
To build `soss-ros2` (e.g. ROS2 foxy) and `soss-mqtt`:
```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to soss-mqtt soss-ros2 soss-ros2-test
```

The following steps need to be run in different terminals (or in the background):
1. start mosquitto (with default settings):
```bash
mosquitto
```

> mosquitto version 1.6.9 starting<br/>
> Using default config.<br/>
> Opening ipv4 listen socket on port 1883.<br/>
> Opening ipv6 listen socket on port 1883.

2. publish to `ROS` topic `out`
```bash
source install/setup.bash
ros2 topic pub /out std_msgs/msg/String "data: Hello world!"
```

> publisher: beginning loop<br/>
> publishing #&#8203;1: std_msgs.msg.String(data='Hello world!')

3. run the `soss` executable with the above configuration:
```bash
source install/setup.bash
soss src/soss-mqtt/doc/test.yml
```

> Connected to tcp&#8203;://localhost:1883<br/>
> Publishing to topic out with type: std_msgs/String and QOS: 0

4. subscribe to MQTT topic:
```bash
mosquitto_sub -t "out"
```

>{<br/>
>  "msg": {<br/>
>    "data": "Hello world!"<br/>
>  },<br/>
>  "op": "publish",<br/>
>  "type": "std_msgs/String"<br/>
>}

## Configuration
The `SOSS` configuration file is explained in detail in [SOSS concept](https://github.com/osrf/soss/blob/master/doc/concept.md). This section explains the configuration specific to `soss-mqtt`.

### System

```yaml
systems:
    mqtt_broker: {type: mqtt, address: "tcp://localhost:1883", qos: 0, json_indent: 2}
```

`address (no default)`: 	The address of the server to connect to, specified as a URI (see [supported protocol](https://github.com/eclipse/paho.mqtt.cpp#supported-network-protocols)).

`client_id` (default: ""): Unique client ID. If it is empty, a unique id will be randomly generated at each (re)connection.

`qos` (default: 0): [Quality of Service](https://www.hivemq.com/blog/mqtt-essentials-part-6-mqtt-quality-of-service-levels/) can be set for either for overall system or for each `topic` individually. `MQTT` defines three levels of QOS:
* QoS 0: At most once delivery; provides no guarantee of delivery
* QoS 1: At least once delivery
* QoS 2: Exactly once delivery; safest and slowest quality of service level

If a `client_id` is set, all topics with QOS > 0 will be [persisted in the session](https://www.hivemq.com/blog/mqtt-essentials-part-7-persistent-session-queuing-messages/), i.e. the broker will queue messages during connection losses as long as soss does not get restarted.

`json_indent` (default: -2): Configures the `JSON` indentation level, where
* -2: binary transmission as [MessagePack](https://msgpack.org/index.html)
* -1: compact text (without any whitespace)
* 0: newlines only
* N: pretty-print (with N spaces per level)

#### Authentication

`MQTT` protocol provides username and password for authentication as an application level security. The client has the option to send a username and password when it connects to an `MQTT` broker.
`username`: UTF-8 encoded string.
`password`: Binary data with a maximum of 65535 bytes.
To configure the `mosquitto` broker for username and password restrictions, please refer [here](http://www.steves-internet-guide.com/mqtt-username-password-example/). Once the broker has been configured, the `soss-mqtt` plugin has to be provided with `username` and `password` keys in the configuration file in the following manner:
```yaml
systems:
  mqtt_broker: { type: mqtt, address: "tcp://localhost:1884", username: user, password: password }
```

#### SSL / TLS

`MQTT` broker can be configured to establish an SSL/TLS connection with its clients. Two scenatios are possible:
- *Server authentication*: The client needs the digital certificate of the server. It is included in the "trust store".
- *Mutual authentication*: Both client and server are authenticated during the SSL handshake. In addition to the digital certificate of the server in a trust store, the client will need its own digital certificate and the private key used to sign its digital certificate stored in a "key store".

```yaml
systems:
  mqtt: { type: mqtt, address: "ssl://localhost:8883",
          truststore: certs/server.crt,
          keystore: certs/client.crt,
          private_key: certs/client.key}
```

`trust_store`: Server's public certificate trusted by the client.  
The following setting(s) are required if the broker has been configured for mutual authentication, as described above.  
`key_store`: Client's public certificate chain. It may also include the client's private key.  
`private_key` - If not included in `key_store`, this setting points to client's private key.  
`private_keypassword`: The password to load the client's `private_key` if encrypted.

All paths can be absolute or relative to the config file or the home directory.

#### Persistence

`soss-mqtt` will reconnect to the broker after a connection was lost, but published messages will be lost in between.
However, it is possible to buffer them and publish after the reconnection.

```yaml
systems:
    mqtt_broker:
      type: mqtt
      address: "tcp://localhost:1883"
      send_while_disconnected: true
      max_buffered_messages: 100
      persist_qos0: false
```

`send_while_disconnected` (default: false): buffer outgoing message during discnnections.

`max_buffered_messages` (default: 100): number of messages to be bufferred, additional messages will be dropped.

`persist_qos0` (default: false): buffer QOS 0 messages as well

### Topics
Both, `qos` and `json_indent`, can be specified for each indiviual topic as well, which override the system's default:

```yaml
topics:
    out:
        type: "std_msgs/String"
        route: {from: ros, to: mqtt_broker }
        mqtt_broker: { qos: 2 }
        remap: { mqtt_broker: "test/out" }
```

### Services

Service support individual settings for `qos` and `json_indent`as well, but in contrast to topics the `qos` defaults to 1.

```yaml
services:
    add_two_ints:
        type: "rospy_tutorials/AddTwoInts"
        route: { server: ros, clients: mqtt_broker }
        mqtt_broker: { qos: 0 }

```

## Mapping

`soss-mqtt` converts SOSS messages into JSON objects, which can be sent as text or [MessagePack](https://msgpack.org/) (default). To distinguish the different operations, SOSS topics are mapped differently.

### Topics

`SOSS` topics get mapped to `MQTT` topics as specified in the configuation file (topic name or remap). The message format should match up to the message definition on the target system. For example, for a topic of type [std_msgs/String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html), the message format should be


```json
{
  "data": "Hello world"
}
```

### Services
For services a dynamic mapping is used, which reflects the reference id of the call.

Service requests should be sent to `<SERVICE>/request/<REFERENCE>`, where `<SERVICE>` is the (remapped) name from the configuration and `<REFERENCE>` is the unqiue reference id. The response will be returned to `<SERVICE>/response/<REFERENCE>`.

In `soss-mqtt` each service client has a [UUID](https://en.wikipedia.org/wiki/Universally_unique_identifier) and tracks all service calls with a reference counter, so `<REFERENCE>` is `<UUID>/<COUNTER>`. This allows the use of [wildcard subscriptions](https://www.hivemq.com/blog/mqtt-essentials-part-5-mqtt-topics-best-practices/) to `<SERVICE>/response/<UUID>/#` for clients and `<SERVICE>/request/#` for servers.

Example: Service `ns1/ns2/my_service` of type type [std_srvs/SetBool](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html)

Request is sent to `ns1/ns2/my_service/request/00112233-4455-6677-8899-aabbccddeeff/0` (first call):
```json
{
  "data" : true
}
```

Response will be sent to `ns1/ns2/my_service/response/00112233-4455-6677-8899-aabbccddeeff/0`:
```json
{
  "success" : true,
  "message" : "Device enabled"
}
```
