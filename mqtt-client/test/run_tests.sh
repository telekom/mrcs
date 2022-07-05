#!/bin/bash
#
# mqtt-client - Wrapper for SOSS with MQTT and ROS
#
# Copyright 2021 Fraunhofer IPA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
function stop_all() {
    jobs -p | xargs -r kill &> /dev/null || true
    wait &> /dev/null
}

function test_client() {
    echo "run test (ros${ROS_VERSION:?ROS_VERSION is not set})"
    
    mqtt-client --address localhost --scope test --debug test/client.yml &

    if [ "$ROS_VERSION" = "1" ]; then
        roscore &
        while ! rostopic list &> /dev/null; do sleep 1; done
    else
        sleep 2
    fi

    if [ "$ROS_VERSION" = "1" ]; then
        rostopic pub /out std_msgs/String "data: 'message'" -r 1 &
    else
        ros2 topic pub /out std_msgs/msg/String "data: 'message'" -r 1 &
        sleep 5
    fi

    "$PYTHON_VERSION_NAME" "test/service${ROS_VERSION}.py" &

    echo "Subscribe:"
    if [ "$ROS_VERSION" = "1" ]; then
        log=$(timeout 3 rostopic echo /in)
    else
        log=$(timeout 3 ros2 topic echo /in std_msgs/msg/String)
    fi

    echo "$log"
    if ! echo "$log" | grep -q 'data: "\?message"\?'; then
        echo "test failed"
        exit 1
    fi

    echo "Call service:"
    if [ "$ROS_VERSION" = "1" ]; then
        log=$(timeout 3 rosservice call /remote_service)
    else
        log=$(timeout 3 ros2 service call /remote_service std_srvs/srv/Trigger)
    fi

    stop_all
    echo "$log"
    if ! echo "$log" | grep -q "success"; then
        echo "test failed"
        exit 1
    fi

    echo "test passed"
}

trap stop_all EXIT

mosquitto &
sleep 2
mosquitto_sub -t '#' &
test_client 
stop_all
