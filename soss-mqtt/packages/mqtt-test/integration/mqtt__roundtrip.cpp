/*
 * Copyright (C) 2021 Fraunhofer IPA
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <soss/mock/api.hpp>
#include <soss/Instance.hpp>
#include <soss/utilities.hpp>

#include <catch2/catch.hpp>

#include <iostream>

TEST_CASE("Transmit and receive test data", "[mqtt]")
{
  using namespace std::chrono_literals;

  for (const std::string config : { "roundtrip", "roundtrip_auth", "roundtrip_ssl"})
  {
    SECTION(config)
    {

      soss::InstanceHandle instance = soss::run_instance(MQTT__RESOURCES + config + ".yaml");
      REQUIRE(instance);
      std::this_thread::sleep_for(2s);

      std::promise<soss::Message> promise;
      auto future = promise.get_future();
      REQUIRE(soss::mock::subscribe("from_broker_"+config, [&](const soss::Message& message){
        promise.set_value(message);
      }));

      REQUIRE(future.wait_for(1s) == std::future_status::timeout);

      soss::Message msg_to_broker;
      msg_to_broker.type = "mqtt_test/Data";
      const int data = 42;
      msg_to_broker.data["data"] = soss::Convert<int>::make_soss_field(data);
      soss::mock::publish_message("to_broker_"+config, msg_to_broker);

      REQUIRE(future.wait_for(5s) == std::future_status::ready);
      const soss::Message result = future.get();
      REQUIRE(result.data.size() > 0);

      int data_result;
      soss::Convert<int>::from_soss_field( result.data.begin(), data_result);
      CHECK(data_result == data);

      CHECK(instance.quit().wait() == 0);
    }
  }
}

TEST_CASE("Request and receive response", "[mqtt]")
{
  using namespace std::chrono_literals;

  const std::string config = "service";

  soss::InstanceHandle instance = soss::run_instance(MQTT__RESOURCES + config + ".yaml");
  REQUIRE(instance);
  std::this_thread::sleep_for(2s);

  soss::Message request;
  request.type = "mqtt_test/AddTwoInts:request";
  const int a = 34;
  const int b = 23;
  request.data["a"] = soss::Convert<int>::make_soss_field(a);
  request.data["b"] = soss::Convert<int>::make_soss_field(b);

  soss::Message response;
  response.type = "mqtt_test/AddTwoInts:response";
  response.data["sum"] = soss::Convert<int>::make_soss_field(57);

  soss::mock::serve("to_mock_roundtrip", [&](const soss::Message& msg)
  {
    return response;
  });

  auto future = soss::mock::request(
      "to_broker_roundtrip", request, 100ms);

  REQUIRE(future.wait_for(5s) == std::future_status::ready);
  const soss::Message res = future.get();
  REQUIRE(res.data.size() > 0);

  int sum;
  soss::Convert<int>::from_soss_field( res.data.begin(), sum);
  CHECK(sum == (a + b));
  CHECK(instance.quit().wait() == 0);
}
