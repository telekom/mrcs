//
// soss-mqtt - MQTT plugin for SOSS
//
// Copyright 2021 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#ifndef SOSS__MQTT__SRC__TOPIC_PROVIDER_HPP
#define SOSS__MQTT__SRC__TOPIC_PROVIDER_HPP

namespace soss {
namespace mqtt {

class TopicPublisher : public soss::TopicPublisher
{
public:
    using publish_t=std::function<bool(const soss::Message &data)>;
    TopicPublisher(
        publish_t pub)
    : _pub(pub)
    {
    }

    bool publish(const soss::Message& message) override
    {
        return _pub(message);
    }

private:
    publish_t _pub;
};

}   // namespace mqtt
}   // namespace soss

#endif // SOSS__MQTT__SRC__TOPIC_PROVIDER_HPP