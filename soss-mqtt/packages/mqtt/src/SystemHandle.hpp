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

#ifndef SOSS__MQTT__SRC__SYSTEM_HANDLE_HPP
#define SOSS__MQTT__SRC__SYSTEM_HANDLE_HPP

#include <string>
#include <soss/SystemHandle.hpp>

#include "Encoding.hpp"
#include "Client.hpp"

namespace soss {
namespace mqtt {

const std::string YamlAddressKey = "address";
const std::string YamlClientIDKey = "client_id";
const std::string YamlUsernameKey = "username";
const std::string YamlPasswordKey = "password";
const std::string YamlTruststoreKey = "truststore";
const std::string YamlKeystoreKey = "keystore";
const std::string YamlPrivateKeyKey = "private_key";
const std::string YamlPrivateKeyPasswordKey = "private_key_password";
const std::string YamlMaxBufferedMessagesKey = "max_buffered_messages";
const std::string YamlSendWhileDisconnectedKey = "send_while_disconnected";
const std::string YamlPersistQos0Key = "persist_qos0";
const std::string YamlQosKey = "qos";
const std::string YamlJsonIndentKey = "json_indent";

const std::string DEFAULT_CLIENT_ID = "";
const int DEFAULT_QOS = 0;
const int DEFAULT_SERVICE_QOS = 1;
const int DEFAULT_JSON_INDENT = Encoding::JSON_BINARY;
const bool DEFAULT_SEND_WHILE_DISCONNECTED = false;
const bool DEFAULT_PERSIST_QOS0 = false;
const int DEFAULT_MAX_BUFFERED_MESSAGES = 100;

class SystemHandle : public TopicSystem, public ServiceSystem
{
public:
    SystemHandle();
    ~SystemHandle();

    virtual bool configure(
        const RequiredTypes& types,
        const YAML::Node& configuration) override;

    virtual bool okay() const override;

    virtual bool spin_once() override;

    virtual std::shared_ptr<soss::TopicPublisher> advertise(
        const std::string& topic_name,
        const std::string& message_type,
        const YAML::Node& configuration) override;

    virtual bool subscribe(
        const std::string& topic_name,
        const std::string& message_type,
        soss::TopicSubscriberSystem::SubscriptionCallback callback,
        const YAML::Node& configuration) override;

    bool create_client_proxy(
        const std::string& service_name,
        const std::string& service_type,
        ServiceClientSystem::RequestCallback callback,
        const YAML::Node& configuration) override;

    std::shared_ptr<soss::ServiceProvider> create_service_proxy(
        const std::string& service_name,
        const std::string& service_type,
        const YAML::Node& configuration) override;

private:
    Client _client;
    int _qos;
    Encoding::Ptr _encoding;
    Encoding::Ptr create_encoding(const YAML::Node& configuration);
};

}   // namespace mqtt
}   // namespace soss

#endif // SOSS__MQTT__SRC__SYSTEM_HANDLE_HPP
