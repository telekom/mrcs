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

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <soss/Search.hpp>

#include "SystemHandle.hpp"

#include "TopicPublisher.hpp"
#include "ServiceProvider.hpp"
#include "ServiceClient.hpp"

namespace soss {
namespace mqtt {


std::string generate_uuid()  {
    static boost::uuids::random_generator gen;
    return boost::uuids::to_string(gen());
}

template<typename T>
bool parse_value(const YAML::Node& configuration, const std::string &name, T& value)
{
    if(const YAML::Node node = configuration[name])
    {
        value = node.as<T>();
        return true;
    }
    std::cerr << name + " not set in configuration" << std::endl;
    return false;
}

template<typename T>
T parse_value_with_default(const YAML::Node& configuration, const std::string &name, const T& default_value)
{
    return configuration[name].as<T>(default_value);
}

std::string parse_path(const YAML::Node& configuration, const std::string &name, bool *valid)
{
    std::string path;
    if(const YAML::Node node = configuration[name])
    {
        path = node.as<std::string>();
        if(!path.empty())
        {
            soss::Search ca_search = soss::Search("mqtt")
                                        .relative_to_config()
                                        .relative_to_home();
            auto found = ca_search.find_file(path);
            if(found.empty())
            {
                std::cerr << "Could not resolve file '" << path << "' for " << name <<  std::endl;
                if(valid){
                    *valid=false;
                }
                return path;
            }
            path = found;
        }
    }
    return path;
}

SystemHandle::SystemHandle()
{
}

SystemHandle::~SystemHandle()
{
    _client.disconnect();
}

Encoding::Ptr SystemHandle::create_encoding(const YAML::Node& configuration)
{
    int json_indent;
    if(_encoding)
    {
        json_indent = parse_value_with_default(configuration, YamlJsonIndentKey, _encoding->json_indent);
        if (json_indent == _encoding->json_indent)
        {
            return _encoding;
        }
    }
    else
    {
        json_indent = parse_value_with_default(configuration, YamlJsonIndentKey, DEFAULT_JSON_INDENT);
    }
    return std::make_shared<Encoding>(Encoding{json_indent});
}

bool SystemHandle::configure(
      const RequiredTypes& /*types*/,
      const YAML::Node& configuration)
{
    std::string address, clientID;
    bool config_valid = true;
    config_valid &= parse_value(configuration, YamlAddressKey, address);
    clientID = parse_value_with_default(configuration, YamlClientIDKey, DEFAULT_CLIENT_ID);
    _qos = parse_value_with_default(configuration, YamlQosKey, DEFAULT_QOS);
    _encoding = create_encoding(configuration);

    auto create_opts = ::mqtt::create_options_builder()
                  .send_while_disconnected(parse_value_with_default(configuration, YamlSendWhileDisconnectedKey, DEFAULT_SEND_WHILE_DISCONNECTED))
                  .max_buffered_messages(parse_value_with_default(configuration, YamlMaxBufferedMessagesKey, DEFAULT_MAX_BUFFERED_MESSAGES))
                  .persist_qos0(parse_value_with_default(configuration, YamlPersistQos0Key, DEFAULT_PERSIST_QOS0))
                  .finalize();

    auto sslopts = ::mqtt::ssl_options_builder()
                       .trust_store(parse_path(configuration, YamlTruststoreKey, &config_valid))
                       .key_store(parse_path(configuration, YamlKeystoreKey, &config_valid))
                       .private_key(parse_path(configuration, YamlPrivateKeyKey, &config_valid))
                       .private_keypassword(parse_value_with_default(configuration, YamlPrivateKeyPasswordKey, std::string()))
                       .error_handler([](const std::string& msg) {
                           std::cerr << "SSL Error: " << msg << std::endl;
                       })
                       .finalize();

    auto connect_opts = ::mqtt::connect_options_builder()
                  .clean_session()
                  .automatic_reconnect()
                  .user_name(parse_value_with_default(configuration, YamlUsernameKey, std::string()))
                  .password(parse_value_with_default(configuration, YamlPasswordKey, std::string()))
                  .ssl(std::move(sslopts))
                  .finalize();

    return config_valid && _client.connect(address, clientID, std::move(create_opts), std::move(connect_opts));
}

bool SystemHandle::okay() const
{
    return _client.okay();
}

bool SystemHandle::spin_once()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return _client.okay();
}

std::shared_ptr<soss::TopicPublisher> SystemHandle::advertise(
    const std::string& topic_name,
    const std::string& message_type,
    const YAML::Node& configuration)
{
    int qos = parse_value_with_default(configuration, YamlQosKey, _qos);
    if(!okay())
    {
        return nullptr;
    }
    std::cout << "Publishing to topic " << topic_name << " with type: " << message_type
                                         << " and QOS: " << qos << std::endl;

    auto encoding = create_encoding(configuration);
    return std::make_shared<TopicPublisher>([=](const soss::Message &data) {
        return this->_client.publish(topic_name, encoding->encode(std::move(data)), qos);
    });
}

bool SystemHandle::subscribe(
    const std::string& topic_name,
    const std::string& message_type,
    TopicSubscriberSystem::SubscriptionCallback callback,
    const YAML::Node& configuration)
{
    if(!okay())
    {
        return false;
    }
    int qos = parse_value_with_default(configuration, YamlQosKey, _qos);

    _client.bind(topic_name, qos, [=](auto msg){
        try
        {
            callback(_encoding->decode(msg->get_payload_ref(), message_type));
        }
        catch(const std::exception& e)
        {
            std::cerr << "Message handling failed for topic " << topic_name << std::endl;
            std::cerr << e.what() << std::endl;
            return;
        }
    });

    std::cout << "Subscribing to topic " << topic_name << " with type: " << message_type
                                         << " and QOS: " << qos << std::endl;
    return true;
}

bool SystemHandle::create_client_proxy(
    const std::string& service_name,
    const std::string& service_type,
    soss::ServiceClientSystem::RequestCallback callback,
    const YAML::Node& configuration)
{
    int qos = parse_value_with_default(configuration, YamlQosKey, DEFAULT_SERVICE_QOS);

    if(!okay())
    {
        return false;
    }

    std::string req_prefix = service_name + "/request/";
    std::string res_prefix = service_name + "/response/";
    std::string req_type = service_type + ":request";

    auto encoding = create_encoding(configuration);

    _client.bind(req_prefix+"#", qos, [=](auto msg){
        try
        {
            auto data = encoding->decode(msg->get_payload_ref(), req_type);
            std::string ref = msg->get_topic().substr(req_prefix.size());
            auto s = std::make_shared<ServiceClient>([=](const soss::Message &data){
                this->_client.publish(res_prefix + ref, encoding->encode(std::move(data)), qos);
            });
            callback(data, *s, s);
        }
        catch(const std::exception& e)
        {
            std::cerr << "Message handling failed for topic " << msg->get_topic() << std::endl;
            std::cerr << e.what() << std::endl;
            return;
        }
    });
    std::cout << "register service: " << service_name << " " << service_type << std::endl;

    return true;
}

std::shared_ptr<soss::ServiceProvider> SystemHandle::create_service_proxy(
    const std::string& service_name,
    const std::string& service_type,
    const YAML::Node& configuration)
{
    int qos = parse_value_with_default(configuration, YamlQosKey, DEFAULT_SERVICE_QOS);

    auto id = generate_uuid();
    std::string req_prefix = service_name + "/request/" + id + "/";
    std::string res_prefix = service_name + "/response/" + id + "/";
    std::string res_type = service_type + ":response";

    auto encoding = create_encoding(configuration);
    auto service_provider = std::make_shared<ServiceProvider>([=](const std::string& ref, const soss::Message &data) {
        this->_client.publish(req_prefix + ref, encoding->encode(std::move(data)), qos);
    });

    _client.bind(res_prefix + "#", qos, [=](auto msg){
        try
        {
            std::string ref = msg->get_topic().substr(res_prefix.size());
            auto data = encoding->decode(msg->get_payload_ref(), res_type);
            service_provider->handle(ref, data);
        }
        catch(const std::exception& e)
        {
            std::cerr << "Message handling failed for topic " << msg->get_topic() << std::endl;
            std::cerr << e.what() << std::endl;
            return;
        }
    });
    std::cout << "Creating ServiceProvider " << id << " for service " << service_name << " with type: " << service_type
                                         << " and QOS: " << qos << std::endl;

    return service_provider;
}

SOSS_REGISTER_SYSTEM("mqtt", soss::mqtt::SystemHandle)

}   // namespace mqtt
}   // namespace soss

