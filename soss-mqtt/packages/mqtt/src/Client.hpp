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

#ifndef SOSS__MQTT__SRC__CLIENT_HPP
#define SOSS__MQTT__SRC__CLIENT_HPP

#include <string>
#include <unordered_map>
#include <mqtt/async_client.h>

namespace soss {
namespace mqtt {

class Client
{
public:
    void bind(const std::string& topic_name, int qos, ::mqtt::async_client::message_handler handler)
    {
        ::mqtt::topic topic{*_mqtt_client, topic_name, qos};
        _subscriptions.emplace(topic_name, Subscription{topic, handler});
        try
        {
            topic.subscribe();
        }
        catch (const ::mqtt::exception& exc)
        {
            std::cerr << "Could not subscribe to " << topic_name << "\n";
            std::cerr << exc.what() << std::endl;
        }
    }
    bool connect(const std::string& address, const std::string &clientID,
                 ::mqtt::create_options create_opts, ::mqtt::connect_options connect_opts)
    {
        if (_mqtt_client)
        {
            return false;
        }

        auto client = std::make_shared<::mqtt::async_client>(address, clientID, create_opts);
        try
        {
            if(!client->connect(connect_opts)->wait_for(std::chrono::seconds(2))) {
                return false;
            }
            if(!clientID.empty()){
                client->disconnect()->wait();
                connect_opts.set_clean_session(false);
                if(!client->connect(connect_opts)->wait_for(std::chrono::seconds(2))) {
                    return false;
                }
            }
            client->set_message_callback([this](auto msg){
                try
                {
                    find(msg->get_topic()).handler(msg);
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << std::endl;
                    return;
                }
            });

            client->set_connected_handler([this](auto&){
                for(auto& it : _subscriptions)
                {
                    it.second.topic.subscribe();
                    std::cout << "resubscribed to topic " << it.second.topic.get_name();
                }
            });

            _mqtt_client = client;
            std::cout << "Connected to " << address << std::endl;
            return true;
        }
        catch (const ::mqtt::exception& exc)
        {
            std::cerr << exc.what() << std::endl;
        }
        return false;
    }
    void disconnect()
    {
        auto client = std::move(_mqtt_client);
        if (client)
        {
            client->disconnect()->wait_for(std::chrono::seconds(60));
        }
    }
    inline bool okay() const
    {
        return _mqtt_client != nullptr;
    }
    bool publish(const std::string& topic_name, ::mqtt::binary_ref payload,
        int qos=::mqtt::message::DFLT_QOS, bool retained=::mqtt::message::DFLT_RETAINED) {
        try
        {
            _mqtt_client->publish(topic_name, payload, qos, retained);
            return true;
        }
        catch (const ::mqtt::exception& exc)
        {
            // do not flood outout, only fir qos > 0
            if (qos > 0)
            {
                std::cerr << "Could not publish to topic '" << topic_name << "'\n";
                std::cerr << exc.what() << std::endl;
            }
        }
        return false;
    }
private:
    ::mqtt::async_client_ptr _mqtt_client;
    struct Subscription {
        ::mqtt::topic topic;
        ::mqtt::async_client::message_handler handler;
    };
    std::unordered_map<std::string, Subscription> _subscriptions;
    Subscription& find(const std::string& topic_name) {
        try
        {
            return _subscriptions.at(topic_name);
        }
        catch (const std::out_of_range&)
        {
            size_t found=topic_name.size();
            do
            {
                found = topic_name.find_last_of("/", found-1);
                auto it = _subscriptions.find(topic_name.substr(0, found) + "/#");
                if (it != _subscriptions.end()){
                    return it->second;
                }
            }
            while(found != std::string::npos);
        }
        throw std::out_of_range("No subscription for " + topic_name);
    }
};

}   // namespace mqtt
}   // namespace soss

#endif // SOSS__MQTT__SRC__CLIENT_HPP
