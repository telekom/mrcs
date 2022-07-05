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

#ifndef SOSS__MQTT__SRC__SERVICE_PROVIDER_HPP
#define SOSS__MQTT__SRC__SERVICE_PROVIDER_HPP

namespace soss {
namespace mqtt {


class ServiceProvider : public ::soss::ServiceProvider
{
public:
    using publish_t=std::function<void(const std::string& ref, const soss::Message &data)>;
    ServiceProvider(publish_t pub)
        : _pub(pub), _ref(0)
    {
    }

    void call_service(
        const soss::Message& request,
        ServiceClient& client,
        std::shared_ptr<void> call_handle) override
    {
        std::string ref = std::to_string(_ref++);
        _service_request_info.emplace(ref, ServiceRequestInfo{client, std::move(call_handle)});
        _pub(ref, request);
    }

    void handle(const std::string& ref, const soss::Message &data)
    {
        auto it = _service_request_info.find(ref);
        if(it == _service_request_info.end())
        {
            std::cerr << "Received a response with an unrecognized id [" << ref << "]" << std::endl;
            return;
        }

        ServiceRequestInfo& info = it->second;
        info.client.receive_response(info.call_handle, data);
        _service_request_info.erase(it);
    }

private:
    struct ServiceRequestInfo {
        ::soss::ServiceClient& client;
        std::shared_ptr<void> call_handle;
    };
    const publish_t _pub;
    unsigned int _ref;
    std::unordered_map<std::string, ServiceRequestInfo> _service_request_info;
};


} // namespace mqtt
} // namespace soss

#endif // SOSS__MQTT__SRC__SERVICE_PROVIDER_HPP
