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

#ifndef SOSS__MQTT__SRC__ENCODING_HPP
#define SOSS__MQTT__SRC__ENCODING_HPP

#include <mqtt/buffer_ref.h>
#include <soss/json/conversion.hpp>

namespace soss {
namespace mqtt {

class Encoding
{
public:
    using Ptr = std::shared_ptr<Encoding>;

    static const int JSON_VERBOSE = 4;
    static const int JSON_BINARY = -2;

    const int json_indent = JSON_BINARY;

    ::mqtt::binary_ref encode(const soss::Message &msg) const
    {
        return encode_json(soss::json::convert(msg));
    }
    soss::Message decode(const ::mqtt::binary_ref &payload, const std::string &type) const
    {
        return soss::json::convert(type, decode_json(payload));
    }
private:
    ::mqtt::binary_ref encode_json (const soss::json::Json& payload) const
    {
        if (json_indent == JSON_BINARY)
        {
            std::string out;
            nlohmann::json::to_msgpack(payload, out);
            return out;
        }
        return payload.dump(json_indent);
    }

    soss::json::Json decode_json (const ::mqtt::binary_ref &payload) const
    {
        if (!payload.empty() && payload[0] != '{')
        {
            // assume messagepack
            return nlohmann::json::from_msgpack(payload.str());
        }
        return soss::json::Json::parse(payload.str());
    }
};

}   // namespace mqtt
}   // namespace soss

#endif // SOSS__MQTT__SRC__ENCODING_HPP
