//
// mqtt-client - Wrapper for SOSS with MQTT and ROS
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

#include <cstdlib>
#include <iostream>
#include <string>
#include <experimental/filesystem>

#include <soss/Instance.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace po = boost::program_options;

std::string expand_vars(std::string str, std::map<std::string, std::string> vars)
{
    for(const auto& var: vars) {
        boost::replace_all(str, "$"+var.first, var.second);
    }
    return str;
}

int main(int argc, char* argv[])
{
    std::string address, client_id, scope, config;
    bool debug, print;

    po::options_description opts_desc("Options");
    opts_desc.add_options()
        ("help", "produce help message")
        ("address", po::value(&address), "MQTT broker address")
        ("config", po::value(&config)->required(), "CONFIG_FILE")
        ("id", po::value(&client_id), "MQTT client ID")
        ("scope", po::value(&scope), "Client scope")
        ("debug", po::bool_switch(&debug), "print soss config")
        ("print", po::bool_switch(&print), "print soss config only")
    ;
    po::positional_options_description pos_opts_desc;
    pos_opts_desc.add("config", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(opts_desc).positional(pos_opts_desc).run(), vm);
    if (vm.count("help")) {
        std::cout << "Usage: mqtt-client [options] CONFIG_FILE \n"  << opts_desc << std::endl;
        return 0;
    }

    try {
        po::notify(vm);
    }catch (const std::exception& e){
        std::cerr << e.what() << std::endl;
        std::cerr << "Usage: mqtt-client [options] CONFIG_FILE \n"  << opts_desc << std::endl;
        return 1;
    }

    const char * ros_version = std::getenv("ROS_VERSION");
    if (!ros_version) {
        std::cerr << "Could not read ROS_VERSION" << std::endl;
        return 1;
    }

    if(ros_version != std::string("1") && ros_version != std::string("2")) {
        std::cerr << "ROS_VERSION must be either 1 or 2 " << std::endl;
        return 1;
    }

    YAML::Node client_config = YAML::LoadFile(config);
    YAML::Node soss_config;

    soss_config["systems"]["mqtt"] = YAML::Clone(client_config["mqtt"]);
    soss_config["systems"]["mqtt"]["type"] = "mqtt";
    if (vm.count("address")) {
        soss_config["systems"]["mqtt"]["address"] = address;
    } else if(!soss_config["systems"]["mqtt"]["address"]) {
        std::cerr << "address needs to be set in config or via command line" << std::endl;
        std::cerr << "Usage: mqtt-client [options] CONFIG_FILE \n"  << opts_desc << std::endl;
        return 1;
    }
    if (vm.count("id")) {
        soss_config["systems"]["mqtt"]["client_id"] = client_id;
    }

    soss_config["systems"]["ros"] = YAML::Clone(client_config["ros"]);
    soss_config["systems"]["ros"]["type"] =  std::string("ros") + ros_version;

    soss_config["routes"]["ros_to_mqtt"]["from"] = "ros";
    soss_config["routes"]["ros_to_mqtt"]["to"]   = "mqtt";
    soss_config["routes"]["mqtt_to_ros"]["from"] = "mqtt";
    soss_config["routes"]["mqtt_to_ros"]["to"]   = "ros";

    soss_config["routes"]["ros_service"]["server"] = "ros";
    soss_config["routes"]["ros_service"]["clients"]   = "mqtt";
    soss_config["routes"]["mqtt_service"]["server"] = "mqtt";
    soss_config["routes"]["mqtt_service"]["clients"]   = "ros";


    auto sections = std::vector<std::tuple<std::string,std::string,std::string>>{
        // client config, soss config, route
        {"publishers", "topics", "ros_to_mqtt"},
        {"subscribers", "topics", "mqtt_to_ros"},
        {"clients", "services", "mqtt_service"},
        {"servers", "services", "ros_service"}
    };

    for(const auto& mapping: sections) {
        for(const auto& item: client_config[std::get<0>(mapping)]) {
            std::string name = item.first.as<std::string>();
            YAML::Node node = soss_config[std::get<1>(mapping)][name];
            std::string type = item.second["type"].as<std::string>("");
            if (type.empty()) {
                std::cerr << std::get<0>(mapping) << "/" << name << " does not have a valid 'type'" << std::endl;
                return 1;
            }
            node["type"]=type;
            node["route"]=std::get<2>(mapping);

            std::string remap = item.second["remap"].as<std::string>(scope.empty() ?  "" : "$scope/$name");
            if(!remap.empty()){
                const std::map<std::string, std::string> vars{{"name", name}, {"scope", scope}, {"id", client_id}};
                node["remap"]["mqtt"]=expand_vars(remap, vars);
            }
            for(auto t: {"mqtt", "ros"}){
                if (YAML::Node n =item.second[t]) {
                    node[t] = n;
                }
            }
        }
    }

    if(debug || print) std::cout << soss_config << std::endl;
    if(print) return 0;
    return soss::run_instance(soss_config, {},
                              {{"mqtt", {std::experimental::filesystem::absolute(config).parent_path()}}})
                             .wait();
}
