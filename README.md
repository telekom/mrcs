<h1 align="center">
    Multi Robot Cloud Services
</h1>

<p align="center">
    <a href="/../../commits/" title="Last Commit"><img src="https://img.shields.io/github/last-commit/telekom/repobasics?style=flat"></a>
    <a href="/../../issues" title="Open Issues"><img src="https://img.shields.io/github/issues/telekom/repobasics?style=flat"></a>
    <a href="./LICENSE" title="License"><img src="https://img.shields.io/badge/License-Apache%202.0-green.svg?style=flat"></a>
</p>

<p align="center">
  <a href="#development">Development</a> •
  <a href="#documentation">Documentation</a> •
  <a href="#support-and-feedback">Support</a> •
  <a href="#how-to-contribute">Contribute</a> •
  <a href="#licensing">Licensing</a>
</p>

The goal of this project is to combine the ROS1 ( Robot Operating System ) framework with the messaging protocol MQTT to enable ROS1 for a multi robot cloud system. 

## About this component
![brokeragearchitecture](https://user-images.githubusercontent.com/104825498/177164087-e678fc00-0527-4d1b-aaaf-1782ff86f607.PNG)


![clientarchitecture](https://user-images.githubusercontent.com/104825498/177164151-5cde4ed8-3a49-43f5-87d4-54e1eb5d6f64.PNG)

ROS message get converted to JSON with soss-json
MQTT client detects if payload is JSON or MessagePack
Format of out-going message can be configured system-wide and per interface
ROS topic are mapped to MQTT topics 1:1
ROS services have a dynamic mapping to differentiate services clients and requests
Request: <SERVICE>/request/<UUID>/<COUNTER> (Wildcard/server: <SERVICE>/request/#)
Response: <SERVICE>/response/<UUID>/<COUNTER> (Wildcard/client: <SERVICE>/response/<UUID>/#)
Server can handle any reference pattern (instead of <UUID>/<COUNTER>)


## Development

_TBD_

### Build

_TBD_

## Code of Conduct

This project has adopted the [Contributor Covenant](https://www.contributor-covenant.org/) in version 2.0 as our code of conduct. Please see the details in our [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md). All contributors must abide by the code of conduct.

## Working Language

We decided to apply _English_ as the primary project language.  

Consequently, all content will be made available primarily in English. We also ask all interested people to use English as language to create issues, in their code (comments, documentation etc.) and when you send requests to us. The application itself and all end-user facing content will be made available in other languages as needed.

## Documentation

The full documentation for the telekom can be found in _TBD_
## Support and Feedback
The following channels are available for discussions, feedback, and support requests:

| Type                     | Channel                                                |
| ------------------------ | ------------------------------------------------------ |
| **Issues**   | <a href="/../../issues/new/choose" title="General Discussion"><img src="https://img.shields.io/github/issues/telekom/repobasics?style=flat-square"></a> </a>   |
| **Other Requests**    | <a href="mailto:opensource@telekom.de" title="Email Open Source Team"><img src="https://img.shields.io/badge/email-Open%20Source%20Team-green?logo=mail.ru&style=flat-square&logoColor=white"></a>   |

## How to Contribute

Contribution and feedback is encouraged and always welcome. For more information about how to contribute, the project structure, as well as additional contribution information, see our [Contribution Guidelines](./CONTRIBUTING.md). By participating in this project, you agree to abide by its [Code of Conduct](./CODE_OF_CONDUCT.md) at all times.

## Licensing

Copyright (c) 2022 Deutsche Telekom AG.

Licensed under the **Apache License, Version 2.0** (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at https://www.apache.org/licenses/LICENSE-2.0.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the [LICENSE](./LICENSE) for the specific language governing permissions and limitations under the License.
