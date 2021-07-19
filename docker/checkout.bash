#!/bin/bash

# Copyright 2021 U.S. Department of Transportation, Federal Highway Administration

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# CMARA source dependencies
vcs import --input ~/src/c1t-rplidar-driver.repos ~/src/

# Other source dependencies. These would normally be install via rosdep if the
# binaries were available. The Noetic binaries are unavailable for the NVIDIA
# Jetson platforms.
vcs import --input ~/src/source-deps.repos ~/src/
