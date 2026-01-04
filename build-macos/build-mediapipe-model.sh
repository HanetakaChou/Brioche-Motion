#!/bin/bash

#
# Copyright (C) YuqiaoZhang(HanetakaChou)
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

set -euo pipefail

PROJECT_DIRECTORY="$(cd "$(dirname "$0")" 1>/dev/null 2>/dev/null && pwd)"  

JOBS="$(sysctl -n hw.ncpu 2>/dev/null)"

MAKE_ARGUMENT=(
  "-j${JOBS}"
)

cd "${PROJECT_DIRECTORY}"

if make -C "${PROJECT_DIRECTORY}/../build-mediapipe-model" -f "mediapipe-model.mk" "${MAKE_ARGUMENT[@]}"; then
    echo "build mediapipe-model passed"
else
    echo "build mediapipe-model failed"
    exit 1
fi

