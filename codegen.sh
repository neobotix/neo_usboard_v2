#!/bin/bash

VNX_INTERFACE_DIR=${VNX_INTERFACE_DIR:-/usr/interface}

cd $(dirname "$0")

vnxcppcodegen --cleanup generated/ neo_usboard_v2 modules/ ${VNX_INTERFACE_DIR}/vnx/ pilot-base/interface/ pilot-usboard/interface/

