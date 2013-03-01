#!/bin/sh

openhrp-controller-bridge \
--server-name RHEX_sampleController \
--out-port angle:JOINT_VALUE \
--in-port torque:JOINT_TORQUE \
--connection angle:RHEX_sample0:angle \
--connection torque:RHEX_sample0:torque
