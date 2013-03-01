#!/bin/sh

openhrp-controller-bridge \
--server-name RHEXPDController \
--out-port angle:JOINT_VALUE \
--in-port torque:JOINT_TORQUE \
--connection angle:RHEXPD0:angle \
--connection torque:RHEXPD0:torque
