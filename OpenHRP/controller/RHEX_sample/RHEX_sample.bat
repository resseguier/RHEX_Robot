openhrp-controller-bridge ^
--server-name RHEX_sampleController ^
--out-port angle:JOINT_VALUE ^
--in-port torque:JOINT_TORQUE ^
--connection angle:angle ^
--connection torque:torque 

