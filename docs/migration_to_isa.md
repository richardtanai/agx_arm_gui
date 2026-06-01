> **DEPRECATED** — superseded by [spb_node_spec.md](spb_node_spec.md). Do not update this file.

We will be migrating the node to the full ISA95 tag naming

Enterprise = DMATDTS
Area = DLSU
Site = LS
Line = Mini_Factory
Cell = agx_arm_bridge
Device = piper_arm


SparkplugB naming will adjust, to maintain the ISA, we will 

GID = 
Node = DLSU
Device = LS
Metric Prefix = /Mini_Factory/agx_arm_bridge/piper_arm

Implement the minimum only

Suffix for actual SpB Tags:

/Status/State/Current
 	- Integer mapping (PackML) - PackML state codes 
 	- IDLE (3), 
	- Execute - 5, 
	- Complete 16
	- Aborted - 8 - when the robot has an error or if the robot is diconnected

/Cmd/CntrlCmd
 	0 - undefined, 
	1 = reset, 
	2 = start, 
	3 = stop,
	9 = clear, - to recover from aborted


/Alarm
/Motion - Joints
/Gripper - Gripper status



Alarms

7001	Robot	MotionTimeout	2	OOS	60 s without completion	Inspect path, restart
7002	Robot	MotionFailed	2	OOS	Trajectory aborted by controller	Check workspace, re-home
7003	Robot	PrimaryHostOffline	1	Failure	STATE/reports offline	Check Ignition gateway
7004	Robot	EStopAsserted	1	Failure	Mirror of safety E-Stop	Resolve hazard, reset
7005	Robot	GripperPartLost	2	OOS	PartPresent → false mid-cycle	Retrieve part, recover
7006	Robot	JointLimitApproach	4	Maintenance	Joint near soft limit	Plan check