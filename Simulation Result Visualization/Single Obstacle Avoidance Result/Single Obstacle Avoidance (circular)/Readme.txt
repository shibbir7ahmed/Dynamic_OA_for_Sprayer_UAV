1. Open "Visualize_Tilted_Object_Avoidance.m"

2. Load obstacle data "Ob_Tilt_Rec.mat"

3. Load Missions from mission list -----||||

    	"1. DroneY0=0_LQ=0.1.mat" - Avoid with 10% liquid
		"2. DroneY0=0_LQ=0.25.mat" - Avoid with 25% liquid
		"3. DroneY0=0_LQ=0.5.mat" - Avoid with 50% liquid
		"4. DroneY0=0_LQ=0.75.mat" - Avoid with 75% liquid
		"5. DroneY0=0_LQ=0.9.mat" - Avoid with 90% liquid
		"6. DroneY0=0_LQ=1.0.mat" - Avoid with 100% liquid

4. Run Simulation

Data:
Small Circle size: radius 0.1 meter
Location [0,0,0]

When the drone's initail Y position is 1.3 (safe, doens't need to avoid)
The drone will still slow down at first, then it will return to normal fly mode after flying across the obstacle.

