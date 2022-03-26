
1. Open "Visualize_Tilted_Object_Avoidance.m"

2. Load obstacle data "Ob_Tilt_Rec.mat"

3. Load Missions from mission list -----||||

		"DroneY0=0_LQ=0.1.mat" - Straight towards obstacle and avoid with lower weight (10%)
		"DroneY0=0_LQ=1.0.mat" - Straight towards obstacle and avoid with lower weight (100%)
		"DroneY0=1_LQ=0.1.mat" - Left position obstacle avoidance with lower weight (10%)
		"DroneY0=1_LQ=1.0.mat" - Left position obstacle avoidance with lower weight (100%)
		"DroneY0=-1_LQ=0.1.mat" - Right position obstacle avoidance with lower weight (10%)
		"DroneY0=-1_LQ=1.0.mat" - Right position obstacle avoidance with lower weight (100%)

4. Run the simulation

Obstacle data: Ob_Tilt_Rec.mat

Obstacle Size: 1.0 x 0.4
Tilte degree: 45 degree
Location [0,0,0]
