/*
	FILE: dynamicExploration.cpp
	-----------------------------
	Implementation of dynamic exploration
*/

#include <autonomous_flight/simulation/dynamicExploration.h>
#include <iomanip>
#include <sstream>
#include <limits>

namespace AutoFlight{
	dynamicExploration::dynamicExploration(const ros::NodeHandle& nh) : flightBase(nh){
		this->initParam();
		this->initModules();
		this->registerPub();
		this->registerMissionInterface();
		if (this->useFakeDetector_){
			// free map callback
			this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::freeMapCB, this);
		}
	}

	void dynamicExploration::initParam(){
    	// use simulation detector	
		if (not this->nh_.getParam("autonomous_flight/use_fake_detector", this->useFakeDetector_)){
			this->useFakeDetector_ = false;
			cout << "[AutoFlight]: No use fake detector param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use fake detector is set to: " << this->useFakeDetector_ << "." << endl;
		}
		
		// desired velocity
		if (not this->nh_.getParam("autonomous_flight/desired_velocity", this->desiredVel_)){
			this->desiredVel_ = 0.5;
			cout << "[AutoFlight]: No desired velocity param. Use default 0.5m/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired velocity is set to: " << this->desiredVel_ << "m/s." << endl;
		}	

		// desired acceleration
		if (not this->nh_.getParam("autonomous_flight/desired_acceleration", this->desiredAcc_)){
			this->desiredAcc_ = 2.0;
			cout << "[AutoFlight]: No desired acceleration param. Use default 2.0m/s^2." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired acceleration is set to: " << this->desiredAcc_ << "m/s^2." << endl;
		}	

		//  desired angular velocity
		if (not this->nh_.getParam("autonomous_flight/desired_angular_velocity", this->desiredAngularVel_)){
			this->desiredAngularVel_ = 0.5;
			cout << "[AutoFlight]: No angular velocity param. Use default 0.5rad/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Angular velocity is set to: " << this->desiredAngularVel_ << "rad/s." << endl;
		}	

		//  desired angular velocity
		if (not this->nh_.getParam("autonomous_flight/waypoint_stablize_time", this->wpStablizeTime_)){
			this->wpStablizeTime_ = 1.0;
			cout << "[AutoFlight]: No waypoint stablize time param. Use default 1.0s." << endl;
		}
		else{
			cout << "[AutoFlight]: Waypoint stablize time is set to: " << this->wpStablizeTime_ << "s." << endl;
		}	

		//  initial scan
		if (not this->nh_.getParam("autonomous_flight/initial_scan", this->initialScan_)){
			this->initialScan_ = false;
			cout << "[AutoFlight]: No initial scan param. Use default 1.0s." << endl;
		}
		else{
			cout << "[AutoFlight]: Initial scan is set to: " << this->initialScan_ << endl;
		}	

    	// replan time for dynamic obstacle
		if (not this->nh_.getParam("autonomous_flight/replan_time_for_dynamic_obstacles", this->replanTimeForDynamicObstacle_)){
			this->replanTimeForDynamicObstacle_ = 0.3;
			cout << "[AutoFlight]: No dynamic obstacle replan time param found. Use default: 0.3s." << endl;
		}
		else{
			cout << "[AutoFlight]: Dynamic obstacle replan time is set to: " << this->replanTimeForDynamicObstacle_ << "s." << endl;
		}	

    	// free range 
    	std::vector<double> freeRangeTemp;
		if (not this->nh_.getParam("autonomous_flight/free_range", freeRangeTemp)){
			this->freeRange_ = Eigen::Vector3d (2, 2, 1);
			cout << "[AutoFlight]: No free range param found. Use default: [2, 2, 1]m." << endl;
		}
		else{
			this->freeRange_(0) = freeRangeTemp[0];
			this->freeRange_(1) = freeRangeTemp[1];
			this->freeRange_(2) = freeRangeTemp[2];
			cout << "[AutoFlight]: Free range is set to: " << this->freeRange_.transpose() << "m." << endl;
		}	

    	// reach goal distance
		if (not this->nh_.getParam("autonomous_flight/reach_goal_distance", this->reachGoalDistance_)){
			this->reachGoalDistance_ = 0.1;
			cout << "[AutoFlight]: No reach goal distance param found. Use default: 0.1m." << endl;
		}
		else{
			cout << "[AutoFlight]: Reach goal distance is set to: " << this->reachGoalDistance_ << "m." << endl;
		}

		// initial waypoints: flat list of [x, y, z, yaw_deg] flown before exploration starts
		std::vector<double> initialWaypointsTemp;
		if (not this->nh_.getParam("autonomous_flight/initial_waypoints", initialWaypointsTemp) or initialWaypointsTemp.size() == 0){
			cout << "[AutoFlight]: No initial waypoints. Start exploration directly." << endl;
		}
		else if (initialWaypointsTemp.size() % 4 != 0){
			cout << "\033[1;31m[AutoFlight]: initial_waypoints must be a list of [x, y, z, yaw_deg] groups. Ignored.\033[0m" << endl;
		}
		else{
			for (size_t i=0; i<initialWaypointsTemp.size(); i+=4){
				this->initialWaypoints_.push_back(Eigen::Vector4d (initialWaypointsTemp[i], initialWaypointsTemp[i+1], initialWaypointsTemp[i+2], initialWaypointsTemp[i+3] * PI_const / 180.0));
			}
			cout << "[AutoFlight]: Initial waypoints (" << this->initialWaypoints_.size() << "):" << endl;
			for (Eigen::Vector4d& wp : this->initialWaypoints_){
				cout << "    [" << wp(0) << ", " << wp(1) << ", " << wp(2) << "], yaw " << wp(3) * 180.0 / PI_const << " deg" << endl;
			}
		}

		// mission interface parameters
		this->nh_.param("autonomous_flight/max_exploration_failures", this->maxExplorationFailures_, 5);
		this->nh_.param("autonomous_flight/inspect_hold_time", this->inspectHoldTime_, 3.0);
		this->nh_.param("autonomous_flight/inspect_distance", this->inspectDistance_, 2.5);
		this->nh_.param("autonomous_flight/land_clearance", this->landClearance_, 0.4);
		this->nh_.param<std::string>("autonomous_flight/land_topic", this->landTopic_, "/CERLAB/quadcopter/land");

		// % explored is measured over this region (default: the DEP exploration region)
		std::vector<double> regionMin, regionMax;
		if (not this->nh_.getParam("autonomous_flight/progress_region_min", regionMin) or not this->nh_.getParam("autonomous_flight/progress_region_max", regionMax)){
			this->nh_.getParam("DEP/global_region_min", regionMin);
			this->nh_.getParam("DEP/global_region_max", regionMax);
		}
		if (regionMin.size() == 3 and regionMax.size() == 3){
			this->progressRegionMin_ = Eigen::Vector3d (regionMin[0], regionMin[1], regionMin[2]);
			this->progressRegionMax_ = Eigen::Vector3d (regionMax[0], regionMax[1], regionMax[2]);
		}
		else{
			this->progressRegionMin_.setZero();
			this->progressRegionMax_.setZero();
		}
		cout << "[AutoFlight]: Progress region: [" << this->progressRegionMin_.transpose() << "] to [" << this->progressRegionMax_.transpose() << "]" << endl;
	}

	void dynamicExploration::initModules(){
		// initialize map
		if (this->useFakeDetector_){
			// initialize fake detector
			this->detector_.reset(new onboardDetector::fakeDetector (this->nh_));	
			this->map_.reset(new mapManager::dynamicMap (this->nh_, false));
		}
		else{
			this->map_.reset(new mapManager::dynamicMap (this->nh_));
		}

		// initialize exploration planner
		this->expPlanner_.reset(new globalPlanner::DEP (this->nh_));
		this->expPlanner_->setMap(this->map_);
		this->expPlanner_->loadVelocity(this->desiredVel_, this->desiredAngularVel_);

		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->nh_));
		this->polyTraj_->setMap(this->map_);
		this->polyTraj_->updateDesiredVel(this->desiredVel_);
		this->polyTraj_->updateDesiredAcc(this->desiredAcc_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->map_);
		this->bsplineTraj_->updateMaxVel(this->desiredVel_);
		this->bsplineTraj_->updateMaxAcc(this->desiredAcc_);		
	}

	void dynamicExploration::registerCallback(){
		// fly the initial waypoints first (current pose is the path start, like a DEP path)
		if (this->initialWaypoints_.size() != 0){
			nav_msgs::Path initialPath;
			geometry_msgs::PoseStamped start;
			start.pose = this->odom_.pose.pose;
			initialPath.poses.push_back(start);
			for (Eigen::Vector4d& wp : this->initialWaypoints_){
				geometry_msgs::PoseStamped ps;
				ps.header.frame_id = "map";
				ps.pose.position.x = wp(0);
				ps.pose.position.y = wp(1);
				ps.pose.position.z = wp(2);
				ps.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, wp(3));
				initialPath.poses.push_back(ps);
			}
			this->flyPath(initialPath, NUDGE);
			this->logEvent("Flying " + std::to_string(this->initialWaypoints_.size()) + " entry waypoints");
		}
		else{
			this->missionState_ = EXPLORE;
			this->explorationReplan_ = true;
			this->logEvent("Exploring");
		}

		// initialize exploration planner replan in another thread
		this->exploreReplanWorker_ = std::thread(&dynamicExploration::exploreReplan, this);
		this->exploreReplanWorker_.detach();

		// exploration callback
		// this->explorationTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicExploration::explorationCB, this);

		// planner callback
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.02), &dynamicExploration::plannerCB, this);

		// replan check timer
		this->replanCheckTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::replanCheckCB, this);
		
		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::trajExeCB, this);
	
		// visualization execution callabck
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicExploration::visCB, this);

		// mark the space the robot flies through as free (the tilted lidar never observes the cone under the robot)
		this->freeRobotRegionTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicExploration::freeRobotRegionCB, this);
	}

	void dynamicExploration::freeRobotRegionCB(const ros::TimerEvent&){
		Eigen::Vector3d robotSize;
		this->map_->getRobotSize(robotSize);
		Eigen::Vector3d pos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		this->map_->freeRegion(pos - robotSize/2, pos + robotSize/2);
	}

	void dynamicExploration::registerPub(){
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/poly_traj", 1000);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/pwl_trajectory", 1000);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/bspline_trajectory", 1000);
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/input_trajectory", 1000);
	}

	void dynamicExploration::explorationCB(const ros::TimerEvent&){
		if (this->explorationReplan_){
			this->expPlanner_->setMap(this->map_);
			ros::Time startTime = ros::Time::now();
			bool replanSuccess = this->expPlanner_->makePlan();
			if (replanSuccess){
				this->waypoints_ = this->expPlanner_->getBestPath();
				this->newWaypoints_ = true;
				this->waypointIdx_ = 1;
				this->explorationReplan_ = false;
			}
			ros::Time endTime = ros::Time::now();
			cout << "[AutoFlight]: DEP planning time: " << (endTime - startTime).toSec() << "s." << endl;			
		}
	}

	void dynamicExploration::plannerCB(const ros::TimerEvent&){
		// cout << "in planner callback" << endl;

		if (this->replan_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useFakeDetector_){
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			else{ 
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}			
			nav_msgs::Path inputTraj;
			std::vector<Eigen::Vector3d> startEndConditions;
			this->getStartEndConditions(startEndConditions); 
			// double initTs = this->bsplineTraj_->getInitTs();

			// generate new trajectory
			nav_msgs::Path simplePath;
			geometry_msgs::PoseStamped pStart, pGoal;
			pStart.pose = this->odom_.pose.pose;
			pGoal = this->goal_;
			simplePath.poses = {pStart, pGoal};
			this->pwlTraj_->updatePath(simplePath, false);
			this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
			// if (not this->trajectoryReady_){
			// 	// generate new trajectory
			// 	nav_msgs::Path simplePath;
			// 	geometry_msgs::PoseStamped pStart, pGoal;
			// 	pStart.pose = this->odom_.pose.pose;
			// 	pGoal = this->goal_;
			// 	simplePath.poses = {pStart, pGoal};
			// 	this->pwlTraj_->updatePath(simplePath, false);
			// 	this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
			// }
			// else{
			// 	Eigen::Vector3d bsplineLastPos = this->trajectory_.at(this->trajectory_.getDuration());
			// 	geometry_msgs::PoseStamped lastPs; lastPs.pose.position.x = bsplineLastPos(0); lastPs.pose.position.y = bsplineLastPos(1); lastPs.pose.position.z = bsplineLastPos(2);
			// 	Eigen::Vector3d goalPos (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
			// 	// if ((bsplineLastPos - goalPos).norm() >= 0.1){
			// 	nav_msgs::Path inputPWLTraj;
			// 	nav_msgs::Path simplePath;
			// 	geometry_msgs::PoseStamped pStart, pGoal;
			// 	pStart = lastPs;
			// 	pGoal = this->goal_;
			// 	simplePath.poses = {pStart, pGoal};
			// 	this->pwlTraj_->updatePath(simplePath, false);
			// 	this->pwlTraj_->makePlan(inputPWLTraj, this->bsplineTraj_->getControlPointDist());


			// 	nav_msgs::Path adjustedInputCombinedTraj;
			// 	bool satisfyDistanceCheck = false;
			// 	double dtTemp = initTs;
			// 	double finalTimeTemp;
			// 	ros::Time startTime = ros::Time::now();
			// 	ros::Time currTime;
			// 	while (ros::ok()){
			// 		currTime = ros::Time::now();
			// 		if ((currTime - startTime).toSec() >= 0.05){
			// 			cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 			break;
			// 		}							
			// 		nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 		nav_msgs::Path inputCombinedTraj;
			// 		inputCombinedTraj.poses = inputRestTraj.poses;
			// 		for (size_t i=1; i<inputPWLTraj.poses.size(); ++i){
			// 			inputCombinedTraj.poses.push_back(inputPWLTraj.poses[i]);
			// 		}
					
			// 		satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputCombinedTraj, adjustedInputCombinedTraj, dtTemp, finalTimeTemp);
			// 		if (satisfyDistanceCheck) break;
					
			// 		dtTemp *= 0.8; // magic number 0.8
			// 	}
			// 	inputTraj = adjustedInputCombinedTraj;
			// 	// }
			// 	// else{
			// 	// 	nav_msgs::Path adjustedInputRestTraj;
			// 	// 	bool satisfyDistanceCheck = false;
			// 	// 	double dtTemp = initTs;
			// 	// 	double finalTimeTemp;
			// 	// 	ros::Time startTime = ros::Time::now();
			// 	// 	ros::Time currTime;
			// 	// 	while (ros::ok()){
			// 	// 		currTime = ros::Time::now();
			// 	// 		if ((currTime - startTime).toSec() >= 0.05){
			// 	// 			cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 	// 			break;
			// 	// 		}
			// 	// 		nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 	// 		satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputRestTraj, adjustedInputRestTraj, dtTemp, finalTimeTemp);
			// 	// 		if (satisfyDistanceCheck) break;
						
			// 	// 		dtTemp *= 0.8;
			// 	// 	}
			// 	// 	inputTraj = adjustedInputRestTraj;					
			// 	// }

			// }

			

			this->inputTrajMsg_ = inputTraj;
			bool updateSuccess = this->bsplineTraj_->updatePath(inputTraj, startEndConditions);
			if (obstaclesPos.size() != 0 and updateSuccess){
				this->bsplineTraj_->updateDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			if (updateSuccess){
				nav_msgs::Path bsplineTrajMsgTemp;
				bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
					this->trajStartTime_ = ros::Time::now();
					this->trajTime_ = 0.0; // reset trajectory time
					this->trajectory_ = this->bsplineTraj_->getTrajectory();

					// optimize time
					// ros::Time timeOptStartTime = ros::Time::now();
					// this->timeOptimizer_->optimize(this->trajectory_, this->desiredVel_, this->desiredAcc_, 0.1);
					// ros::Time timeOptEndTime = ros::Time::now();
					// cout << "[AutoFlight]: Time optimizatoin spends: " << (timeOptEndTime - timeOptStartTime).toSec() << "s." << endl;

					this->trajectoryReady_ = true;
					this->replan_ = false;
					cout << "\033[1;32m[AutoFlight]: Trajectory generated successfully.\033[0m " << endl;
				}
				else{
					// if the current trajectory is still valid, then just ignore this iteration
					// if the current trajectory/or new goal point is assigned is not valid, then just stop
					if (this->hasCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails." << endl;
						this->replan_ = false;
						this->onPathFinished(false);
					}
					else if (this->hasDynamicCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails. Replan for dynamic obstacles." << endl;
						this->replan_ = true;
					}
					else{
						if (this->trajectoryReady_){
							cout << "[AutoFlight]: Trajectory fail. Use trajectory from previous iteration." << endl;
							this->replan_ = false;
						}
						else{
							cout << "[AutoFlight]: Unable to generate a feasible trajectory." << endl;
							this->replan_ = false;
							this->onPathFinished(false);
						}
					}
				}
			}
			else{
				this->trajectoryReady_ = false;
				this->stop();
				this->replan_ = false;
				cout << "[AutoFlight]: Goal is not valid. Stop." << endl;
				this->onPathFinished(false);
			}


		}
	}

	void dynamicExploration::replanCheckCB(const ros::TimerEvent&){
		/*
			Replan if
			1. collision detected
			2. new goal point assigned
			3. fixed distance
		*/

		// take over a path planned by the planning thread (exploration, return home, inspection)
		this->applyPendingPath();

		// landing: cut the motors once the robot has settled (target reached or resting on the ground)
		if (this->missionState_ == LAND){
			double vz = this->odom_.twist.twist.linear.z;
			if (vz < -0.1) this->descentStarted_ = true;
			if (not this->descentStarted_ or std::abs(vz) > 0.05 or this->currPos()(2) - this->landTarget_(2) < 0.15){
				this->touchdownCheckTime_ = ros::Time::now();
			}
			else if ((ros::Time::now() - this->touchdownCheckTime_).toSec() > 1.5){
				this->touchdown(); // stuck above the target: resting on unmapped ground
				return;
			}
		}

		if (this->newWaypoints_){
			this->replan_ = false;
			this->trajectoryReady_ = false;
			double yaw = atan2(this->waypoints_.poses[1].pose.position.y - this->odom_.pose.pose.position.y, this->waypoints_.poses[1].pose.position.x - this->odom_.pose.pose.position.x);
			// cout << "[AutoFlight]: Go to next waypoint. Press ENTER to continue rotation." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();
			this->moveToOrientation(yaw, this->desiredAngularVel_);
			// cout << "[AutoFlight]: Press ENTER to move forward." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();		
			this->replan_ = true;
			this->newWaypoints_ = false;
			if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
				this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			}
			++this->waypointIdx_;
			cout << "[AutoFlight]: Replan for new waypoints." << endl; 

			return;
		}

		// if (this->isReach(this->goal_, 0.1, false) and this->waypointIdx_ <= int(this->waypoints_.poses.size())){
		// cout << "outside the if" << endl;
		// cout << "waypoints size: " << this->waypoints_.poses.size() << endl;
		// cout << "current waypoint idx: " << this->waypointIdx_ << endl;
		if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, false) and this->waypointIdx_ <= int(this->waypoints_.poses.size())){
			// cout << "1" << endl;
			// when reach current goal point, reset replan and trajectory ready
			this->replan_ = false;
			this->trajectoryReady_ = false;
			// cout << "[AutoFlight]: Go to next waypoint. Press ENTER to continue rotation." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();
			cout << "[AutoFlight]: Rotate and replan..." << endl;
			geometry_msgs::Quaternion quat = this->goal_.pose.orientation;
			double yaw = AutoFlight::rpy_from_quaternion(quat);
			this->waitTime(this->wpStablizeTime_);
			this->moveToOrientation(yaw, this->desiredAngularVel_);
			cout << "[AutoFlight]: Finish rotation." << endl;
			// cout << "[AutoFlight]: Press ENTER to move forward." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();	

			// change current goal
			if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
				this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			}
			bool pathFinished = this->waypointIdx_ + 1 > int(this->waypoints_.poses.size());
			if (pathFinished){
				this->replan_ = false;
			}
			else{
				cout << "[AutoFlight]: Start planning for next waypoint." << endl;
				this->replan_ = true;
			}
			++this->waypointIdx_;
			this->trajectoryReady_ = false;
			if (pathFinished){
				this->onPathFinished(true);
			}
			return;
		}
		else if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, true) and (this->replan_ or this->trajectoryReady_)){
			this->replan_ = false;
			this->trajectoryReady_ = false;
			this->onPathFinished(true);
			return;
		}

		if (this->waypoints_.poses.size() != 0){
			if (not this->isGoalValid() and (this->replan_ or this->trajectoryReady_)){
				this->replan_ = false;
				this->trajectoryReady_ = false;
				cout << "[AutoFlight]: Current goal is invalid." << endl;
				this->onPathFinished(false);
				return;
			}
		}

		// if (this->reachExplorationGoal()){
		// 	this->replan_ = false;
		// 	this->trajectoryReady_ = false;
		// 	geometry_msgs::Quaternion quat = this->waypoints_.poses.back().pose.orientation;
		// 	double yaw = AutoFlight::rpy_from_quaternion(quat);
		// 	cout << "[AutoFlight]: Reach exploration goal. Rotate and replan..." << endl;
		// 	this->moveToOrientation(yaw, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Finish rotation. Start to replan." << endl;
		// 	this->replan_ = true;
		// 	return;
		// }

		if (this->trajectoryReady_){
			// only exploration goals must be known free with a safety margin (DEP check); entry, return, inspection
			// and landing goals may lie in unobserved space (e.g. the lidar's blind cone), so only reject them when occupied
			if (this->missionState_ != EXPLORE){
				if (not this->isGoalValid()){
					this->trajectoryReady_ = false;
					this->replan_ = false;
					this->stop();
					cout << "[AutoFlight]: The goal of the current leg is occupied." << endl;
					this->onPathFinished(false);
					return;
				}
			}
			else if (not this->expPlanner_->isPosValid(this->trajectory_.at(this->trajectory_.getDuration()))){
				this->trajectoryReady_ = false;
				this->replan_ = false;
				this->stop();
				cout << "[AutoFlight]: The goal of the current local trajectory is not safe." << endl;
				this->onPathFinished(false);
				return;
			}

			if (this->hasCollision()){ // if trajectory not ready, do not replan
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for collision." << endl;
				return;
			}

			// replan for dynamic obstacles
			if (this->computeExecutionDistance() >= 0.3 and this->hasDynamicCollision()){
			// if (this->hasDynamicObstacle()){
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for dynamic obstacles." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 1.5 and AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) >= 3){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 0.3 and this->replanForDynamicObstacle()){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan for dynamic obstacles." << endl;
				return;
			}
		}	
	}

	void dynamicExploration::trajExeCB(const ros::TimerEvent&){
		if (this->trajectoryReady_){
			ros::Time currTime = ros::Time::now();
			double realTime = (currTime - this->trajStartTime_).toSec();
			this->trajTime_ = this->bsplineTraj_->getLinearReparamTime(realTime);
			double linearReparamFactor = this->bsplineTraj_->getLinearFactor();
			Eigen::Vector3d pos = this->trajectory_.at(this->trajTime_);
			Eigen::Vector3d vel = this->trajectory_.getDerivative().at(this->trajTime_) * linearReparamFactor;
			Eigen::Vector3d acc = this->trajectory_.getDerivative().getDerivative().at(this->trajTime_) * pow(linearReparamFactor, 2);
			double endTime = this->trajectory_.getDuration()/linearReparamFactor;

			double leftTime = endTime - realTime; 
			tracking_controller::Target target;
			if (leftTime <= 0.0){ // zero vel and zero acc if close to
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = 0.0;
				target.velocity.y = 0.0;
				target.velocity.z = 0.0;
				target.acceleration.x = 0.0;
				target.acceleration.y = 0.0;
				target.acceleration.z = 0.0;
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				this->updateTargetWithState(target);						
			}
			else{
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = vel(0);
				target.velocity.y = vel(1);
				target.velocity.z = vel(2);
				target.acceleration.x = acc(0);
				target.acceleration.y = acc(1);
				target.acceleration.z = acc(2);
				this->updateTargetWithState(target);						
			}
		}
	}

	void dynamicExploration::visCB(const ros::TimerEvent&){
		if (this->polyTrajMsg_.poses.size() != 0){
			this->polyTrajPub_.publish(this->polyTrajMsg_);
		}
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		}
		if (this->bsplineTrajMsg_.poses.size() != 0){
			this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
		}
		if (this->inputTrajMsg_.poses.size() != 0){
			this->inputTrajPub_.publish(this->inputTrajMsg_);
		}
	}

	void dynamicExploration::freeMapCB(const ros::TimerEvent&){
		std::vector<onboardDetector::box3D> obstacles;
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		this->detector_->getObstacles(obstacles);
		double fov = 1.57;
		for (onboardDetector::box3D ob: obstacles){
			if (this->detector_->isObstacleInSensorRange(ob, fov)){
				Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-0.3, ob.y-ob.y_width/2-0.3, ob.z);
				Eigen::Vector3d upperBound (ob.x+ob.x_width/2+0.3, ob.y+ob.y_width/2+0.3, ob.z+ob.z_width+0.3);
				freeRegions.push_back(std::make_pair(lowerBound, upperBound));
			}
		}
		this->map_->updateFreeRegions(freeRegions);
		this->map_->freeRegions(freeRegions);
	}

	void dynamicExploration::run(){
		// wait for the start command (mission GUI, or: rosservice call /exploration/start)
		cout << "\033[1;32m[AutoFlight]: Ready. Waiting for the start command (mission GUI or `rosservice call /exploration/start`).\033[0m" << endl;
		this->logEvent("Ready: waiting for Start");
		ros::Rate waitRate (20);
		while (ros::ok() and not this->startRequested_){
			ros::spinOnce();
			waitRate.sleep();
		}
		if (not ros::ok()) return;

		this->missionStartTime_ = ros::Time::now();
		this->homeGroundZ_ = this->odom_.pose.pose.position.z;
		this->homePos_ = Eigen::Vector3d (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->takeoffHgt_);
		this->missionState_ = TAKEOFF;
		this->logEvent("Taking off");
		this->takeoff();
		this->logEvent("Takeoff complete");

		int temp1 = system("mkdir ~/rosbag_exploration_info &");
		int temp2 = system("mv ~/rosbag_exploration_info/exploration_info.bag ~/rosbag_exploration_info/previous.bag &");
		int temp3 = system("rosbag record -O ~/rosbag_exploration_info/exploration_info.bag /camera/aligned_depth_to_color/image_raw_t /camera/color/image_raw_t /dynamic_map/inflated_voxel_map_t /onboard_detector/dynamic_bboxes /mavros/local_position/pose /dynamicExploration/bspline_trajectory /mavros/setpoint_position/local /tracking_controller/target_pose /dep/best_paths /dep/roadmap /dep/candidate_paths /dep/best_paths /dep/frontier_regions /dynamic_map/2D_occupancy_map __name:=exploration_bag_info &");
		if (temp1==-1 or temp2==-1 or temp3==-1){
			cout << "[AutoFlight]: Recording fails." << endl;
		}

		this->initExplore();

		this->registerCallback();
	}

	void dynamicExploration::initExplore(){
		// set start region to be free
		// Eigen::Vector3d range (2.0, 2.0, 1.0);
		Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d c1 = startPos - this->freeRange_;
		Eigen::Vector3d c2 = startPos + this->freeRange_;
		this->map_->freeRegion(c1, c2);
		cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << this->freeRange_.transpose() << endl;

		if (this->initialScan_){
			cout << "[AutoFlight]: Start initial scan..." << endl;
			this->moveToOrientation(-PI_const/2, this->desiredAngularVel_);
						
			this->moveToOrientation(-PI_const, this->desiredAngularVel_);

			this->moveToOrientation(PI_const/2, this->desiredAngularVel_);
			
			this->moveToOrientation(0, this->desiredAngularVel_);
			cout << "[AutoFlight]: End initial scan." << endl; 
		}		
	}

	void dynamicExploration::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions){
		/*	
			1. start velocity
			2. start acceleration (set to zero)
			3. end velocity
			4. end acceleration (set to zero) 
		*/

		Eigen::Vector3d currVel = this->currVel_;
		Eigen::Vector3d currAcc = this->currAcc_;
		Eigen::Vector3d endVel (0.0, 0.0, 0.0);
		Eigen::Vector3d endAcc (0.0, 0.0, 0.0);

		// if (not this->trajectoryReady_){
		// 	double yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		// 	Eigen::Vector3d direction (cos(yaw), sin(yaw), 0.0);
		// 	currVel = this->desiredVel_ * direction;
		// 	currAcc = this->desiredAcc_ * direction;
		// }
		startEndConditions.push_back(currVel);
		startEndConditions.push_back(endVel);
		startEndConditions.push_back(currAcc);
		startEndConditions.push_back(endAcc);
	}

	bool dynamicExploration::hasCollision(){
		if (this->trajectoryReady_){
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
				Eigen::Vector3d p = this->trajectory_.at(t);
				bool hasCollision = this->map_->isInflatedOccupied(p);
				if (hasCollision){
					return true;
				}
			}
		}
		return false;
	}

	bool dynamicExploration::hasDynamicCollision(){
		if (this->trajectoryReady_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useFakeDetector_){
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			else{ 
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}

			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
				Eigen::Vector3d p = this->trajectory_.at(t);
				
				for (size_t i=0; i<obstaclesPos.size(); ++i){
					Eigen::Vector3d ob = obstaclesPos[i];
					Eigen::Vector3d size = obstaclesSize[i];
					Eigen::Vector3d lowerBound = ob - size/2;
					Eigen::Vector3d upperBound = ob + size/2;
					if (p(0) >= lowerBound(0) and p(0) <= upperBound(0) and
						p(1) >= lowerBound(1) and p(1) <= upperBound(1) and
						p(2) >= lowerBound(2) and p(2) <= upperBound(2)){
						return true;
					}					
				}
			}
		}
		return false;
	}

	void dynamicExploration::exploreReplan(){
		// set start region to be free
		// Eigen::Vector3d range (2.0, 2.0, 1.0);
		// Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		// Eigen::Vector3d c1 = startPos - range;
		// Eigen::Vector3d c2 = startPos + range;
		// this->map_->freeRegion(c1, c2);
		// cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << range.transpose() << endl;

		// if (this->initialScan_){
		// 	cout << "[AutoFlight]: Start initial scan..." << endl;
		// 	this->moveToOrientation(-PI_const/2, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();
						
		// 	this->moveToOrientation(-PI_const, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();

		// 	this->moveToOrientation(PI_const/2, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();
			
		// 	this->moveToOrientation(0, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: End initial scan." << endl;
		// }

		// planning thread: owns DEP. Plans exploration paths when a path is needed and handles
		// return home / inspection requests (they need roadmap paths, which can take a few seconds)
		int failCount = 0;
		while (ros::ok()){
			ros::Duration(0.05).sleep();

			int command = this->pendingCommand_.exchange(CMD_NONE);
			if (command == CMD_RETURN_HOME){
				this->expPlanner_->setMap(this->map_);
				nav_msgs::Path path;
				if (this->expPlanner_->findPathToPoint(this->homePos_, path)){
					path.poses.back().pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, this->currYaw());
					this->setPendingPath(path, RETURN);
					this->logEvent("Returning home (" + std::to_string(int(path.poses.size()) - 1) + " legs)");
				}
				else{
					// no roadmap route: climb to the home height first, then fly across (the pit is open above)
					Eigen::Vector3d p = this->currPos();
					Eigen::Vector3d climb (p(0), p(1), this->homePos_(2));
					this->setPendingPath(this->makePath({p, climb, this->homePos_}, this->currYaw()), RETURN);
					this->logEvent("Returning home: no roadmap route, climbing straight up first");
				}
				continue;
			}
			else if (command == CMD_INSPECT){
				Eigen::Vector3d target = this->inspectTarget_;
				Eigen::Vector3d viewpoint;
				if (not this->findInspectViewpoint(target, viewpoint)){
					this->logEvent("Inspect: no safe viewpoint near the selected point");
					if (this->missionState_ == EXPLORE) this->explorationReplan_ = true;
					continue;
				}
				double yawToTarget = atan2(target(1) - viewpoint(1), target(0) - viewpoint(0));
				this->expPlanner_->setMap(this->map_);
				nav_msgs::Path path;
				if (not this->expPlanner_->findPathToPoint(viewpoint, path)){
					this->logEvent("Inspect: no route to a viewpoint for the selected point");
					if (this->missionState_ == EXPLORE) this->explorationReplan_ = true;
					continue;
				}
				path.poses.back().pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, yawToTarget);
				this->setPendingPath(path, INSPECT);
				std::ostringstream text;
				text << std::fixed << std::setprecision(1) << "Inspecting (" << target(0) << ", " << target(1) << ", " << target(2) << ")";
				this->logEvent(text.str());
				continue;
			}

			if (this->missionState_ != EXPLORE or not this->explorationReplan_) continue;

			this->expPlanner_->setMap(this->map_);
			ros::Time startTime = ros::Time::now();
			bool replanSuccess = this->expPlanner_->makePlan();
			double planTime = (ros::Time::now() - startTime).toSec();
			this->frontierNum_ = this->expPlanner_->getFrontierNum();
			cout << "[AutoFlight]: DEP planning time: " << planTime << "s." << endl;
			if (this->missionState_ != EXPLORE or this->pendingCommand_ != CMD_NONE) continue; // a command arrived while planning

			if (replanSuccess){
				failCount = 0;
				this->explorationReplan_ = false;
				this->setPendingPath(this->expPlanner_->getBestPath(), EXPLORE);
			}
			else{
				++failCount;
				if (this->frontierNum_ == 0 or failCount >= this->maxExplorationFailures_){
					failCount = 0;
					this->explorationReplan_ = false;
					this->logEvent(this->frontierNum_ == 0 ? "Exploration complete: no frontiers left" : "Exploration complete: no reachable frontiers left");
					this->pendingCommand_ = CMD_RETURN_HOME;
				}
				else{
					this->logEvent("No reachable frontier, retrying (" + std::to_string(failCount) + "/" + std::to_string(this->maxExplorationFailures_) + ")");
					ros::Duration(1.0).sleep(); // let the map grow before retrying
				}
			}
		}
	}

	void dynamicExploration::registerMissionInterface(){
		this->startSrv_ = this->nh_.advertiseService("/exploration/start", &dynamicExploration::startSrvCB, this);
		this->pauseSrv_ = this->nh_.advertiseService("/exploration/pause", &dynamicExploration::pauseSrvCB, this);
		this->resumeSrv_ = this->nh_.advertiseService("/exploration/resume", &dynamicExploration::resumeSrvCB, this);
		this->returnHomeSrv_ = this->nh_.advertiseService("/exploration/return_home", &dynamicExploration::returnHomeSrvCB, this);
		this->landSrv_ = this->nh_.advertiseService("/exploration/land", &dynamicExploration::landSrvCB, this);
		this->inspectSub_ = this->nh_.subscribe("/exploration/inspect", 1, &dynamicExploration::inspectCB, this);
		this->statusPub_ = this->nh_.advertise<std_msgs::String>("/exploration/status", 10);
		this->landPub_ = this->nh_.advertise<std_msgs::Empty>(this->landTopic_, 1);
		this->statusTimer_ = this->nh_.createTimer(ros::Duration(0.5), &dynamicExploration::statusCB, this);
	}

	bool dynamicExploration::startSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
		if (this->missionState_ != IDLE){
			res.success = false;
			res.message = "Mission already started (" + stateName(this->missionState_) + ")";
			return true;
		}
		this->startRequested_ = true;
		res.success = true;
		res.message = "Starting";
		return true;
	}

	bool dynamicExploration::pauseSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
		int state = this->missionState_;
		if (state != NUDGE and state != EXPLORE and state != INSPECT and state != RETURN and state != LAND){
			res.success = false;
			res.message = "Nothing to pause (" + stateName(state) + ")";
			return true;
		}
		// keep the rest of the current leg so Resume can continue it
		this->pausedRemainingPath_.poses.clear();
		int goalIdx = this->newWaypoints_ ? 1 : this->waypointIdx_ - 1;
		for (int i=std::max(goalIdx, 1); i<int(this->waypoints_.poses.size()); ++i){
			this->pausedRemainingPath_.poses.push_back(this->waypoints_.poses[i]);
		}
		{
			std::lock_guard<std::mutex> lock (this->pendingPathMutex_);
			this->pendingPathReady_ = false;
		}
		this->stateBeforePause_ = state;
		this->missionState_ = PAUSED;
		this->explorationReplan_ = false;
		this->newWaypoints_ = false;
		this->replan_ = false;
		this->trajectoryReady_ = false;
		this->waypoints_.poses.clear();
		this->stop(); // hover here
		this->logEvent("Paused");
		res.success = true;
		res.message = "Paused";
		return true;
	}

	bool dynamicExploration::resumeSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
		if (this->missionState_ != PAUSED){
			res.success = false;
			res.message = "Not paused (" + stateName(this->missionState_) + ")";
			return true;
		}
		int state = this->stateBeforePause_;
		if (state == RETURN and this->pausedRemainingPath_.poses.size() == 0){
			this->missionState_ = RETURN; // paused while the way home was being planned
			this->pendingCommand_ = CMD_RETURN_HOME;
		}
		else if (state == EXPLORE or this->pausedRemainingPath_.poses.size() == 0){
			this->missionState_ = EXPLORE;
			this->explorationReplan_ = true;
		}
		else{
			nav_msgs::Path path;
			geometry_msgs::PoseStamped start;
			start.pose = this->odom_.pose.pose;
			path.poses.push_back(start);
			path.poses.insert(path.poses.end(), this->pausedRemainingPath_.poses.begin(), this->pausedRemainingPath_.poses.end());
			this->flyPath(path, state);
		}
		this->logEvent("Resumed: " + stateName(this->missionState_));
		res.success = true;
		res.message = "Resumed";
		return true;
	}

	bool dynamicExploration::returnHomeSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
		int state = this->missionState_;
		if (state == IDLE or state == TAKEOFF or state == LAND or state == LANDED){
			res.success = false;
			res.message = "Cannot return home now (" + stateName(state) + ")";
			return true;
		}
		// hover while the way home is planned, so the current leg cannot finish and be taken for "home reached"
		{
			std::lock_guard<std::mutex> lock (this->pendingPathMutex_);
			this->pendingPathReady_ = false;
		}
		this->explorationReplan_ = false;
		this->newWaypoints_ = false;
		this->replan_ = false;
		this->trajectoryReady_ = false;
		this->waypoints_.poses.clear();
		this->stop();
		this->missionState_ = RETURN;
		this->returnRetries_ = 0;
		this->pendingCommand_ = CMD_RETURN_HOME;
		this->logEvent("Return home requested");
		res.success = true;
		res.message = "Planning the way home";
		return true;
	}

	bool dynamicExploration::landSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
		int state = this->missionState_;
		if (state == IDLE or state == TAKEOFF or state == LAND or state == LANDED){
			res.success = false;
			res.message = "Cannot land now (" + stateName(state) + ")";
			return true;
		}
		this->pendingCommand_ = CMD_NONE;
		this->startLanding();
		res.success = true;
		res.message = "Landing";
		return true;
	}

	void dynamicExploration::inspectCB(const geometry_msgs::PointStampedConstPtr& target){
		int state = this->missionState_;
		if (state != EXPLORE and state != NUDGE and state != INSPECT and state != PAUSED){
			this->logEvent("Inspect ignored while " + stateName(state));
			return;
		}
		this->inspectTarget_ = Eigen::Vector3d (target->point.x, target->point.y, target->point.z);
		if (state == PAUSED){
			this->missionState_ = EXPLORE; // the inspection replaces the paused leg
		}
		this->explorationReplan_ = false;
		this->pendingCommand_ = CMD_INSPECT;
	}

	void dynamicExploration::onPathFinished(bool success){
		// the current leg ended (reached its last waypoint, or was aborted): decide what comes next
		int state = this->missionState_;
		if (state == NUDGE){
			this->missionState_ = EXPLORE;
			this->explorationReplan_ = true;
			this->logEvent(success ? "Entry waypoints done: exploring" : "Entry waypoints aborted: exploring from here");
		}
		else if (state == EXPLORE){
			this->explorationReplan_ = true;
		}
		else if (state == INSPECT){
			if (success){
				this->logEvent("At inspection viewpoint, holding " + std::to_string(int(this->inspectHoldTime_)) + " s");
				this->waitTime(this->inspectHoldTime_);
				this->logEvent("Inspection done: exploring");
			}
			else{
				this->logEvent("Inspection leg aborted: exploring");
			}
			if (this->missionState_ == INSPECT){ // not changed by a command while holding
				this->missionState_ = EXPLORE;
				this->explorationReplan_ = true;
			}
		}
		else if (state == RETURN){
			if (success){
				this->logEvent("Home reached");
				this->startLanding();
			}
			else if (this->returnRetries_ < 3){
				++this->returnRetries_;
				this->logEvent("Return leg aborted, replanning (" + std::to_string(this->returnRetries_) + "/3)");
				this->pendingCommand_ = CMD_RETURN_HOME;
			}
			else{
				this->logEvent("Cannot find a way home: landing here");
				this->startLanding();
			}
		}
		else if (state == LAND){
			if (success){
				this->touchdown();
			}
			else{
				// descend straight down with position control; touchdown detection finishes the landing
				geometry_msgs::PoseStamped ps;
				ps.pose = this->odom_.pose.pose;
				ps.pose.position.z = this->landTarget_(2);
				this->updateTarget(ps);
				this->descentStarted_ = true;
				this->touchdownCheckTime_ = ros::Time::now();
			}
		}
	}

	void dynamicExploration::setPendingPath(const nav_msgs::Path& path, int state){
		if (path.poses.size() < 2) return;
		std::lock_guard<std::mutex> lock (this->pendingPathMutex_);
		this->pendingPath_ = path;
		this->pendingPathState_ = state;
		this->pendingPathReady_ = true;
	}

	void dynamicExploration::applyPendingPath(){
		nav_msgs::Path path;
		int state;
		{
			std::lock_guard<std::mutex> lock (this->pendingPathMutex_);
			if (not this->pendingPathReady_) return;
			path = this->pendingPath_;
			state = this->pendingPathState_;
			this->pendingPathReady_ = false;
		}
		int currState = this->missionState_;
		if (currState == PAUSED or currState == LAND or currState == LANDED) return; // a command overrode this path
		if (state == EXPLORE and currState != EXPLORE) return; // stale exploration path
		this->flyPath(path, state);
	}

	void dynamicExploration::flyPath(const nav_msgs::Path& path, int state){
		// main thread only
		if (path.poses.size() < 2) return;
		this->waypoints_ = path;
		this->waypointIdx_ = 1;
		this->trajectoryReady_ = false;
		this->replan_ = false;
		this->missionState_ = state;
		if (state == LAND){
			this->descentStarted_ = false;
			this->touchdownCheckTime_ = ros::Time::now();
		}
		this->newWaypoints_ = true;
	}

	void dynamicExploration::startLanding(){
		// main thread only: descend to just above the mapped ground under the robot
		Eigen::Vector3d p = this->currPos();
		double res = this->map_->getRes();
		double groundTop = std::numeric_limits<double>::quiet_NaN();
		if ((Eigen::Vector2d (p(0), p(1)) - this->homePos_.head<2>()).norm() < 1.0){
			groundTop = this->homeGroundZ_; // the takeoff spot
		}
		else{
			for (double z=p(2); this->map_->isInMap(Eigen::Vector3d (p(0), p(1), z)); z-=res){
				if (this->map_->isOccupied(Eigen::Vector3d (p(0), p(1), z))){
					groundTop = z + res/2;
					break;
				}
			}
		}
		if (std::isnan(groundTop)){
			// ground under the robot is not mapped: descend until touchdown is detected
			groundTop = p(2) - 50.0;
			while (not this->map_->isInMap(Eigen::Vector3d (p(0), p(1), groundTop))) groundTop += res;
			this->logEvent("Landing: ground below not mapped, descending until touchdown");
		}
		this->landTarget_ = Eigen::Vector3d (p(0), p(1), std::min(groundTop + this->landClearance_, p(2)));
		this->pausedRemainingPath_.poses.clear();
		this->explorationReplan_ = false;
		this->touchdownCheckTime_ = ros::Time::now();
		if (p(2) - this->landTarget_(2) < 0.1){
			this->missionState_ = LAND;
			this->touchdown();
			return;
		}
		this->flyPath(this->makePath({p, this->landTarget_}, this->currYaw()), LAND);
		this->logEvent("Landing");
	}

	void dynamicExploration::touchdown(){
		this->trajectoryReady_ = false;
		this->replan_ = false;
		this->newWaypoints_ = false;
		this->waypoints_.poses.clear();
		this->targetPubEnabled_ = false;
		this->landPub_.publish(std_msgs::Empty());
		this->missionState_ = LANDED;
		this->logEvent("Landed");
	}

	bool dynamicExploration::findInspectViewpoint(const Eigen::Vector3d& target, Eigen::Vector3d& viewpoint){
		// a known-free spot about inspect_distance from the target, preferring the side the robot is on,
		// with a clear line of sight to the target
		Eigen::Vector3d p = this->currPos();
		double baseYaw = atan2(p(1) - target(1), p(0) - target(0));
		std::vector<double> yawOffsets {0.0, 0.5, -0.5, 1.0, -1.0, 1.6, -1.6, 2.2, -2.2, PI_const};
		std::vector<double> distances {this->inspectDistance_, this->inspectDistance_ * 0.75, this->inspectDistance_ * 1.3};
		std::vector<double> heights {0.0, 1.0, -1.0};
		for (double dh : heights){
			for (double d : distances){
				for (double dyaw : yawOffsets){
					double yaw = baseYaw + dyaw;
					Eigen::Vector3d candidate = target + Eigen::Vector3d (d * cos(yaw), d * sin(yaw), dh);
					if (not this->expPlanner_->isPosValid(candidate)) continue;
					Eigen::Vector3d toTarget = target - candidate;
					Eigen::Vector3d nearTarget = target - toTarget.normalized() * 0.5; // the target itself is a surface
					if (not this->map_->isInflatedFreeLine(candidate, nearTarget)) continue;
					viewpoint = candidate;
					return true;
				}
			}
		}
		return false;
	}

	nav_msgs::Path dynamicExploration::makePath(const std::vector<Eigen::Vector3d>& points, double finalYaw){
		nav_msgs::Path path;
		for (size_t i=0; i<points.size(); ++i){
			geometry_msgs::PoseStamped ps;
			ps.header.frame_id = "map";
			ps.pose.position.x = points[i](0);
			ps.pose.position.y = points[i](1);
			ps.pose.position.z = points[i](2);
			ps.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, finalYaw);
			path.poses.push_back(ps);
		}
		return path;
	}

	void dynamicExploration::statusCB(const ros::TimerEvent&){
		// % explored: known voxels in the progress region (main thread: owns the map)
		Eigen::Vector3d size = this->progressRegionMax_ - this->progressRegionMin_;
		if (size.minCoeff() > 0){
			double res = this->map_->getRes();
			long known = 0, total = 0;
			for (double x=this->progressRegionMin_(0)+res/2; x<this->progressRegionMax_(0); x+=res){
				for (double y=this->progressRegionMin_(1)+res/2; y<this->progressRegionMax_(1); y+=res){
					for (double z=this->progressRegionMin_(2)+res/2; z<this->progressRegionMax_(2); z+=res){
						Eigen::Vector3d p (x, y, z);
						if (not this->map_->isInMap(p)) continue;
						++total;
						if (not this->map_->isUnknown(p)) ++known;
					}
				}
			}
			this->exploredPercent_ = total == 0 ? 0.0 : 100.0 * double(known) / double(total);
			this->exploredVolume_ = double(known) * pow(res, 3);
		}

		Eigen::Vector3d p = this->currPos();
		double elapsed = this->missionStartTime_.isZero() ? 0.0 : (ros::Time::now() - this->missionStartTime_).toSec();
		std::ostringstream json;
		json << std::fixed << std::setprecision(2);
		json << "{\"state\":\"" << stateName(this->missionState_) << "\"";
		json << ",\"paused_state\":\"" << (this->missionState_ == PAUSED ? stateName(this->stateBeforePause_) : "") << "\"";
		json << ",\"elapsed\":" << elapsed;
		json << ",\"explored_pct\":" << this->exploredPercent_;
		json << ",\"explored_m3\":" << this->exploredVolume_;
		json << ",\"frontiers\":" << this->frontierNum_;
		json << ",\"map_res\":" << this->map_->getRes();
		json << ",\"position\":[" << p(0) << "," << p(1) << "," << p(2) << "]";
		json << ",\"yaw\":" << this->currYaw();
		json << ",\"home\":[" << this->homePos_(0) << "," << this->homePos_(1) << "," << this->homePos_(2) << "]";
		json << ",\"region_min\":[" << this->progressRegionMin_(0) << "," << this->progressRegionMin_(1) << "," << this->progressRegionMin_(2) << "]";
		json << ",\"region_max\":[" << this->progressRegionMax_(0) << "," << this->progressRegionMax_(1) << "," << this->progressRegionMax_(2) << "]";
		json << ",\"events\":[";
		{
			std::lock_guard<std::mutex> lock (this->eventsMutex_);
			for (size_t i=0; i<this->events_.size(); ++i){
				json << (i == 0 ? "" : ",") << "{\"t\":" << this->events_[i].first << ",\"text\":\"" << this->events_[i].second << "\"}";
			}
		}
		json << "]}";
		std_msgs::String msg;
		msg.data = json.str();
		this->statusPub_.publish(msg);
	}

	void dynamicExploration::logEvent(const std::string& text){
		cout << "\033[1;36m[Mission]: " << text << "\033[0m" << endl;
		double t = this->missionStartTime_.isZero() ? 0.0 : (ros::Time::now() - this->missionStartTime_).toSec();
		std::lock_guard<std::mutex> lock (this->eventsMutex_);
		this->events_.push_back(std::make_pair(t, text));
		while (this->events_.size() > 30) this->events_.pop_front();
	}

	std::string dynamicExploration::stateName(int state){
		switch (state){
			case IDLE: return "IDLE";
			case TAKEOFF: return "TAKEOFF";
			case NUDGE: return "ENTRY";
			case EXPLORE: return "EXPLORING";
			case PAUSED: return "PAUSED";
			case INSPECT: return "INSPECTING";
			case RETURN: return "RETURNING";
			case LAND: return "LANDING";
			case LANDED: return "LANDED";
		}
		return "UNKNOWN";
	}

	Eigen::Vector3d dynamicExploration::currPos(){
		return Eigen::Vector3d (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
	}

	double dynamicExploration::currYaw(){
		return AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
	}

	double dynamicExploration::computeExecutionDistance(){
		if (this->trajectoryReady_ and not this->replan_){
			Eigen::Vector3d prevP, currP;
			bool firstTime = true;
			double totalDistance = 0.0;
			double remainDistance = 0.0;
			for (double t=0.0; t<=this->trajectory_.getDuration(); t+=0.1){
				currP = this->trajectory_.at(t);
				if (firstTime){
					firstTime = false;
				}
				else{
					if (t <= this->trajTime_){
						totalDistance += (currP - prevP).norm();
					}
					else{
						remainDistance += (currP - prevP).norm();
					}
				}
				prevP = currP;
			}
			if (remainDistance <= 1.0){ // no replan when less than 1m
				return -1.0;
			}
			return totalDistance;
		}
		return -1.0;
	}

	bool dynamicExploration::replanForDynamicObstacle(){
		ros::Time currTime = ros::Time::now();
		std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
		if (this->useFakeDetector_){
			this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}
		else{ 
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}

		bool replan = false;
		bool hasDynamicObstacle = (obstaclesPos.size() != 0);
		if (hasDynamicObstacle){
			double timePassed = (currTime - this->lastDynamicObstacleTime_).toSec();
			if (timePassed >= this->replanTimeForDynamicObstacle_){
				replan = true;
				this->lastDynamicObstacleTime_ = currTime;
			}
		}
		return replan;
	}

	bool dynamicExploration::reachExplorationGoal(){
		if (this->waypoints_.poses.size() == 0) return false;
		double distThresh = 0.1;
		double yawDiffThresh = 0.1;
		Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d targetPos (this->waypoints_.poses.back().pose.position.x, this->waypoints_.poses.back().pose.position.y, this->waypoints_.poses.back().pose.position.z);
		double yawTarget = AutoFlight::rpy_from_quaternion(this->waypoints_.poses.back().pose.orientation); 
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		if ((currPos - targetPos).norm() <= distThresh and std::abs(currYaw - yawTarget) >= yawDiffThresh){
			return true;
		}
		return false;
	}

	bool dynamicExploration::isGoalValid(){
		Eigen::Vector3d pGoal (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
		if (this->map_->isInflatedOccupied(pGoal)){
			return false;
		}
		else{
			return true;
		}
	}

	nav_msgs::Path dynamicExploration::getCurrentTraj(double dt){
		nav_msgs::Path currentTraj;
		currentTraj.header.frame_id = "map";
		currentTraj.header.stamp = ros::Time::now();
	
		if (this->trajectoryReady_){
			// include the current pose
			// geometry_msgs::PoseStamped psCurr;
			// psCurr.pose = this->odom_.pose.pose;
			// currentTraj.poses.push_back(psCurr);
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=dt){
				Eigen::Vector3d pos = this->trajectory_.at(t);
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = pos(0);
				ps.pose.position.y = pos(1);
				ps.pose.position.z = pos(2);
				currentTraj.poses.push_back(ps);
			}		
		}
		return currentTraj;
	}

	nav_msgs::Path dynamicExploration::getRestGlobalPath(){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, pDiff) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	nav_msgs::Path dynamicExploration::getRestGlobalPath(const Eigen::Vector3d& pos){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr = pos;
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, pDiff) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	nav_msgs::Path dynamicExploration::getRestGlobalPath(const Eigen::Vector3d& pos, double yaw){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr = pos;
		Eigen::Vector3d direction (cos(yaw), sin(yaw), 0);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, direction) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	void dynamicExploration::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize){
		std::vector<onboardDetector::box3D> obstacles;
		this->detector_->getObstaclesInSensorRange(PI_const, obstacles);
		for (onboardDetector::box3D ob : obstacles){
			Eigen::Vector3d pos (ob.x, ob.y, ob.z);
			Eigen::Vector3d vel (ob.Vx, ob.Vy, 0.0);
			Eigen::Vector3d size (ob.x_width, ob.y_width, ob.z_width);
			obstaclesPos.push_back(pos);
			obstaclesVel.push_back(vel);
			obstaclesSize.push_back(size);
		}
	}


	void dynamicExploration::waitTime(double time){
		ros::Rate r (30);
		ros::Time startTime = ros::Time::now();
		ros::Time currTime = ros::Time::now();
		double passtime = 0.0;
		while (ros::ok() and passtime < time){
			currTime = ros::Time::now();
			passtime = (currTime - startTime).toSec();
			ros::spinOnce();
			r.sleep();
		}
	}
}