/*
	FILE: dynamicExploration.h
	-----------------------------
	header of dynamic exploration
*/
#ifndef DYNAMIC_EXPLORATION
#define DYNAMIC_EXPLORATION
#include <autonomous_flight/simulation/flightBase.h>
#include <global_planner/dep.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <map_manager/dynamicMap.h>
#include <onboard_detector/fakeDetector.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <atomic>
#include <mutex>
#include <deque>


namespace AutoFlight{
	class dynamicExploration : flightBase{
	private:
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<onboardDetector::fakeDetector> detector_;
		std::shared_ptr<globalPlanner::DEP> expPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		ros::Timer explorationTimer_;
		ros::Timer plannerTimer_;
		ros::Timer replanCheckTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer visTimer_;
		ros::Timer freeMapTimer_;
		ros::Timer freeRobotRegionTimer_;

		ros::Publisher polyTrajPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;
		ros::Publisher inputTrajPub_;
		
		// parameters
		bool useFakeDetector_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		double wpStablizeTime_;
		bool initialScan_;
		double replanTimeForDynamicObstacle_;
		Eigen::Vector3d freeRange_;
		double reachGoalDistance_;
		std::vector<Eigen::Vector4d> initialWaypoints_; // x, y, z, yaw (rad) flown before exploration starts
		int maxExplorationFailures_; // consecutive failed DEP plans before exploration counts as complete
		double inspectHoldTime_; // s, hover time at an inspection viewpoint
		double inspectDistance_; // m, viewpoint distance from the inspected point
		double landClearance_; // m, descend to this height above the mapped ground, then cut motors
		std::string landTopic_;
		Eigen::Vector3d progressRegionMin_, progressRegionMax_; // region used for the % explored figure

		// mission (GUI / services): every leg is a waypoint path flown by the trajectory planner
		enum MissionState {IDLE, TAKEOFF, NUDGE, EXPLORE, PAUSED, INSPECT, RETURN, LAND, LANDED};
		enum MissionCommand {CMD_NONE, CMD_RETURN_HOME, CMD_INSPECT};
		std::atomic<int> missionState_ {IDLE};
		int stateBeforePause_ = IDLE;
		nav_msgs::Path pausedRemainingPath_;
		std::atomic<int> pendingCommand_ {CMD_NONE}; // handled by the planning thread (it owns DEP)
		std::atomic<bool> startRequested_ {false};
		std::mutex pendingPathMutex_; // planning thread -> main thread handoff, never held while spinning
		nav_msgs::Path pendingPath_;
		int pendingPathState_ = IDLE;
		bool pendingPathReady_ = false;
		Eigen::Vector3d homePos_; // hover point above the takeoff spot
		double homeGroundZ_ = 0.0; // height of the takeoff spot
		Eigen::Vector3d inspectTarget_;
		int returnRetries_ = 0;
		Eigen::Vector3d landTarget_;
		ros::Time touchdownCheckTime_;
		bool descentStarted_ = false; // touchdown only counts once the robot has actually been descending
		ros::Time missionStartTime_;
		std::mutex eventsMutex_;
		std::deque<std::pair<double, std::string>> events_; // (mission time, text) for the GUI
		double exploredPercent_ = 0.0;
		double exploredVolume_ = 0.0;
		std::atomic<int> frontierNum_ {-1};

		ros::ServiceServer startSrv_, pauseSrv_, resumeSrv_, returnHomeSrv_, landSrv_;
		ros::Subscriber inspectSub_;
		ros::Publisher statusPub_, landPub_;
		ros::Timer statusTimer_;

		// exploration data
		std::atomic<bool> explorationReplan_ {false};
		bool replan_ = false;
		bool newWaypoints_ = false;
		int waypointIdx_ = 1;
		nav_msgs::Path waypoints_; // latest waypoints from exploration planner
		nav_msgs::Path inputTrajMsg_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;
		bool trajectoryReady_ = false;
		ros::Time trajStartTime_;
		double trajTime_; // current trajectory time
		trajPlanner::bspline trajectory_;
		ros::Time lastDynamicObstacleTime_;
	
	public:
		std::thread exploreReplanWorker_;
		dynamicExploration();
		dynamicExploration(const ros::NodeHandle& nh);

		void initParam();
		void initModules();
		void registerCallback();
		void registerPub();

		void explorationCB(const ros::TimerEvent&);
		void plannerCB(const ros::TimerEvent&);
		void replanCheckCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void freeMapCB(const ros::TimerEvent&); // using fake detector
		void freeRobotRegionCB(const ros::TimerEvent&); // the robot's own volume is free space

		// mission interface
		void registerMissionInterface();
		bool startSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool pauseSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool resumeSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool returnHomeSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool landSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		void inspectCB(const geometry_msgs::PointStampedConstPtr& target);
		void statusCB(const ros::TimerEvent&);
		void onPathFinished(bool success);
		void applyPendingPath();
		void setPendingPath(const nav_msgs::Path& path, int state);
		void flyPath(const nav_msgs::Path& path, int state);
		void startLanding();
		void touchdown();
		bool findInspectViewpoint(const Eigen::Vector3d& target, Eigen::Vector3d& viewpoint);
		nav_msgs::Path makePath(const std::vector<Eigen::Vector3d>& points, double finalYaw);
		void logEvent(const std::string& text);
		static std::string stateName(int state);
		Eigen::Vector3d currPos();
		double currYaw();

		void run();
		void initExplore();
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions);
		bool hasCollision();
		bool hasDynamicCollision();
		void exploreReplan();
		double computeExecutionDistance();
		bool replanForDynamicObstacle();
		bool reachExplorationGoal();
		bool isGoalValid();
		nav_msgs::Path getCurrentTraj(double dt);
		nav_msgs::Path getRestGlobalPath();
		nav_msgs::Path getRestGlobalPath(const Eigen::Vector3d& pos);
		nav_msgs::Path getRestGlobalPath(const Eigen::Vector3d& pos, double yaw);
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize);
		void waitTime(double time);
	};
}
#endif