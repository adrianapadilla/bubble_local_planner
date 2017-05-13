
#include "bubble_local_planner/bubble_local_planner.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(bubble_local_planner, BubblePlannerROS, bubble_local_planner::BubblePlannerROS, nav_core::BaseLocalPlanner)

namespace bubble_local_planner{

	BubblePlannerROS::BubblePlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

	BubblePlannerROS::BubblePlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
         : costmap_ros_(NULL), tf_(NULL), initialized_(false)
         {
		// initialize planner
		initialize(name, tf, costmap_ros);
         }

	BubblePlannerROS::~BubblePlannerROS() {}

	void BubblePlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
	{

		// check if the plugin is already initialized
		if(!initialized_)
		{
		// copy adress of costmap and Transform Listener (handed over from move_base)
		costmap_ros_ = costmap_ros;
		tf_ = tf;

		//initializing flags
		flag = 1;
		flag2 = 1;
		flag3 = 1;

		//initializing speeds

		cmd.linear.x= 0.0;
		cmd.linear.y= 0.0;
		cmd.angular.z= 0.0;

		setBubble();

		// subscribe to topics
		ros::NodeHandle n;
		amcl_sub = n.subscribe("amcl_pose", 100, &BubblePlannerROS::amclCallback, this);
		laser_sub = n.subscribe("laser", 100, &BubblePlannerROS::laserCallback, this);

		//publish to topics
		//bubble_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

		// set initialized flag
		initialized_ = true;

		// this is only here to make this process visible in the rxlogger right from the start
		ROS_DEBUG("Bubble Local Planner plugin initialized.");
		}
		else
		{
		ROS_WARN("This planner has already been initialized, doing nothing.");
		}

	}


	bool BubblePlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
	{

		// check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}


		//reset next counter
		count = 1;

		//set plan, length and next goal
		plan = orig_global_plan; 
		length = (plan).size();  
		setNext(); 

		// set goal as not reached
		goal_reached_ = false;

		return true;

	}

	bool BubblePlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{

		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

		if(length){
			
			if(!obstacle() & flag){

				ROS_INFO("no obstaclse: normal mode");

				setNowError(); 
				
				if(distance < 0.3){ 

					if(count<(length-1)){

						if((length - 1 - count) < 11){
							count = length - 1;
						}else{
							count += 10;
						}
						
						setNext();
						
					}else{
	
						
						setVelZ(); 
						goal_reached_ = true;

					}


				}else{

					
					if(fabs(nError.az) > 25*D2R){
					
						setRot();

					}else{
					
						setVel();

					}

				}
				
			}else if(!obstacle() & flag3){


				ROS_INFO("no obstacles: avoidance mode");

				if(goalVisible()){  

					ROS_INFO("intermediate goal = %d", intermGoal);
					ROS_INFO("angle= %d", intermGoalA);
					ROS_INFO("distance = %f", intermGoalD);

					ROS_INFO("yawR fase2 = %f", yawR);

					ROS_INFO("back on the global path!");

					count = intermGoal;
					setNext();
				
					flag = 1; 
				
					 
				}else{

					computeReboundAngle2();
					setNowErrorR();

					if(fabs(nError.az) > 5*D2R){ 

						setRotR();
						ROS_INFO("angular: fase 2");

					}else{
					
						setVelR(); 
						ROS_INFO("linear: fase 2");
			
					}

				}


			}else{

				ROS_INFO("obstacle detected: rebound angle");

				flag = 0;
				flag3 = 0;

				if(flag2){
					computeReboundAngle(); 
					flag2 = 0;
				}

				ROS_INFO("the new angle is: %.4f", yawR);
				setNowErrorR();

				if(fabs(nError.az) > 5*D2R){ 

					ROS_INFO("angular: fase 1");

					setRotRR();
					
				}else{
					flag2 = 1;
					flag3 = 1;
				}
		
			}

			ROS_INFO("vel->linear.x = %.4f", cmd.linear.x);
			ROS_INFO("vel->angular.z=%.4f", cmd.angular.z);

			ROS_INFO("count = %d", count);
			ROS_INFO("length of path: %d", length);


		}


		setBubble();
		cmd_vel = cmd;  

		return true;

	}


	bool BubblePlannerROS::isGoalReached()
	{
		// check if plugin initialized
		if(!initialized_)
		{
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}

		// this info comes from compute velocity commands:
		return goal_reached_;

	}


	void BubblePlannerROS::laserCallback(const sensor_msgs::LaserScan::Ptr& msg)
	{
		ROS_INFO("Seq. laser: [%d]", msg->header.seq);

		laserData = *msg;

		for(int i = 0; i<180; i++){

			if(laserData.ranges[i]>3) laserData.ranges[i] = 3;

		}
 
	}

	void BubblePlannerROS::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg)
	{

		ROS_INFO("Seq: [%d]", msg->header.seq);

		poseNow.position = msg->pose.pose.position;
		poseNow.orientation = msg->pose.pose.orientation;
		now.x = msg->pose.pose.position.x;
		now.y = msg->pose.pose.position.y;
		now.az = getYaw(*msg); 
		setNowError();
		setNowErrorR();

	}



	void BubblePlannerROS::setBubble() 
	{

		double speed = std::sqrt(cmd.linear.x*cmd.linear.x ); //translation speed

		for(int i = 0; i<80; i++){
			bubble[i] =(i/90)*speed; 
		}
		for(int i = 80; i<90; i++){
			bubble[i] =(i/90)*4.5*speed; 
		}
		for(int i = 90; i<100; i++){
			bubble[i] =((180-i)/90)*4.5*speed;
		}
		for(int i = 100; i<180; i++){
			bubble[i] =((180-i)/90)*speed; 
		}


		ROS_INFO("bubble radius is %.4f", bubble[0]);

				
		//bubbleVisualization();	

	}

	
	bool BubblePlannerROS::obstacle()
	{
		for(int i = 0; i<180; i++){
			if(laserData.ranges[i]<=bubble[i]) return true;			
		}

		return false;
	}


	void BubblePlannerROS::computeReboundAngle()
	{		
		double num = 0;
		double den = 0;
		double zone1 = 0, zone2 = 0, zone3 = 0;
		int n, N;

		//check which side has less obstacles:

		//first I add up all the laser readings from each side
		for(int j = 0; j<90; j++){ 

			if(laserData.ranges[j]<3) {zone1 += laserData.ranges[j];}else{zone1 +=3;}

			if(laserData.ranges[j+90]<3) {zone3 += laserData.ranges[j+90];}else{zone3 += 3;}
		}

		ROS_INFO("zone1 = %f",zone1);
		ROS_INFO("zone3 = %f",zone3);

		//I choose the zone with higher overall laser readings (which means less obstacles):
		if(zone1>zone3){ 
			n=0; N=60; d = 2; 
		}else{ 
		        n=120; N=180; d = 1; 
		}

		//calculate the weighted arithmetic mean of the zone with less obstacles:
		for(int i = n; i<N; i++){

			// i is the angle (the data) and laserData.ranges[i] is the distance the laser reads (the weight)	
			num += (i*D2R)*laserData.ranges[i]; 
			den += laserData.ranges[i];

		}

		int k = 0;
		if(num/den <= 90) k=-1;
		else k = 1;

		//transforming the angle from the robot's frame to the global frame
		yawR = (num/den)-90*D2R + k*10*D2R + now.az; 
		ROS_INFO("division is: %.4f", (num/den));


	}

	void BubblePlannerROS::computeReboundAngle2()
	{

		double thetaObsR, distObsR, thetaObsL, distObsL;
		double theta, dist, alpha;
		int angle;
		double n, p, q;
		int min, max, m;
		bool thisWay = 1;
		double yawR_;


		theta = 90;
		dist= laserData.ranges[90];

		if(d == 2){ 
			for(int i = 90; i<179 ; i++){
				if((laserData.ranges[i-1]-laserData.ranges[i])>1){ 
					
						theta = i; 
						dist = laserData.ranges[i];
						m = -1;
						break; 

				}
			}  
		}

		if(d == 1){
			for(int i = 90; i>=0 ; i--){
				if((laserData.ranges[i+1]-laserData.ranges[i])>1){ 

						theta = i; 
						dist = laserData.ranges[i]; 
						m = 1; 
						break;

				}
			}  
		}


		if(dist<=0.8) {

			thetaObsL = 0;
			distObsL = laserData.ranges[0];

			thetaObsR = theta;
			distObsR = laserData.ranges[90]; 

		
			for(int i = 0; i < theta; i++){
				if(laserData.ranges[i]<distObsL){		
					thetaObsL = i;
					distObsL = laserData.ranges[i];
				}
			}

			for(int i = theta; i < 180; i++){
				if(laserData.ranges[i]<distObsR){		
					thetaObsR = i;
					distObsR = laserData.ranges[i];
				}
			}

			//we will be calculating a weighted arithmetic mean

			n = distObsR/(distObsR + 0.5); //the closer to the obstacle, the more the obstacle's angle weighs

			p = distObsL/(distObsL + 0.5); //the closer to the obstacle, the more the obstacle's angle weighs

			q = distObsL/(distObsL + distObsR); //the closer to obstacle L, the more this obstacle's angle weighs
		
			angle = q*(theta*n + (-180 + thetaObsR)*(1-n)) + (1-q)*(theta*p + (180 + thetaObsL)*(1-p));

		
			ROS_INFO("close to obtacle");
			

		}else{

			//calculating an angle that will create a distance bigger than 0.8m between the robot and nearby obstacle

			alpha = (1/D2R)*std::asin(0.8/dist);
			angle = theta + m*alpha;

			ROS_INFO("far from obtacle");
		
		}

		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( angle > 180 )  angle -= 360;
		if ( angle < -180 )  angle += 360;

		yawR_ = (angle - 90  + now.az/D2R)*D2R;
		
		//trust me, there is a good reason for this:
		if(fabs(yawR-yawR_)>0.18){

			yawR = yawR_;

		}

	}

	void BubblePlannerROS::setNext()
	{

		next.x = plan[count].pose.position.x;
		next.y = plan[count].pose.position.y;

	}


	double BubblePlannerROS::getYaw(geometry_msgs::PoseWithCovarianceStamped msg)
	{

		double q[4];
		q[0]= msg.pose.pose.orientation.x;
		q[1]= msg.pose.pose.orientation.y;
		q[2]= msg.pose.pose.orientation.z;
		q[3]= msg.pose.pose.orientation.w;

		double t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
		double t4 = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);  

		return std::atan2(t3, t4);

	}

//////////////////////// functions for setting errors, and angular and linear speeds:


	void BubblePlannerROS::setNowError()
	{

		double d;

		nError.x = (next.x - now.x);
		nError.y = (next.y - now.y);

		// if these two variables are null, the tangent doesn't exist 
		// plus, the angle error is irrelevant because we have arrived at our destination

		if (nError.y == 0 & nError.x == 0){  
			d = now.az;
		}else{	
			d = std::atan2(nError.y, nError.x);
		}

		distance = std::sqrt(nError.x*nError.x +nError.y*nError.y);
		nError.az = d - now.az;

		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( nError.az > 180*D2R ) { nError.az -= 360*D2R; }
		if ( nError.az < -180*D2R ) { nError.az += 360*D2R;  }

	}


	void BubblePlannerROS::setVel()
	{

		// the output speed has been adjusted with a P regulator, that depends on how close we are to our current goal

		cmd.linear.x= 0.75*distance;

		// keeping a small angular speed so that the movement is smooth
		cmd.angular.z= 0.75*(nError.az);

	}

	void BubblePlannerROS::setRot()
	{



		// the angular speed has been adjusted with a P regulator, that depends on how close we are to pointing at our current goal
		if (fabs(nError.az) > 50*D2R){


			cmd.angular.z=(nError.az)*0.3;//0.2

			// linear speed is zero while the angle is too big
			cmd.linear.x= 0.0;

		}else{

			cmd.angular.z=(nError.az)*0.5;

			// keeping a small linear speed so that the movement is smooth
			cmd.linear.x= 0.1;
		}

	}

	void BubblePlannerROS::setVelZ()
	{

		cmd.linear.x= 0;
		cmd.angular.z=0;

	}


	void BubblePlannerROS::setVelR()
	{

		cmd.linear.x= 0.2;

		// keeping a small angular speed so that the movement is smooth
		cmd.angular.z= 0.75*(nError.az);


	}

	void BubblePlannerROS::setRotRR()
	{

		// the angular speed has been adjusted with a P regulator, that depends on how close we are to pointing at our current goal
		if (fabs(nError.az) > 50*D2R){

			cmd.angular.z=(nError.az)*0.3;

		}else{

			cmd.angular.z=(nError.az)*0.5;
		}

		cmd.linear.x= 0.0;

	}

	void BubblePlannerROS::setRotR()
	{

		// the angular speed has been adjusted with a P regulator, that depends on how close we are to pointing at our current goal
		if (fabs(nError.az) > 50*D2R){

			cmd.angular.z=(nError.az)*0.3;

			cmd.linear.x = 0;

		}else{

			cmd.angular.z=(nError.az)*0.5;

			cmd.linear.x= 0.1;
		}


	}

	void BubblePlannerROS::setNowErrorR()
	{

		distance = intermGoalD;
		nError.az = yawR - now.az;

		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( nError.az > 180*D2R ) { nError.az -= 360*D2R; }
		if ( nError.az < -180*D2R ) { nError.az += 360*D2R;  }

	}




///////////////// functions related to finding an intermediate goal (belonging to the global path):
 
	bool BubblePlannerROS::goalVisible()
	{		
		int intermGoal_ = findIntermGoal();
		
		if(intermGoal_ == 0){ 

			return false; 

		}else{ 

			intermGoal = intermGoal_;
			setIntermGoal();

		}

		return true;
	}


	void BubblePlannerROS::setIntermGoal()
	{

		pos iG;
		double d;

		iG.x = plan[intermGoal].pose.position.x - now.x;
		iG.y = plan[intermGoal].pose.position.y - now.y;

		ROS_INFO("distance x = %f", iG.x);
		ROS_INFO("distance y = %f", iG.y);

		if (iG.y == 0 & iG.x == 0){  
			d = now.az;
		}else{	
			d = std::atan2(iG.y, iG.x); 
		}

		intermGoalD = std::sqrt(iG.x*iG.x + iG.y*iG.y);
		intermGoalA = d/D2R;

		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( intermGoalA > 180 )  intermGoalA -= 360;
		if ( intermGoalA < -180 )  intermGoalA += 360;

	}



	int BubblePlannerROS::findIntermGoal()
	{
		int nextPoint = 0;

		for(int m = length-1; m > count+100; m--){ 
				
				int allClear = 1;
				double alpha;	
				double dist;	
				pos fError;
				int min, max;

				fError.x = plan[m].pose.position.x - now.x;
				fError.y = plan[m].pose.position.y - now.y;

				dist = std::sqrt(fError.x*fError.x +fError.y*fError.y);		

				alpha = std::atan2(fError.y,fError.x); //this is the angle to the goal (straight line)

				alpha = alpha +90*D2R -now.az; // passing to robot's frame

				alpha = alpha/D2R; //passing radians to degrees

				if(dist>1){

					allClear = 0;
				}else{

					if(alpha < 154 | alpha > 25){

						min = alpha - 25;
						max = alpha + 25;

						for(int i = min; i < max; i++)
						{

							if(laserData.ranges[i] < 1) allClear = 0; 

						}

						if (allClear) nextPoint = m;

					}

				}
				
			
		}


		return nextPoint;
	}


	

}
