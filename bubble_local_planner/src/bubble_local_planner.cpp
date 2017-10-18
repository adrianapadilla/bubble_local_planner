
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
     		flag4 = 1;

		//initializing speeds

		cmd.linear.x= 0.0;
		cmd.linear.y= 0.0;
		cmd.angular.z= 0.0;

		setBubble();

		// subscribe to topics
		ros::NodeHandle n;
		amcl_sub = n.subscribe("amcl_pose", 100, &BubblePlannerROS::amclCallback, this);
		laser_sub = n.subscribe("laser", 100, &BubblePlannerROS::laserCallback, this);
        	path_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

		//initializing the visualization markers
		points.header.frame_id = "/map";
	 
		points.header.stamp = ros::Time::now();
	 
		points.ns = "path_drawing";
	 
		points.action = visualization_msgs::Marker::ADD;
	 
		points.id = 0;
	 
		//points.pose.position.x = now.x;
		//points.pose.position.y = now.y;
		//points.pose.position.z = 0.5;
	 
		points.type = visualization_msgs::Marker::POINTS;
	 
		points.scale.x = 0.1;
		points.scale.y = 0.1;
	 
		points.color.g = 1.0f;
		points.color.a = 1.0;

		average = 0;
		num = 0;
		firstTime = 1;
		hasStarted = 0;
                pathLength = 0;

		//open file
		file.open("/home/adriana/adrianaTFG/bubble_cs/bubble_cs_10.txt", ios::out);
		//file.open("/home/adriana/adrianaTFG/my_robot_name_2dnav/bubble_bc_1.txt", ios::out); //| ios::trunc);
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

			if(firstTime){
				startTime = ros::Time::now().toSec();
				firstTime = 0;
			}

			double beginning = ros::Time::now().toSec();
			
			if(!obstacle() & flag){

				ROS_INFO("no obstacles: normal mode");

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
						stopTime = ros::Time::now().toSec();
						
						//time to reach goal
						ROS_INFO("journey duration: %f", (stopTime-startTime));

						//path length
						ROS_INFO("path length: %f", pathLength);

						//average executation time (for computational cost)
						ROS_INFO("avrg exec time: %f", average/num);

						if(file.is_open()){
							ROS_INFO("I'm OPEN!");
							//file.close();
						}else{
							ROS_INFO("I'm NOT open!");
						}
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



				
				if(goalVisible()){ 

					ROS_INFO("goal is visible again!");
							
					count = intermGoal;
					setNext();

					ROS_INFO("intermgoal is: %d", intermGoal);
					
					flag = 1; //once the goal is visible, we keep following the global path
					flag2 = 1; //this flag is meant to only allow computeReboundAngle() to go once

				}else{

					ROS_INFO("no obstacles: avoidance mode");
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


				flag = 0;
				flag3 = 0;

				if(goalVisible()){ 

					ROS_INFO("goal is visible again!");
							
					count = intermGoal;
					setNext();

					ROS_INFO("intermgoal is: %d", intermGoal);
					
					flag = 1; //once the goal is visible, we keep following the global path
					flag2 = 1; //this flag is meant to only allow computeReboundAngle() to go once

				}else{


					ROS_INFO("obstacle detected: rebound angle");
					
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
			}

			//max and min speeds:
			if(cmd.linear.x>0.3)cmd.linear.x = 0.3;
			if(cmd.angular.z>0.3)cmd.angular.z = 0.3;
			if(cmd.linear.x<-0.3)cmd.linear.x = -0.3;
			if(cmd.angular.z<-0.3)cmd.angular.z = -0.3;

			ROS_INFO("vel->linear.x = %.4f", cmd.linear.x);
			ROS_INFO("vel->angular.z=%.4f", cmd.angular.z);

			ROS_INFO("count = %d", count);
			ROS_INFO("length of path: %d", length);

			double ending = ros::Time::now().toSec();
			average += (ending-beginning);
			num++;

		}


		setBubble();
		file << cmd.linear.x << " "<< cmd.linear.y << " "<< cmd.angular.z << endl;
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

			if(laserData.ranges[i]>4) laserData.ranges[i] = 4;

		}
 
	}

	void BubblePlannerROS::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg)
	{

		ROS_INFO("Seq: [%d]", msg->header.seq);

		//poseNow.position = msg->pose.pose.position;
		//poseNow.orientation = msg->pose.pose.orientation;
		
		pos before;
		if(hasStarted){
			before.x = now.x;
			before.y = now.y;
		}

		now.x = msg->pose.pose.position.x;
		now.y = msg->pose.pose.position.y;
		now.az = getYaw(*msg); 
		setNowError();
		setNowErrorR();

		if(hasStarted){
			pathLength += std::sqrt((now.x-before.x)*(now.x-before.x) + (now.y-before.y)*(now.y-before.y));
			ROS_INFO("%f, %f, %f", stopTime, startTime, pathLength);
		}

		//ROS_INFO("%f - %f, %f - %f", now.x, before.x, now.y, before.y);

		pathVisualization();
		
		hasStarted = 1;

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


		ROS_INFO("bubble radius is %.4f", bubble[90]);

				
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
		double ang;

		//check which side has less obstacles:

		//first I add up all the laser readings from each side
		for(int j = 0; j<90; j++){ 

			if(laserData.ranges[j]<4) {zone1 += laserData.ranges[j];}else{zone1 +=4;}

			if(laserData.ranges[j+90]<4) {zone3 += laserData.ranges[j+90];}else{zone3 += 4;}
		}

		ROS_INFO("zone1 = %f",zone1);
		ROS_INFO("zone3 = %f",zone3);

		/*if(zone1>zone3){
		
			d = 2; 

			for(int i = 89; i>=0; i--){
				if((laserData.ranges[i-1]-laserData.ranges[i])>1){
					ang = i;
					break;
				}
			}

		}else{

			d = 1; 

			for(int i = 90; i<180; i++){

				if((laserData.ranges[i+1]-laserData.ranges[i])>1){
					ang = i;
					break;
				}

			}

		}


		yawR = ang*D2R -90*D2R + now.az; 
		ROS_INFO("division is: %.4f", ang);*/

		//I choose the zone with higher overall laser readings (which means less obstacles):
		if(zone1>zone3){ 
			n=0; N=90; d = 2; 
		}else{ 
		        n=90; N=180; d = 1; 
		}

		//calculate the weighted arithmetic mean of the zone with less obstacles:
		for(int i = n; i<N; i++){

			// i is the angle (the data) and laserData.ranges[i] is the distance the laser reads (the weight)	
			num += (i*D2R)*laserData.ranges[i]; 
			den += laserData.ranges[i];

		}

		int k = 0;
		if(num/den <= 90*D2R) k=1;
		else k = -1;

		//transforming the angle from the robot's frame to the global frame
		yawR = (num/den)-90*D2R + now.az; //k*10*D2R + now.az; 
		ROS_INFO("division is: %.4f", (num/den));


	}


	void BubblePlannerROS::computeReboundAngle2()
	{

		double thetaObsR, distObsR, thetaObsL, distObsL;
		double theta, dist, alpha;
		int angle = 90;
		double n, p, q;
		int min, max, m;
		bool thisWay = 1;
		double yawR_;
		bool lala = 0;


		theta = 90;
		dist= laserData.ranges[90];

		if(d == 2){
			m = -1;  
			for(int i = 90; i<180 ; i++){
				if((laserData.ranges[i-1]-laserData.ranges[i])>1){ 
					
					if(laserData.ranges[i]<2){
						theta = i; 
						dist = laserData.ranges[i];
						ROS_INFO("lalal1");
						lala = 1;
						break; 
					}

				}
			}  
		}

		if(d == 1){
			m = 1; 
			for(int i = 90; i>=0 ; i--){
				if((laserData.ranges[i+1]-laserData.ranges[i])>1){ 

					if(laserData.ranges[i]<2){
						theta = i; 
						dist = laserData.ranges[i]; 
						ROS_INFO("lalal2");
						lala = 1;
						break;
					}

				}
			}  
		}

		ROS_INFO("theta = %f", theta);

		/*for(int i = 135; i < 180; i++){
			if(laserData.ranges[i]<0.8){		
				close = 1;
			}
		}

		for(int i = 0; i < 45; i++){
			if(laserData.ranges[i]<0.8){		
				close = 1;
			}
		}*/
		
		if(dist<=0.8) {

			thetaObsL = 0;
			distObsL = laserData.ranges[0];

		
			for(int i = 0; i < 180; i++){
				if(laserData.ranges[i]<distObsL){		
					thetaObsL = i;
					distObsL = laserData.ranges[i];
				}
			}


			//we will be calculating a weighted arithmetic mean

			p = distObsL/(0.8); //the closer to the obstacle, the more the obstacle's angle weighs

			//q = distObsL/(distObsL + distObsR); //the closer to obstacle L, the more this obstacle's angle weighs

			if(thetaObsL<theta) q = 1;
			else q = -1;
		
			angle = theta*p + (q*180 + thetaObsL)*(1-p);

		
			ROS_INFO("close to obtacle: %d", angle);
			

		}else{

			//calculating an angle that will create a distance bigger than 0.8m between the robot and nearby obstacle

			alpha = (1/D2R)*std::asin(0.8/dist);
			angle = theta + m*alpha;

			ROS_INFO("far from obtacle: %d", angle);
		
		}

		if(!lala) angle = theta;
		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( angle > 180 )  angle -= 360;
		if ( angle < -180 )  angle += 360;

		yawR_ = (angle - 90  + now.az/D2R)*D2R;
		
		//trust me, there is a good reason for this:
		if(fabs(yawR-yawR_)>0.18){

			yawR = yawR_;

		}

	}

/*	void BubblePlannerROS::computeReboundAngle2()
	{

		double thetaObsR, distObsR, thetaObsL, distObsL;
		double theta, dist, alpha;
		int angle;
		double n, p, q;
		int min, max, m;
		bool thisWay = 1;
		double yawR_;
		bool close = 0;


		theta = 90;
		dist= laserData.ranges[90];

		if(d == 2){ 
			for(int i = 87; i<180 ; i++){
				if((laserData.ranges[i-1]-laserData.ranges[i])>1){ 
					
						theta = i; 
						dist = laserData.ranges[i];
						ROS_INFO("lalal1");
						m = -1;
						break; 

				}
			}  
		}

		if(d == 1){
			for(int i = 93; i>=0 ; i--){
				if((laserData.ranges[i+1]-laserData.ranges[i])>1){ 

						theta = i; 
						dist = laserData.ranges[i]; 
						ROS_INFO("lalal2");
						m = 1; 
						break;

				}
			}  
		}

		ROS_INFO("thetaDisc = %f", theta);

		/*for(int i = 135; i < 180; i++){
			if(laserData.ranges[i]<0.8){		
				close = 1;
			}
		}

		for(int i = 0; i < 45; i++){
			if(laserData.ranges[i]<0.8){		
				close = 1;
			}
		}/*****

		if(dist<=0.8) {

			thetaObsL = 0;
			distObsL = laserData.ranges[0];

		
			for(int i = 0; i < 180; i++){
				if(laserData.ranges[i]<distObsL){		
					thetaObsL = i;
					distObsL = laserData.ranges[i];
				}
			}


			//we will be calculating a weighted arithmetic mean

			p = distObsL/(0.8); //the closer to the obstacle, the more the obstacle's angle weighs

			//q = distObsL/(distObsL + distObsR); //the closer to obstacle L, the more this obstacle's angle weighs

			if(thetaObsL<theta) q = 1;
			else q = -1;
		
			angle = theta*p + (q*180 + thetaObsL)*(1-p);

		
			ROS_INFO("close to obtacle");
			

		}else{

			//calculating an angle that will create a distance bigger than 0.8m between the robot and nearby obstacle

			alpha = (1/D2R)*std::asin(0.5/dist);
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

	}*/

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

		double ang;

		nError.x = (next.x - now.x);
		nError.y = (next.y - now.y);

		// if these two variables are null, the tangent doesn't exist 
		// plus, the angle error is irrelevant because we have arrived at our destination

		if (nError.y == 0 & nError.x == 0){  
			ang = now.az;
		}else{	
			ang = std::atan2(nError.y, nError.x);
		}

		distance = std::sqrt(nError.x*nError.x +nError.y*nError.y);
		nError.az = ang - now.az;

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
		double ang;

		iG.x = plan[intermGoal].pose.position.x - now.x;
		iG.y = plan[intermGoal].pose.position.y - now.y;

		ROS_INFO("distance x = %f", iG.x);
		ROS_INFO("distance y = %f", iG.y);

		if (iG.y == 0 & iG.x == 0){  
			ang = now.az;
		}else{	
			ang = std::atan2(iG.y, iG.x); 
		}

		intermGoalD = std::sqrt(iG.x*iG.x + iG.y*iG.y);
		intermGoalA = ang/D2R;

		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( intermGoalA > 180 )  intermGoalA -= 360;
		if ( intermGoalA < -180 )  intermGoalA += 360;

	}


	int BubblePlannerROS::findIntermGoal()
	{
		int nextPoint = 0;
		int minimum;
		int min, max;

		if(count+100>=(length-1)) minimum = length-2;
		else minimum = count + 90; 
		
		for(int k = length-1; k > minimum; k--){  
		//for(int k = length-1; k > count; k--){  
				
				int allClear = 1;
				double alpha;	
				double dist;	
				pos iError;

				iError.x = plan[k].pose.position.x - now.x;
				iError.y = plan[k].pose.position.y - now.y;

				dist = std::sqrt(iError.x*iError.x +iError.y*iError.y);
				
				//alpha = std::atan2(iError.y,iError.x); //this is the angle to the goal (straight line)

				//alpha = alpha +90*D2R -now.az; // passing to robot's frame

				//alpha = alpha/D2R; //passing radians to degrees

				//if(alpha < 0 | alpha >= 180) return 0;

				//if((dist<=1)&(laserData.ranges[alpha] >= dist)) nextPoint = m;

				if(dist>3){
					allClear = 0;
				}else{

					alpha = std::atan2(iError.y,iError.x); //this is the angle to the goal 

					//ROS_INFO("alpha %f", alpha/D2R);

					alpha = alpha +90*D2R -now.az; // passing to robot's frame
					alpha = alpha/D2R; //passing radians to degrees
					if ( alpha > 180 ) { alpha -= 360; }
					if ( alpha < -180 ) { alpha += 360;  }

					//ROS_INFO("now.az %f", now.az/D2R);

					min = alpha - 20;
					max = alpha + 20;

					/*ROS_INFO("alpha %f", alpha);
					ROS_INFO("min %d", min);
					ROS_INFO("max %d", max);
					ROS_INFO("error x %f", iError.x);
					ROS_INFO("error y %f", iError.y);*/

					/*if(min < 0) min = 0;
					if(max > 180) max = 180;

					ROS_INFO("alpha %f", alpha);
					ROS_INFO("min %d", min);
					ROS_INFO("max %d", max);
					ROS_INFO("error x %f", iError.x);
					ROS_INFO("error y %f", iError.y);

					ROS_INFO("point %d", k);
					ROS_INFO("angle %f", alpha);
					for(int i = min; i < max; i++)
					{
						if(laserData.ranges[i] < 2) {
							allClear = 0; 
							ROS_INFO("laser[%d] = %f", i, laserData.ranges[i]);
						}
					}*/
					if(min < 0 | max > 180) {
						allClear = 0;
						//ROS_INFO("point %d", k);
					}else{
						//ROS_INFO("point %d", k);
						//ROS_INFO("angle %f", alpha);
						for(int i = min; i < max; i++)
						{
							if(laserData.ranges[i] < 2) {
								allClear = 0; 
								//ROS_INFO("laser[%d] = %f", i, laserData.ranges[i]);
							}
						}
					}

				}

				if (allClear == 1){ 
					nextPoint = k;
					/*ROS_INFO("alpha = %f", alpha);
					ROS_INFO("iError.x = %f",iError.x);
					ROS_INFO("iError.y = %f", iError.y);
					ROS_INFO("dist = %f", dist);
					ROS_INFO("minimum = %d", minimum);
					for(int i = min; i < max; i++)
					{
						ROS_INFO("laser[%d] = %f", i, laserData.ranges[i]);
					}*/
				}
			
		}

		ROS_INFO("intermediate goal is:=%d", nextPoint);

		return nextPoint;
	}


	void BubblePlannerROS::pathVisualization()
	{
		// Visualization in rviz
	 
		/*visualization_msgs::Marker points;
	 
		points.header.frame_id = "/map";
	 
		points.header.stamp = ros::Time::now();
	 
		points.ns = "path_drawing";
	 
		points.action = visualization_msgs::Marker::ADD;
	 
		points.id = 0;
	 
		//points.pose.position.x = now.x;
		//points.pose.position.y = now.y;
		//points.pose.position.z = 0.5;
	 
		points.type = visualization_msgs::Marker::POINTS;
	 
		points.scale.x = 0.1;
		points.scale.y = 0.1;
	 
		points.color.g = 1.0f;
		points.color.a = 1.0;*/
	 
	        geometry_msgs::Point p;
 
	        p.x = now.x;
	        p.y = now.y;
	        p.z = 0.5;
 
      	        points.points.push_back(p);
   
	     
		path_pub.publish(points); 
	 
	 }	

}




