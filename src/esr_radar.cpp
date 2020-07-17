#include "esr_radar.h"
#include <iostream>
#include <ros/ros.h>

using std::cout;
using std::endl;
using std::hex;

EsrRadar::EsrRadar()
{
	bbox_array_.header.frame_id = "esr_radar";
	bbox_array_.boxes.reserve(64);
	bbox_.header.frame_id = "esr_radar";
	bbox_.pose.position.z = 1.0;
	bbox_.dimensions.x = bbox_.dimensions.y = bbox_.dimensions.z = 1.0;

	esr_objects_.header.frame_id = "esr_radar";
	esr_objects_.objects.reserve(64);
	
	diagnostic_msgs_.hardware_id = "esr_radar";
	diagnostic_msgs_.level = diagnostic_msgs_.OK;
}

bool EsrRadar::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	nh_private.param<std::string>("from_can_topic", from_can_topic_, "/from_usbcan");
	nh_private.param<std::string>("to_can_topic", to_can_topic_, "/to_usbcan");
	nh_private.param<int>  ("install_height",install_height_,20); //cm
	nh_private.param<float>("install_angle",install_angle_,0.0); //deg
	nh_private.param<bool> ("is_sendMsgToEsr",is_sendMsgToEsr_,false);

	pub_bbox_    = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/esr_bboxes",2);
	pub_objects_ = nh.advertise<esr_radar::ObjectArray>("/esr_objects",5);
	sub_can_     = nh.subscribe(from_can_topic_,100,&EsrRadar::canMsg_callback, this);
	pub_diagnostic_=nh.advertise<diagnostic_msgs::DiagnosticStatus>("/sensors/diagnostic",1);
	
	ROS_INFO("esr radar initialization complete.");
	
	return true;
}

void EsrRadar::run()
{
	if(is_sendMsgToEsr_)
	{
		ros::NodeHandle nh;
		pub_can_ = nh.advertise<can_msgs::FrameArray>(to_can_topic_,5);
		timer_ = nh.createTimer(ros::Duration(0.05),&EsrRadar::sendMsgToEsr,this);
	}
		
	ros::spin();
}

void EsrRadar::canMsg_callback(const can_msgs::FrameArray::ConstPtr& msg)
{
	if(msg->header.frame_id != "r") 
		return;

	for(int i=0; i<msg->frames.size(); ++i)
		parse_msg(msg->frames[i]);
}

void show_canmsg(const can_msgs::Frame &frame)
{
	cout << "ID:" << hex << frame.id << "\t";
	for(size_t i=0;i<frame.len;i++)
		printf("%x\t",frame.data[i]);
	printf("\n");
}

void EsrRadar::parse_msg(const can_msgs::Frame &frame)
{
	static uint16_t scan_index; 
	static size_t last_frame_id = 0x0;

	//一帧目标获取完毕
	if(frame.id == 0x4E0 /*|| frame.id < last_frame_id*/)
	{
		/*
		if(frame.id == 0x4E0)
			scan_index = frame.data[3]*256 + frame.data[4];
		else
			scan_index +=1;
		*/
		esr_objects_.size = esr_objects_.objects.size();
		if(esr_objects_.size == 0)
			return;
		pub_objects_.publish(esr_objects_);

		if(pub_bbox_.getNumSubscribers())
		{
			for(const auto& object:esr_objects_.objects)
			{
				bbox_.pose.position.x = object.x;
				bbox_.pose.position.y = object.y;	
				//if(object.status != CoastedTarget)
					bbox_array_.boxes.push_back(bbox_);
			}
			bbox_array_.header.stamp = esr_objects_.header.stamp;
			pub_bbox_.publish(bbox_array_);
			bbox_array_.boxes.clear();
		}
		esr_objects_.objects.clear();
		
		static int diagnostic_cnt = 0;
		if(pub_diagnostic_.getNumSubscribers() && (++diagnostic_cnt)%10==0)
		{
			pub_diagnostic_.publish(diagnostic_msgs_);
		}
	}
	
	last_frame_id = frame.id;
	
	if(frame.id <= 0x53f && frame.id >=0x500)
	{
		uint8_t measurementStatus = (frame.data[1]&0xE0) >> 5;
		
		if(//measurementStatus != NewTarget && 
		   measurementStatus != UpdateTarget && 
		   measurementStatus != CoastedTarget)
			return;
		
		uint16_t u16_tempAngle = (frame.data[1] &0x1F);
		u16_tempAngle <<=5;
		u16_tempAngle += ((frame.data[2] &0xf8)>>3) ;
		
		int16_t s16_angle = (int16_t)u16_tempAngle;
		
		if((frame.data[1]&0x10) !=0 )
			s16_angle -=1024;
		
		uint16_t u16_distance = (frame.data[2] &0x07);
		u16_distance <<=8;
		u16_distance += frame.data[3];
		
		uint16_t u16_tempTargetSpeed = frame.data[6] & 0x1f; 
		u16_tempTargetSpeed <<= 8;
		u16_tempTargetSpeed += frame.data[7];
		
		int16_t s16_targetSpeed =(int16_t)u16_tempTargetSpeed;
		
		if((frame.data[6]&0x20) !=0)
			s16_targetSpeed -= 8192;
		
		float azimuth = s16_angle*0.1 + install_angle_;  //left is negative(-) 
		float distance = u16_distance*0.1;
		float speed = s16_targetSpeed*0.01; // m/s
		
		if(distance>100.0 || distance < 0.2)
			return;
		
		float x = distance*sin(azimuth*M_PI/180.0);
		float y = distance*cos(azimuth*M_PI/180.0);
		
		esr_object_.azimuth = azimuth;
		esr_object_.distance = distance;
		esr_object_.speed = speed;
		esr_object_.x = x;
		esr_object_.y = y;
		esr_object_.status = measurementStatus;
		esr_object_.id = frame.id - 0x500;
		esr_objects_.objects.push_back(esr_object_);
		if(esr_objects_.objects.size() == 1)
			esr_objects_.header.stamp = ros::Time::now();
		
		/*
		switch(measurementStatus)
		{
			case 1: //new target  新检测到的目标
				cout << std::hex << frame.id << "   new     target ----> angle:" << azimuth << "\t range:" 
					 << distance <<"\tspeed:" << speed << endl;
				break;
			case 3: //update target 更新目标
				cout << std::hex << frame.id << "   update  target ----> angle:" << azimuth << "\t range:" 
					 << distance <<"\tspeed:" << speed << endl;
				break;
			case 4: //coasted target 滑行目标，之前检测到的目标，当前时刻未检测到，根据其运动趋势估计当前状态
				cout << std::hex << frame.id << "   coasted target ----> angle:" << azimuth << "\t range:" 
					 << distance <<"\tspeed:" << speed << endl;
				break;
					
			default:
				break;
		}
		*/
	}
	
}

void EsrRadar::sendMsgToEsr(const ros::TimerEvent&)
{
	can_msgs::FrameArray frame_array;
	can_msgs::Frame frame;
	frame.id = 0x5F2;
	frame.len = 8;
	
	uint8_t installHeight = 20;
	 
	frame.data[4] &= 0x80; //clear low 7bits
	frame.data[4] |= installHeight & 0x7f;//install_height
	
	frame_array.frames.push_back(frame);
	frame_array.header.frame_id = "w";
	pub_can_.publish(frame_array);
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"esr_radar_node");
	EsrRadar app;
	
	if(app.init())
		app.run();
	
	return 0;
}
