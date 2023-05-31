/*
 *  SLLIDAR ROS2 NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include "ttlidar_driver.h"

#include "math.h"
#include <unistd.h>
#include <signal.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

#define ROS2VERSION "1.0.1"

using namespace ttlidar;

bool need_exit = false;

class TTlidarNode : public rclcpp::Node
{

  private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service;

    std::string channel_type;
    std::string tcp_ip;
    std::string udp_ip;
    std::string serial_port;
    int tcp_port = 20108;
    int udp_port = 8089;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;
    float max_distance = 8.0;
    size_t angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
    std::string scan_mode;
    float scan_frequency;

    TTlidarDriver *drv;    



  public:
    TTlidarNode()
    : Node("ttlidar_node")
    {

      scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(10)));
      
    }

  private:    
    void init_param()
    {
        this->declare_parameter<std::string>("channel_type","serial");
        this->declare_parameter<std::string>("tcp_ip", "192.168.0.7");
        this->declare_parameter<int>("tcp_port", 20108);
        this->declare_parameter<std::string>("udp_ip","192.168.11.2");
        this->declare_parameter<int>("udp_port",8089);
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("serial_baudrate",1000000);
        this->declare_parameter<std::string>("frame_id","laser_frame");
        this->declare_parameter<bool>("inverted", false);
        this->declare_parameter<bool>("angle_compensate", false);
        this->declare_parameter<std::string>("scan_mode",std::string());
        this->declare_parameter<float>("scan_frequency",10);
              
        this->get_parameter_or<std::string>("channel_type", channel_type, "serial");
        this->get_parameter_or<std::string>("tcp_ip", tcp_ip, "192.168.0.7"); 
        this->get_parameter_or<int>("tcp_port", tcp_port, 20108);
        this->get_parameter_or<std::string>("udp_ip", udp_ip, "192.168.11.2"); 
        this->get_parameter_or<int>("udp_port", udp_port, 8089);
        this->get_parameter_or<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
        this->get_parameter_or<int>("serial_baudrate", serial_baudrate, 115200);// 115200 to A3
        this->get_parameter_or<std::string>("frame_id", frame_id, "laser_frame");
        this->get_parameter_or<bool>("inverted", inverted, false);
        this->get_parameter_or<bool>("angle_compensate", angle_compensate, false);
        this->get_parameter_or<std::string>("scan_mode", scan_mode, std::string());
        if(channel_type == "udp")
            this->get_parameter_or<float>("scan_frequency", scan_frequency, 10.0);
        else
            this->get_parameter_or<float>("scan_frequency", scan_frequency, 6.0);
    }

    
    bool start_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if(!drv)
           return false;
        if(drv->isConnected())
        {
            result_t     result;

            /*
            RCLCPP_DEBUG(this->get_logger(),"Start motor");
            result = drv->setMotorRpm(600,1000); // 600->6r/s    1000ms->1s
            if (IS_FAIL(result)) {
                RCLCPP_WARN(this->get_logger(), "Failed to start motor: %08x", ans);
                return false;
            }
            */

            result = drv->startScan(SCAN_MODE,5000); //TIMEOUT 5000ms->5s
            if (IS_OK(result)) { 
                RCLCPP_INFO(this->get_logger(),"TTldiar start scan ok!");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),"Error, cannot make the lidar start scan: %x\n", result);
                return false;
            }        
        } 
        else 
        {
            RCLCPP_INFO(this->get_logger(),"TTldiar  lost connection");
            return false;
        }

        return true;
    }
    
    bool stop_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if(!drv)
            return false;

        RCLCPP_DEBUG(this->get_logger(),"Stop motor");
        drv->setMotorRpm(0,1000); 
        return true;
    }


    void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub,
                  LIDAR_SCAN_INFO_T *scan_points,
                  size_t points_count, rclcpp::Time start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  float max_distance,
                  std::string frame_id)
    {
        static int scan_count = 0;
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

        scan_msg->header.stamp = start;
        scan_msg->header.frame_id = frame_id;
        scan_count++;

        bool reversed = (angle_max > angle_min);
        if ( reversed ) {
            scan_msg->angle_min =  M_PI - angle_max;
            scan_msg->angle_max =  M_PI - angle_min;
        } else {
            scan_msg->angle_min =  M_PI - angle_min;
            scan_msg->angle_max =  M_PI - angle_max;
        }
        //points_count = 360;
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(points_count-1);

        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / (double)(points_count-1);
        scan_msg->range_min = 0.10;
        scan_msg->range_max = max_distance;//8.0;

        scan_msg->intensities.resize(points_count);
        scan_msg->ranges.resize(points_count);
       

            //copy ttlidar_points to scan_msg
        for (size_t i = 0; i < points_count; i++)
        {
            float read_value = (float)scan_points[i].distance;
            if (read_value == 0.0)
                scan_msg->ranges[points_count- 1- i] = std::numeric_limits<float>::infinity();
            else
                scan_msg->ranges[points_count -1- i] = read_value;

        //	scan_msg.intensities[points_count -1- i] = 0;

        }   
    
        /*
            for (size_t i = 0; i < points_count; i++) {           
                scan_msg->ranges[points_count-1-i] = 2.0;
                scan_msg->intensities[points_count-1-i] = 5;
            }
        */
        
        
        RCLCPP_INFO(this->get_logger(),"scan_count=%d\n", scan_count);

        pub->publish(*scan_msg);
    }
    public:    
    int work_loop()
    {        
        init_param();
        /*
         int ver_major = SL_LIDAR_SDK_VERSION_MAJOR;
        int ver_minor = SL_LIDAR_SDK_VERSION_MINOR;
        int ver_patch = SL_LIDAR_SDK_VERSION_PATCH;
        RCLCPP_INFO(this->get_logger(),"SLLidar running on ROS2 package SLLidar.ROS2 SDK Version:" ROS2VERSION ", SLLIDAR SDK Version:%d.%d.%d",ver_major,ver_minor,ver_patch);   
        */
       
        // create the driver instance
        drv =   new TTlidarDriver();
        if (IS_FAIL(drv->connect(serial_port.c_str(), serial_baudrate))) 
        {
            RCLCPP_ERROR(this->get_logger(),"Error, Open serail port %s failed! \n",serial_port.c_str());    
                    
            delete drv;
            return -1;
        }

        RCLCPP_INFO(this->get_logger(),"TTLidar connected\n");    

        //set lidar rotate speed
        //drv->setMotorRpm(6,1000);

        //start scan
        if (!drv->startScan(SCAN_MODE,1000))
        {
            //delete drv;
        // return -1;
        }
        
        RCLCPP_INFO(this->get_logger(),"TTlidar start scan..........................\n");
        
  
        rclcpp::Time start_scan_time;
        rclcpp::Time end_scan_time;
        double scan_duration;
        result_t  result;
        while (rclcpp::ok() && !need_exit) {

            LIDAR_SCAN_INFO_T scan_points[360 *6];
            size_t points_count = 360*6;

            start_scan_time = this->now();
            result = drv->grabScanData(scan_points, points_count);   
            end_scan_time = this->now();
            scan_duration = (end_scan_time - start_scan_time).seconds();

            if (IS_OK(result)) 
            {
                RCLCPP_INFO(this->get_logger(),"TTlidar points_count = %d\n", points_count);                       
                // All the data is invalid, just publish them
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);
                publish_scan(scan_pub, scan_points, points_count,
                            start_scan_time, scan_duration, inverted,
                            angle_min, angle_max, max_distance,
                            frame_id);
                
            }

            rclcpp::spin_some(shared_from_this());
        }

        // done!
        drv->setMotorRpm(0,1000);
        drv->stopScan();
        RCLCPP_INFO(this->get_logger(),"Stop motor");

        return 0;
    }


};

void ExitHandler(int sig)
{
    (void)sig;
    need_exit = true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  
  auto TTlidar_node = std::make_shared<TTlidarNode>();
  signal(SIGINT,ExitHandler);
  int ret = TTlidar_node->work_loop();
  rclcpp::shutdown();
  return ret;
}

