#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <queue>
#include <fstream>
#include <random_numbers/random_numbers.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <pharos_msgs/SCBADUheader.h>
#include <pharos_msgs/CAN_GWAY_header.h>


using namespace std;

shared_ptr<queue<pharos_msgs::SCBADUheader::Ptr> > SCBADU_que_ (new queue<pharos_msgs::SCBADUheader::Ptr>);
shared_ptr<queue<sensor_msgs::Image::Ptr> > Img_que_ (new queue<sensor_msgs::Image::Ptr>);
shared_ptr<queue<pharos_msgs::CAN_GWAY_header::Ptr> > CAN_que_ (new queue<pharos_msgs::CAN_GWAY_header::Ptr>);

class TimeDelayGenerating
{
public:
    
    std::string host_ = "host";
    std::string rover_str_ = "rover";
    std::string station_str_ = "station";

    ros::NodeHandlePtr node_;
    ros::NodeHandlePtr pnode_;

    ros::Publisher delayed_SCBADU_pub_;
    ros::Publisher delayed_CAN_pub_;
    ros::Publisher delayed_Img_pub_;

    ros::Publisher SCBADU_time_delay_pub_;
    ros::Publisher CAN_time_delay_pub_;
    ros::Publisher Img_time_delay_pub_;

    ros::Subscriber SCBADU_sub_;
    ros::Subscriber CAN_sub_;
    ros::Subscriber Img_sub_;

    random_numbers::RandomNumberGenerator rng_;

    struct time_delay_params{
        int time_delay_type;
        double mean_time_delay;
        double min_time_delay;
        double max_time_delay;
        double std_dev;

        double time_delay{.0};
    };

    time_delay_params SCBADU_td_;
    time_delay_params CAN_td_;
    time_delay_params image_td_;

//    double time_delay_SCBADU_{.0};
//    double time_delay_CAN_{.0};
//    double time_delay_image_{.0};

    bool symmetric_;

    double TimeDelayConst(const double time_delay) {
//        time_delay_ = time_delay;
//        cout<<"Const time delay: " << time_delay_ << endl;
        return time_delay;
    }

    double TimeDelayUniform(const double min_time_delay, const double max_time_delay) {
        double time_delay = rng_.uniformReal(min_time_delay,max_time_delay);
//        cout<<"Uniform time delay: " << time_delay << endl;
//        if(time_delay < (min_time_delay+max_time_delay)*0.5){
//            time_delay = (min_time_delay+max_time_delay)*0.5 + (min_time_delay+max_time_delay)*0.5-time_delay;
//        }
        return time_delay;
    }

    double TimeDelayGaussian(const double mean_time_delay, const double std_dev) {

        double time_delay = rng_.gaussian(mean_time_delay,std_dev);
//        if(time_delay < mean_time_delay){
//            time_delay = mean_time_delay*2.0-time_delay;
//        }
        if(time_delay < 0 ) time_delay = 0.0;
//        cout<<"Gaussian time delay: " << time_delay << endl;

        return time_delay;
    }

    double TimeDelaySinewave(const double mean_time_delay, const double t) {
        
        double time_delay = sin(t);
//        time_delay_ = time_delay;
//        cout<<"Const time delay: " << time_delay_ << endl;
        return time_delay;
    }

    void SCBADU_Callback(const pharos_msgs::SCBADUheader::ConstPtr &msg){

        pharos_msgs::SCBADUheader delayed_SCBADU;
        pharos_msgs::SCBADUheader::Ptr input_data (new pharos_msgs::SCBADUheader);
        *input_data = *msg;

        if(SCBADU_td_.time_delay == 0.0){
            delayed_SCBADU = *msg;
            // delayed_SCBADU.header.stamp = ros::Time::now();
            delayed_SCBADU_pub_.publish(delayed_SCBADU);
            return;
        }

        if(SCBADU_que_->empty()){
            SCBADU_que_->push(input_data);
        }
        else{
            double buffer_time_length;
            SCBADU_que_->push(input_data);

            buffer_time_length = (SCBADU_que_->back()->header.stamp - SCBADU_que_->front()->header.stamp).toSec();
//            std::cout << "buffer time length: " << buffer_time_length << endl;
            if(buffer_time_length > SCBADU_td_.time_delay){
                delayed_SCBADU = *SCBADU_que_->front();
                // delayed_CAN.header.stamp = ros::Time::now();
                delayed_SCBADU_pub_.publish(delayed_SCBADU);
                SCBADU_que_->pop();
                buffer_time_length = (SCBADU_que_->back()->header.stamp - SCBADU_que_->front()->header.stamp).toSec();

                while(buffer_time_length > SCBADU_td_.time_delay){
                    delayed_SCBADU = *SCBADU_que_->front();
                    // delayed_CAN.header.stamp = ros::Time::now();
                    delayed_SCBADU_pub_.publish(delayed_SCBADU);
                    SCBADU_que_->pop();
                    if(SCBADU_que_->empty()){
                        ROS_ERROR("SCBADU Buffer is empty!");
                        break;
                    }
                    buffer_time_length = (SCBADU_que_->back()->header.stamp - SCBADU_que_->front()->header.stamp).toSec();
                }
            }
            else{

            }
        }
    }


    void CAN_Callback(const pharos_msgs::CAN_GWAY_header::ConstPtr &msg){

        pharos_msgs::CAN_GWAY_header delayed_CAN;
        pharos_msgs::CAN_GWAY_header::Ptr input_data (new pharos_msgs::CAN_GWAY_header);
        *input_data = *msg;

        if(CAN_td_.time_delay == 0.0){
            delayed_CAN = *msg;
            // delayed_CAN.header.stamp = ros::Time::now();
            delayed_CAN_pub_.publish(delayed_CAN);
            return;
        }

        if(CAN_que_->empty()){
            CAN_que_->push(input_data);
        }
        else{
            double buffer_time_length;
            CAN_que_->push(input_data);

            buffer_time_length = (CAN_que_->back()->header.stamp - CAN_que_->front()->header.stamp).toSec();
//            std::cout << "buffer time length: " << buffer_time_length << std::endl;
            if(buffer_time_length > CAN_td_.time_delay){
                delayed_CAN = *CAN_que_->front();
                // delayed_CAN.header.stamp = ros::Time::now();
                delayed_CAN_pub_.publish(delayed_CAN);
                CAN_que_->pop();
                buffer_time_length = (CAN_que_->back()->header.stamp - CAN_que_->front()->header.stamp).toSec();

                while(buffer_time_length > CAN_td_.time_delay){
                    delayed_CAN = *CAN_que_->front();
                    // delayed_CAN.header.stamp = ros::Time::now();
                    delayed_CAN_pub_.publish(delayed_CAN);
                    CAN_que_->pop();

                    if(CAN_que_->empty()){
                        ROS_ERROR("CAN Buffer is empty!");
                        break;
                    }
                    buffer_time_length = (CAN_que_->back()->header.stamp - CAN_que_->front()->header.stamp).toSec();
                }
            }
            else{

            }
        }
    }

    void CameraImageCallback(const sensor_msgs::Image::ConstPtr &msg) {

        sensor_msgs::ImagePtr delayed_Img (new sensor_msgs::Image);
        sensor_msgs::ImagePtr input_data (new sensor_msgs::Image);
        *input_data = *msg;
        input_data->header.frame_id += "2";

        if(image_td_.time_delay == 0.0){
            *delayed_Img = *msg;
            // delayed_Img->header.stamp = ros::Time::now();
            delayed_Img_pub_.publish(delayed_Img);
            return;
        }

        if(Img_que_->empty()){
            Img_que_->push(input_data);
        }
        else{
            double buffer_time_length;
            Img_que_->push(input_data);

            buffer_time_length = (Img_que_->back()->header.stamp - Img_que_->front()->header.stamp).toSec();
//            std::cout << "buffer time length: " << buffer_time_length << std::endl;
            if(buffer_time_length > image_td_.time_delay){
                *delayed_Img = *Img_que_->front();
                // delayed_Img->header.stamp = ros::Time::now();
                delayed_Img_pub_.publish(delayed_Img);
                Img_que_->pop();
                buffer_time_length = (Img_que_->back()->header.stamp - Img_que_->front()->header.stamp).toSec();

                while(buffer_time_length > image_td_.time_delay){
                    *delayed_Img = *Img_que_->front();
                    // delayed_Img->header.stamp = ros::Time::now();
                    delayed_Img_pub_.publish(delayed_Img);
                    Img_que_->pop();

                    if(Img_que_->empty()){
                        ROS_ERROR("Image Buffer is empty!");
                        break;
                    }
                    buffer_time_length = (Img_que_->back()->header.stamp - Img_que_->front()->header.stamp).toSec();
                }
            }
        }
    }


    // Load parameters etc
    int init()
    {
        node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));


        pnode_->param<std::string>("host", host_, station_str_);
        std::cout<<"host: "<<host_<<std::endl;

        std::string prefix;
        pnode_->param<std::string>("prefix", prefix, ""); //"/relay"

        if(host_.compare(rover_str_) == 0){
            pnode_->param("SCBADU/time_delay_type", SCBADU_td_.time_delay_type, 1);
            pnode_->param("SCBADU/mean_time_delay", SCBADU_td_.mean_time_delay, 0.1);
            pnode_->param("SCBADU/min_time_delay", SCBADU_td_.min_time_delay, 0.0);
            pnode_->param("SCBADU/max_time_delay", SCBADU_td_.max_time_delay, 0.2);
            pnode_->param("SCBADU/std_dev", SCBADU_td_.std_dev, 0.05);

            pnode_->param("CAN/time_delay_type", CAN_td_.time_delay_type, 1);
            pnode_->param("CAN/mean_time_delay", CAN_td_.mean_time_delay, 0.1);
            pnode_->param("CAN/min_time_delay", CAN_td_.min_time_delay, 0.0);
            pnode_->param("CAN/max_time_delay", CAN_td_.max_time_delay, 0.2);
            pnode_->param("CAN/std_dev", CAN_td_.std_dev, 0.05);

            SCBADU_sub_ = node_->subscribe("/station/data1", 10, &TimeDelayGenerating::SCBADU_Callback, this);
            CAN_sub_ =  node_->subscribe("/station/data2", 10, &TimeDelayGenerating::CAN_Callback, this);

            delayed_SCBADU_pub_ = node_->advertise<pharos_msgs::SCBADUheader>(prefix+"/master/interface",10);
            delayed_CAN_pub_ = node_->advertise<pharos_msgs::CAN_GWAY_header>(prefix+"/ping/CAN_Gateway",10);

            SCBADU_time_delay_pub_ = node_->advertise<std_msgs::Float64>("time_delay/master/interface",10);
            CAN_time_delay_pub_ = node_->advertise<std_msgs::Float64>("time_delay/CAN_Gateway",10);

        }else if(host_.compare(station_str_) == 0){
            pnode_->param("CAN/time_delay_type", CAN_td_.time_delay_type, 1);
            pnode_->param("CAN/mean_time_delay", CAN_td_.mean_time_delay, 0.1);
            pnode_->param("CAN/min_time_delay", CAN_td_.min_time_delay, 0.0);
            pnode_->param("CAN/max_time_delay", CAN_td_.max_time_delay, 0.2);
            pnode_->param("CAN/std_dev", CAN_td_.std_dev, 0.05);

            std::string image_name;
            pnode_->param<std::string>("image_name", image_name, "/rviz_cam2/image_raw");
            pnode_->param("image/time_delay_type", image_td_.time_delay_type, 1);
            pnode_->param("image/mean_time_delay", image_td_.mean_time_delay, 0.1);
            pnode_->param("image/min_time_delay", image_td_.min_time_delay, 0.0);
            pnode_->param("image/max_time_delay", image_td_.max_time_delay, 0.2);
            pnode_->param("image/std_dev", image_td_.std_dev, 0.05);

            CAN_sub_ =  node_->subscribe("/rover/data1", 10, &TimeDelayGenerating::CAN_Callback, this);
            Img_sub_ = node_->subscribe("/front_cam/image_tele/h264", 10, &TimeDelayGenerating::CameraImageCallback, this);

            delayed_CAN_pub_ = node_->advertise<pharos_msgs::CAN_GWAY_header>(prefix+"/CAN_Gateway",10);
            delayed_Img_pub_ = node_->advertise<sensor_msgs::Image>(prefix+image_name,10);

            CAN_time_delay_pub_ = node_->advertise<std_msgs::Float64>("time_delay/CAN_Gateway",10);
            Img_time_delay_pub_ = node_->advertise<std_msgs::Float64>("time_delay/rviz_cam2",10);
        }

        return 0;
    }


    // Publish data
    void publish()
    {

        // ofstream log_file("/home/jun/logs/time_delay.txt");

        ros::Rate loop_rate(100);
        while (node_->ok()) {

            if(SCBADU_td_.time_delay_type == 1){
                SCBADU_td_.time_delay = TimeDelayConst(SCBADU_td_.mean_time_delay);
            }
            else if(SCBADU_td_.time_delay_type == 2){
                SCBADU_td_.time_delay = TimeDelayUniform(SCBADU_td_.min_time_delay,SCBADU_td_.max_time_delay);
            }
            else if(SCBADU_td_.time_delay_type == 3){
                SCBADU_td_.time_delay = TimeDelayGaussian(SCBADU_td_.mean_time_delay,SCBADU_td_.std_dev);
            }

            if(CAN_td_.time_delay_type == 1){
                CAN_td_.time_delay = TimeDelayConst(CAN_td_.mean_time_delay);
            }
            else if(CAN_td_.time_delay_type == 2){
                CAN_td_.time_delay = TimeDelayUniform(CAN_td_.min_time_delay,CAN_td_.max_time_delay);
            }
            else if(CAN_td_.time_delay_type == 3){
                CAN_td_.time_delay = TimeDelayGaussian(CAN_td_.mean_time_delay,CAN_td_.std_dev);
            }

            if(image_td_.time_delay_type == 1){
                image_td_.time_delay = TimeDelayConst(image_td_.mean_time_delay);
            }
            else if(image_td_.time_delay_type == 2){
                image_td_.time_delay = TimeDelayUniform(image_td_.min_time_delay,image_td_.max_time_delay);
            }
            else if(image_td_.time_delay_type == 3){
                image_td_.time_delay = TimeDelayGaussian(image_td_.mean_time_delay,image_td_.std_dev);
            }

            std_msgs::Float64 td_SCBADU, td_CAN, td_img;

            td_SCBADU.data = SCBADU_td_.time_delay;
            td_CAN.data = CAN_td_.time_delay;
            td_img.data = image_td_.time_delay;

            SCBADU_time_delay_pub_.publish(td_SCBADU);
            CAN_time_delay_pub_.publish(td_CAN);
            Img_time_delay_pub_.publish(td_img);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_delay_generating_node initialization");

    TimeDelayGenerating tdg;
    if(tdg.init())
    {
        ROS_FATAL("time_delay_generating_node initialization failed");
        return -1;
    }

    tdg.publish();

    return 0;
}


