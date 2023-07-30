#include <ros/ros.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "pharos_msgs/RacingWheelStamped.h"

#define TRIGGER_MIN 256
#define TRIGGER_MAX 65535

double map(double x, double in_min, double in_max, double out_min, double out_max, bool limit = false) {
    double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if(limit) return std::min(std::max(temp, out_min), out_max);
    else return temp;
}

double trigger_map(double x){
    return round(map(x, TRIGGER_MIN, TRIGGER_MAX, 0, 100, true)*100)/100;
}

class TLCM_Pedal {

private:
    ros::Subscriber sub_target;
    ros::Publisher pub_state;
    ros::Timer timer;

    // device info
    int m_device_handle;
    int m_axis_code = ABS_X;
    int m_axis_min;
    int m_axis_max;

    int m_clutch_code = ABS_Y;
    int m_brake_code = ABS_Z;
    int m_accel_code = ABS_X;

    pharos_msgs::RacingWheelStamped m_state_msg;

    // rosparam
    std::string m_device_name;
    double m_loop_rate;
    double m_max_torque;
    double m_min_torque;
    double m_brake_position;
    double m_brake_torque_rate;
    double m_auto_centering_max_torque;
    double m_auto_centering_max_position;
    double m_eps;
    bool m_auto_centering;

public:
    TLCM_Pedal();

private:
    void loop(const ros::TimerEvent&);
    int testBit(int bit, unsigned char *array);
    void initDevice();
};


TLCM_Pedal::TLCM_Pedal() {

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pub_state = pnh.advertise<pharos_msgs::RacingWheelStamped>("/tlcm/state", 1);

    m_state_msg.header.stamp = ros::Time::now();
    m_state_msg.header.frame_id = "tlcm";

    pnh.getParam("device_name", m_device_name);
    pnh.getParam("loop_rate", m_loop_rate);
    pnh.getParam("max_torque", m_max_torque);
    pnh.getParam("min_torque", m_min_torque);
    pnh.getParam("brake_position", m_brake_position);
    pnh.getParam("brake_torque_rate", m_brake_torque_rate);
    pnh.getParam("auto_centering_max_torque", m_auto_centering_max_torque);
    pnh.getParam("auto_centering_max_position", m_auto_centering_max_position);
    pnh.getParam("eps", m_eps);
    pnh.getParam("auto_centering", m_auto_centering);

    initDevice();

    ros::Duration(1).sleep();
    timer = pnh.createTimer(ros::Duration(m_loop_rate), &TLCM_Pedal::loop, this);
}

// update input event with timer callback
void TLCM_Pedal::loop(const ros::TimerEvent&) {
    struct input_event event;
    // get current state
    while(read(m_device_handle, &event, sizeof(event)) == sizeof(event)) {
        if(event.type == EV_ABS){
            if(event.code == m_clutch_code) m_state_msg.state.clutch = trigger_map(event.value);
            else if(event.code == m_brake_code) m_state_msg.state.brake = trigger_map(event.value);
            else if(event.code == m_accel_code) m_state_msg.state.accel = trigger_map(event.value);
        }
    }
    m_state_msg.header.stamp = ros::Time::now();
    pub_state.publish(m_state_msg);

}


// initialize force feedback device
void TLCM_Pedal::initDevice() {
    // setup device
    unsigned char key_bits[1+KEY_MAX/8/sizeof(unsigned char)];
    unsigned char abs_bits[1+ABS_MAX/8/sizeof(unsigned char)];
    unsigned char ff_bits[1+FF_MAX/8/sizeof(unsigned char)];
    struct input_event event;
    struct input_absinfo abs_info;
    m_device_handle = open(m_device_name.c_str(), O_RDWR|O_NONBLOCK);
    if (m_device_handle < 0) {
        std::cout << "ERROR: cannot open device : "<< m_device_name << std::endl;
        ROS_WARN("Check event port with >> evtest --grab");
        exit(1);
    }else{
        std::cout << "device opened" << std::endl;
    }

    // which axes has the device?
    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0) {
        std::cout << "ERROR: cannot get abs bits" << std::endl;
        exit(1);
    }

    // get axis value range
    if (ioctl(m_device_handle, EVIOCGABS(m_axis_code), &abs_info) < 0) {
        std::cout << "ERROR: cannot get axis range" << std::endl;
        exit(1);
    }
    m_axis_max = abs_info.maximum;
    m_axis_min = abs_info.minimum;
    if (m_axis_min >= m_axis_max) {
        std::cout << "ERROR: axis range has bad value" << std::endl;
        exit(1);
    }
}


// util for initDevice()
int TLCM_Pedal::testBit(int bit, unsigned char *array) {

    return ((array[bit / (sizeof(unsigned char) * 8)] >> (bit % (sizeof(unsigned char) * 8))) & 1);
}


int main(int argc, char **argv ){

    ros::init(argc, argv, "pharos_interface_driver_node");
    TLCM_Pedal tlcm_pedal;
    ros::spin();
    return(0);
}
