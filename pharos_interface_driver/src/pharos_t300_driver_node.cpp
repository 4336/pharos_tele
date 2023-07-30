#include <ros/ros.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "pharos_interface_driver/ForceFeedback.h"
#include "pharos_msgs/RacingWheelStamped.h"

#define BUTTON_NUM 13
#define TRIGGER_MIN 1010
#define TRIGGER_MAX 10

double map(double x, double in_min, double in_max, double out_min, double out_max, bool limit = false) {
    double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if(limit) return std::min(std::max(temp, out_min), out_max);
    else return temp;
}

double trigger_map(double x){
    return map(x, TRIGGER_MIN, TRIGGER_MAX, 0, 100, true);
}

class T300ForceFeedback {

private:
    ros::Subscriber sub_target, sub_pedal;
    ros::Publisher pub_state;
    ros::Timer timer;

    pharos_msgs::RacingWheelStamped tlcm_pedal_;

    // device info
    int m_device_handle;
    int m_axis_code = ABS_X;
    int m_axis_min;
    int m_axis_max;

    int m_steering_code = ABS_X;
    int m_clutch_code = ABS_Y;
    int m_brake_code = ABS_RZ;
    int m_accel_code = ABS_Z;
    int m_gear_down_code = BTN_TRIGGER;
    int m_gear_up_code = BTN_THUMB;

    int m_button_code[BUTTON_NUM] = {ABS_HAT0Y, ABS_HAT0X,
                            BTN_THUMB2, BTN_PINKIE, BTN_TOP, BTN_TOP2,
                            BTN_BASE4, BTN_BASE3, BTN_BASE, BTN_BASE2, 300,
                            BTN_BASE5, BTN_BASE6};

    pharos_msgs::RacingWheelStamped m_state_msg, m_override_msg;

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

    // variables
    pharos_interface_driver::ForceFeedback m_target;
    bool m_is_target_updated = false;
    bool m_is_brake_range = false;
    struct ff_effect m_effect;
    double m_position;
    double m_torque;
    double m_attack_length;

public:
    T300ForceFeedback();
    ~T300ForceFeedback();

private:
    void targetCallback(const pharos_interface_driver::ForceFeedback::ConstPtr &in_target);
    void pedalCallback(const pharos_msgs::RacingWheelStamped::ConstPtr &msg);
    void loop(const ros::TimerEvent&);
    int testBit(int bit, unsigned char *array);
    void initDevice();
    void calcRotateForce(double &torque, double &attack_length, const pharos_interface_driver::ForceFeedback &target, const double &current_position);
    void calcCenteringForce(double &torque, const pharos_interface_driver::ForceFeedback &target, const double &current_position);
    void uploadForce(const double &position, const double &force, const double &attack_length);
};


T300ForceFeedback::T300ForceFeedback() {

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    sub_target = pnh.subscribe("/t300/ff_cmd", 1, &T300ForceFeedback::targetCallback, this);
    sub_pedal = pnh.subscribe("/tlcm/state", 1, &T300ForceFeedback::pedalCallback, this);
    pub_state = pnh.advertise<pharos_msgs::RacingWheelStamped>("/t300/state", 1);

    m_state_msg.header.stamp = ros::Time::now();
    m_state_msg.header.frame_id = "t300";
    for(int i=0; i<BUTTON_NUM; i++) m_state_msg.state.buttons.push_back(0);

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
    timer = pnh.createTimer(ros::Duration(m_loop_rate), &T300ForceFeedback::loop, this);
}

T300ForceFeedback::~T300ForceFeedback() {

/*
 * Force feedback effect types
 */
// #define FF_RUMBLE	0x50
// #define FF_PERIODIC	0x51
// #define FF_CONSTANT	0x52
// #define FF_SPRING	0x53
// #define FF_FRICTION	0x54
// #define FF_DAMPER	0x55
// #define FF_INERTIA	0x56
// #define FF_RAMP		0x57

// #define FF_EFFECT_MIN	FF_RUMBLE
// #define FF_EFFECT_MAX	FF_RAMP


    m_effect.type = FF_CONSTANT; // 0x52 @ ?/uapi/linux/input.h
    m_effect.id = -1;
    m_effect.u.constant.level = 0;
    m_effect.direction = 0;
    // upload m_effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        std::cout << "failed to upload m_effect" << std::endl;
    }
}


// update input event with timer callback
void T300ForceFeedback::loop(const ros::TimerEvent&) {

    struct input_event event;
    double last_position = m_position;
    // get current state
    while (read(m_device_handle, &event, sizeof(event)) == sizeof(event)) {
        if (event.type == EV_ABS){
            if(event.code == m_steering_code){
                m_position = (event.value - (m_axis_max + m_axis_min) * 0.5) * 2 / (m_axis_max - m_axis_min); //m_axis_max=65535
        		m_state_msg.state.steering = ((m_axis_max + m_axis_min) * 0.5 - event.value) / (m_axis_max - m_axis_min) * 900;
            }else if(event.code == m_clutch_code) m_state_msg.state.clutch = trigger_map(event.value);
            else if(event.code == m_brake_code) m_state_msg.state.brake = trigger_map(event.value);
            else if(event.code == m_accel_code) m_state_msg.state.accel = trigger_map(event.value);
            else if(event.code == m_button_code[0]) m_state_msg.state.buttons[0] =-event.value;
            else if(event.code == m_button_code[1])m_state_msg.state.buttons[1] = event.value;

        }else if (event.type == EV_KEY){
            if(event.code == m_gear_down_code) m_state_msg.state.gear_down = event.value;
            else if(event.code ==  m_gear_up_code) m_state_msg.state.gear_up = event.value;
            else{
                for(int i=2; i<BUTTON_NUM; i++){
                    if(event.code == m_button_code[i]) m_state_msg.state.buttons[i] = event.value;
                }
            }
        }
    }

    m_state_msg.header.stamp = ros::Time::now();

    if(ros::Time::now() - tlcm_pedal_.header.stamp < ros::Duration(0.1) && 
    (tlcm_pedal_.state.clutch > 0 || tlcm_pedal_.state.brake > 0 || tlcm_pedal_.state.accel > 0)){ 
        m_override_msg = m_state_msg;
        
        if(tlcm_pedal_.state.clutch > 0) m_override_msg.state.clutch = tlcm_pedal_.state.clutch;
        if(tlcm_pedal_.state.brake > 0) m_override_msg.state.brake = tlcm_pedal_.state.brake;
        if(tlcm_pedal_.state.accel > 0) m_override_msg.state.accel = tlcm_pedal_.state.accel;
        pub_state.publish(m_override_msg); //Override T-LCM Pedal
    }else{
        pub_state.publish(m_state_msg);
    }

    if (m_is_brake_range || m_auto_centering) {
        calcCenteringForce(m_torque, m_target, m_position);
        m_attack_length = 0.0;

    } else {
        calcRotateForce(m_torque, m_attack_length, m_target, m_position);
        m_is_target_updated = false;
    }

    uploadForce(m_target.position, m_torque, m_attack_length);
}


void T300ForceFeedback::calcRotateForce(double &torque,
                                       double &attack_length,
                                       const pharos_interface_driver::ForceFeedback &target,
                                       const double &current_position) {

    double diff = target.position - current_position;
    double direction = (diff > 0.0) ? 1.0 : -1.0;

    if (fabs(diff) < m_eps) {
        torque = 0.0;
        attack_length = 0.0;

    } else if (fabs(diff) < m_brake_position) {
        m_is_brake_range = true;
        torque = target.torque * m_brake_torque_rate * -direction;
        attack_length = m_loop_rate;

    } else {
        torque = target.torque * direction;
        attack_length = m_loop_rate;
    }
}


void T300ForceFeedback::calcCenteringForce(double &torque,
                                          const pharos_interface_driver::ForceFeedback &target,
                                          const double &current_position) {

    double diff = target.position - current_position;
    double direction = (diff > 0.0) ? 1.0 : -1.0;

    if (fabs(diff) < m_eps)
        torque = 0.0;

    else {
        double torque_range = m_auto_centering_max_torque - m_min_torque;
        double power = (fabs(diff) - m_eps) / (m_auto_centering_max_position - m_eps);
        double buf_torque = power * torque_range + m_min_torque;
        torque = std::min(buf_torque, m_auto_centering_max_torque) * direction;
    }
}


// update input event with writing information to the event file
void T300ForceFeedback::uploadForce(const double &position,
                                   const double &torque,
                                   const double &attack_length) {

    // std::cout << torque << std::endl;
    // set effect
    m_effect.u.constant.level = 0x7fff * std::min(torque, m_max_torque);
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_level = 0; /* 0x7fff * force / 2 */
    m_effect.u.constant.envelope.attack_length = attack_length;
    m_effect.u.constant.envelope.fade_level = 0;
    m_effect.u.constant.envelope.fade_length = attack_length;

    // upload effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        std::cout << "failed to upload effect" << std::endl;
    }
}


// get target information of wheel control from ros message
void T300ForceFeedback::targetCallback(const pharos_interface_driver::ForceFeedback::ConstPtr &in_msg) {

    if (m_target.position == in_msg->position && m_target.torque == fabs(in_msg->torque)) {
        m_is_target_updated = false;

    } else {
        m_target = *in_msg;
        m_target.torque = fabs(m_target.torque);
        m_is_target_updated = true;
        m_is_brake_range = false;
    }
}


// get target information of wheel control from ros message
void T300ForceFeedback::pedalCallback(const pharos_msgs::RacingWheelStamped::ConstPtr &msg) {
    tlcm_pedal_ = *msg;
}


// initialize force feedback device
void T300ForceFeedback::initDevice() {
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

    } else {std::cout << "device opened" << std::endl;}

    // which axes has the device?
    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0) {
        std::cout << "ERROR: cannot get abs bits" << std::endl;
        exit(1);
    }

    // get some information about force feedback
    memset(ff_bits, 0, sizeof(ff_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0) {
        std::cout << "ERROR: cannot get ff bits" << std::endl;
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

    // check force feedback is supported?
    if(!testBit(FF_CONSTANT, ff_bits)) {
        std::cout << "ERROR: force feedback is not supported" << std::endl;
        exit(1);

    } else { std::cout << "force feedback supported" << std::endl; }

    // auto centering off
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = FF_AUTOCENTER;
    event.value = 0;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event)) {
        std::cout << "failed to disable auto centering" << std::endl;
        exit(1);
    }

    // init effect and get effect id
    memset(&m_effect, 0, sizeof(m_effect));
    m_effect.type = FF_CONSTANT;
    m_effect.id = -1; // initial value
    m_effect.trigger.button = 0;
    m_effect.trigger.interval = 0;
    m_effect.replay.length = 0xffff;  // longest value
    m_effect.replay.delay = 0; // delay from write(...)
    m_effect.u.constant.level = 0;
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_length = 0;
    m_effect.u.constant.envelope.attack_level = 0;
    m_effect.u.constant.envelope.fade_length = 0;
    m_effect.u.constant.envelope.fade_level = 0;

    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        std::cout << "failed to upload m_effect" << std::endl;
        exit(1);
    }

    // start m_effect
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = m_effect.id;
    event.value = 1;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event)) {
        std::cout << "failed to start event" << std::endl;
        exit(1);
    }
}


// util for initDevice()
int T300ForceFeedback::testBit(int bit, unsigned char *array) {

    return ((array[bit / (sizeof(unsigned char) * 8)] >> (bit % (sizeof(unsigned char) * 8))) & 1);
}


int main(int argc, char **argv ){

    ros::init(argc, argv, "pharos_interface_driver_node");
    T300ForceFeedback t300_ff;
    ros::spin();
    return(0);
}
