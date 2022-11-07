#include <path_predict.h>

namespace pharos_path_prediction
{
  Prediction::Prediction(ros::NodeHandle nh)
  {
    init_ = false;

    ros::NodeHandle pnh("~");
    // pnh.param("view", view_, view_);

    //Pubscribe

    // Subscribe
    // sub = it.subscribe(topic, 1, &View::ImageCallback, this, Hints);
  }
}
