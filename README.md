header ONLY

# sample
```cpp
#include "hps.h"
#include "maf_interface/maf_vehicle_info.h"
#include "maf_interface/maf_remote_pilot.h"
#include "maf_interface/maf_driver_behavior.h"
#include "maf_interface/maf_planning.h"
#include "maf_interface/maf_endpoint.h"
#include "maf_interface/maf_odd_detection.h"
#include "maf_interface/maf_mla_localization.h"
#include "maf_interface/maf_worldmodel.h"

template<typename T>
std::string ser(T *maf) {
    return hps::to_string(*maf);
}

template<typename T>
T des(std::string s) {
    return hps::from_string<T>(s);
}

void test_serdes() {
    maf_worldmodel::PredictionResult maf_src, maf_dst;
    std::string serialized = ser<maf_worldmodel::PredictionResult>(&maf_src);
    maf_dst = des<maf_worldmodel::PredictionResult>(serialized);
}

```
## des sample
```cpp
int timeout = 1000;
DataCenter::getInstance()->pop(ap_topic, ap_data, timeout);
auto maf = hps::from_string<maf_worldmodel::PredictionResult>(ap_data);
ros_cpp_struct_convert::to_ros(maf, msg);
pubrs_[ros_topic].publish(msg);
```

## ser sample
```cpp

                    maf.header.frame_id = std::string(value.header.frame_id.c_str());
                    maf.meta.frame_id = std::string(value.meta.frame_id.c_str());
                    maf.object_prediction_data.resize(value.object_prediction_data.size());
                    for(uint64_t i = 0; i < maf.object_prediction_data.size(); i++) {
                        auto si = &value.object_prediction_data[i];
                        auto di = &maf.object_prediction_data[i];

                        di->extra_json = std::string(si->extra_json.c_str());
                        di->trajectories.resize(si->trajectories.size());
                        for(uint64_t j = 0; j < di->trajectories.size(); j++) {
                            auto sj = &si->trajectories[j];
                            auto dj = &di->trajectories[j];

                            dj->confidence           = sj->confidence;
                            dj->prediction_interval  = sj->prediction_interval;
                            dj->intention.value      = sj->intention;
                            dj->extra_json           = std::string(sj->extra_json.c_str());
                            dj->trajectory_points.resize(sj->trajectory_points.size());
                            for(uint64_t k = 0; k < dj->trajectory_points.size(); k++) {
                                auto sk = &sj->trajectory_points[k];
                                auto dk = &dj->trajectory_points[k];

                                dk->position.x = sk->position.x;
                                dk->position.y = sk->position.y;
                                dk->yaw        = sk->yaw;
                                dk->theta      = sk->theta;
                                dk->velocity   = sk->velocity;
                                dk->confidence = sk->confidence;
                                dk->covariance_xy.x00 = sk->covariance_xy.x00;
                                dk->covariance_xy.x01 = sk->covariance_xy.x01;
                                dk->covariance_xy.x10 = sk->covariance_xy.x10;
                                dk->covariance_xy.x11 = sk->covariance_xy.x11;
                            }
                        }
                    }

std::string serialized = hps::to_string(maf);
publish(serialized, "/msd/prediction/prediction_result");
```
