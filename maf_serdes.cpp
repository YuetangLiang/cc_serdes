#include "hps.h"
#include "maf_interface/maf_vehicle_info.h"
#include "maf_interface/maf_remote_pilot.h"
#include "maf_interface/maf_driver_behavior.h"
#include "maf_interface/maf_planning.h"
#include "maf_interface/maf_endpoint.h"
#include "maf_interface/maf_odd_detection.h"
#include "maf_interface/maf_mla_localization.h"
#include "maf_interface/maf_worldmodel.h"

void hps_test() {
    std::vector<double> input;
    const int n_elems = 1 << 25;
    input.resize(n_elems);
    for (int i = 0; i < n_elems; i++) input[i] = i;

    std::string str = hps::to_string(input);
}

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
