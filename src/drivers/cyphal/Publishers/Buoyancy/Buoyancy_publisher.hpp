#pragma once

#include "../Publisher.hpp"
#include <uORB/topics/buoyancy_control.h>
#include <robosub/BuoyancyControl_1_0.h>
#include <uORB/uORB.h>            // Required for the ORB_ID macro
#include <uORB/Subscription.hpp>  // Required for the uORB::Subscription class

class BuoyancyPublisher : public UavcanPublisher
{
public:
    // Constructor: Passes names to the base class to create "UCAN1_PUB_BUOY_ID" automatically
    BuoyancyPublisher(CanardHandle &handle, UavcanParamManager &pmgr) :
        UavcanPublisher(handle, pmgr, "uavcan.pub", "buoyancy", 0)
    {}

    // Just the declaration. The logic is in the .cpp file.
    void update() override;

private:
    uORB::Subscription _sub{ORB_ID(buoyancy_control)};
};
