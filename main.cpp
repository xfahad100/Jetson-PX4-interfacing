#include <iostream>
#include <chrono>
#include "mavlink_interface.h"

int main()
{
        //std::unique_ptr<MavlinkInterface> _mavlink_interface;
        //_mavlink_interface->Load();
    MavlinkInterface MAVnew;
    MAVnew.Load();
    return  0;
}
