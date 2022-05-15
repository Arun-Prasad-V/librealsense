// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#include <algorithm>
#include <iostream>
#include "lrs-device-watcher.h"

using namespace tools;

lrs_device_watcher::lrs_device_watcher()
    : _rs_device_list( std::make_shared< std::vector< rs2::device > >() )
    , _ctx( "{"
            "\"dds-discovery\" : false"
            "}" )
{
}


lrs_device_watcher::~lrs_device_watcher() 
{
}

void lrs_device_watcher::run( std::function< void( rs2::device ) > add_device_cb,
                              std::function< void( rs2::device ) > remove_device_cb )
{
    notify_connected_devices_on_wake_up( add_device_cb );

    // Register to LRS device change callback.
    // For each device added, call callback and store it in a list
    // For each device removed, call callback and remove it from the list
    std::weak_ptr< std::vector< rs2::device > > weak_rs_device_list ( _rs_device_list );
    _ctx.set_devices_changed_callback(
        [this, add_device_cb, remove_device_cb, weak_rs_device_list]( rs2::event_information & info ) {
            auto strong_rs_device_list = weak_rs_device_list.lock();
            if( strong_rs_device_list )
            {
                std::vector<rs2::device> devices_to_remove;
                for( auto && rs_device : *strong_rs_device_list )
                {
                    if( info.was_removed( rs_device ) )
                    {
                        devices_to_remove.push_back(rs_device);
                    }
                }

                for( auto device_to_remove : devices_to_remove )
                {
                    remove_device_cb( device_to_remove );
                    std::cout << "Device '" << device_to_remove.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "' - removed" << std::endl;
                    strong_rs_device_list->erase( std::remove_if( strong_rs_device_list->begin(),
                                                           strong_rs_device_list->end(),
                                                           [&]( const rs2::device & dev ) {
                                                               return dev == device_to_remove;
                                                           } ),
                                           strong_rs_device_list->end() );
                }
 

                for( auto && rs_device : info.get_new_devices() )
                {
                    add_device_cb( rs_device );
                    strong_rs_device_list->push_back(rs_device);
                    std::cout << "Device '" << rs_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "' - detected" << std::endl;
                }
            }
        } );
}

void tools::lrs_device_watcher::notify_connected_devices_on_wake_up(
    std::function< void( rs2::device ) > add_device_cb )
{
    auto connected_dev_list = _ctx.query_devices();
    for( auto connected_dev : connected_dev_list )
    {
        add_device_cb( connected_dev );
        _rs_device_list->push_back(connected_dev);
        std::cout << "Device '" << connected_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "' - detected" << std::endl;
    }
}
