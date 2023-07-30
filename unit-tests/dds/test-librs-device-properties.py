# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#test:donotrun:!dds

from rspy import log, test
log.nested = 'C  '

import d435i
import d405
import d455
import dds

import pyrealsense2 as rs
if log.is_debug_on():
    rs.log_to_console( rs.log_severity.debug )
from time import sleep

context = rs.context( { 'dds': { 'domain': 123, 'participant': 'device-properties-client' }} )
only_sw_devices = int(rs.product_line.sw_only) | int(rs.product_line.any_intel)


import os.path
cwd = os.path.dirname(os.path.realpath(__file__))
remote_script = os.path.join( cwd, 'device-broadcaster.py' )
with test.remote( remote_script, nested_indent="  S" ) as remote:
    remote.wait_until_ready()
    #
    #############################################################################################
    #
    test.start( "Test D435i" )
    try:
        remote.run( 'instance = broadcast_device( d435i, d435i.device_info )' )
        n_devs = 0
        for dev in dds.wait_for_devices( context, only_sw_devices ):
            n_devs += 1
        test.check_equal( n_devs, 1 )
        test.check_equal( dev.get_info( rs.camera_info.name ), d435i.device_info.name )
        test.check_equal( dev.get_info( rs.camera_info.serial_number ), d435i.device_info.serial )
        test.check_equal( dev.get_info( rs.camera_info.physical_port ), d435i.device_info.topic_root )
        sensors = {sensor.get_info( rs.camera_info.name ) : sensor for sensor in dev.query_sensors()}
        test.check_equal( len(sensors), 3 )
        if test.check( 'Stereo Module' in sensors ):
            sensor = sensors.get('Stereo Module')
            test.check_equal( len(sensor.get_stream_profiles()), 104 ) # As measured running rs-sensor-control example
        if test.check( 'RGB Camera' in sensors ):
            sensor = sensors['RGB Camera']
            test.check_equal( len(sensor.get_stream_profiles()), 64 ) # As measured running rs-sensor-control example
        if test.check( 'Motion Module' in sensors ):
            sensor = sensors['Motion Module']
            test.check_equal( len(sensor.get_stream_profiles()), 2 ) # Only the Gyro profiles
        remote.run( 'close_server( instance )' )
    except:
        test.unexpected_exception()
    dev = None
    test.finish()
    #
    #############################################################################################
    #
    test.start( "Test D405" )
    try:
        remote.run( 'instance = broadcast_device( d405, d405.device_info )' )
        n_devs = 0
        for dev in dds.wait_for_devices( context, only_sw_devices ):
            n_devs += 1
        test.check_equal( n_devs, 1 )
        test.check_equal( dev.get_info( rs.camera_info.name ), d405.device_info.name )
        test.check_equal( dev.get_info( rs.camera_info.serial_number ), d405.device_info.serial )
        test.check_equal( dev.get_info( rs.camera_info.physical_port ), d405.device_info.topic_root )
        sensors = {sensor.get_info( rs.camera_info.name ) : sensor for sensor in dev.query_sensors()}
        test.check_equal( len(sensors), 1 )
        if test.check( 'Stereo Module' in sensors ):
            sensor = sensors.get('Stereo Module')
            test.check_equal( len(sensor.get_stream_profiles()), 146 ) # As measured running rs-sensor-control example
        remote.run( 'close_server( instance )' )
    except:
        test.unexpected_exception()
    dev = None
    test.finish()
    #
    #############################################################################################
    #
    test.start( "Test D455" )
    try:
        remote.run( 'instance = broadcast_device( d455, d455.device_info )' )
        n_devs = 0
        for dev in dds.wait_for_devices( context, only_sw_devices ):
            n_devs += 1
        test.check_equal( n_devs, 1 )
        test.check_equal( dev.get_info( rs.camera_info.name ), d455.device_info.name )
        test.check_equal( dev.get_info( rs.camera_info.serial_number ), d455.device_info.serial )
        test.check_equal( dev.get_info( rs.camera_info.physical_port ), d455.device_info.topic_root )
        sensors = {sensor.get_info( rs.camera_info.name ) : sensor for sensor in dev.query_sensors()}
        test.check_equal( len(sensors), 3 )
        if test.check( 'Stereo Module' in sensors ):
            sensor = sensors.get('Stereo Module')
            test.check_equal( len(sensor.get_stream_profiles()), 100 ) # As measured running rs-sensor-control example
        if test.check( 'RGB Camera' in sensors ):
            sensor = sensors['RGB Camera']
            test.check_equal( len(sensor.get_stream_profiles()), 62 ) # As measured running rs-sensor-control example
        if test.check( 'Motion Module' in sensors ):
            sensor = sensors['Motion Module']
            test.check_equal( len(sensor.get_stream_profiles()), 2 ) # Only the Gyro profiles
        remote.run( 'close_server( instance )' )
    except:
        test.unexpected_exception()
    dev = None
    test.finish()
    #
    #############################################################################################

context = None
test.print_results_and_exit()
