#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
guided_set_speed_yaw.py: (Copter Only)

This example shows how to move/direct Copter and send commands in GUIDED mode using DroneKit Python.

Example documentation: http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
"""

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import optparse
import socket
import struct
import sys
import argparse  

# constants
## DronAssistantStatus for STR package
STR_STATUS = 6

## PackageDirection
PKG_DIRECTION = 1

# exception classes

# interface functions
def build_package(drone_id, lat, lon, alt, heading, speed):
    """Build the STR package.

    Returns the packed package.
    """
    length = 48
    format = "HbbIQffffffffH"

    return struct.pack(format,
                       length,
                       STR_STATUS,
                       PKG_DIRECTION,
                       drone_id,
                       int(time.time() * 1000000000),
                       lat,
                       lon,
                       alt,
                       heading,
                       speed,
                       0.0, # temperature
                       0.0, # humidity
                       0.0, # pressure
                       1000) # battery remaining


# classes

# internal functions & classes
class Options(object):
    """Handle command line options.
    """
    def __init__(self):
        """Create the options.
        """
        self.parser = optparse.OptionParser(
            usage = "usage: %prog --host=--lat=latitude --lon=logitude --alt=altitude --heading=heading --speed=speed"
        )

        self.parser.add_option(
            '-i', '--host', action = 'store',
            dest = 'host', default="dronweb.dronsystems-dev.com",
            help = 'Host/IP of the package API host.')

        self.parser.add_option(
            '-p', '--port', action = 'store',
            dest = 'port', default = 21000,
            help = 'Port the package API is listening on.')

        self.parser.add_option(
            '-d', '--id', action = 'store',
            dest = 'id',
            help = 'Drone ID making the report.')

        self.parser.add_option(
            '-t', '--lat', action = 'store',
            dest = 'latitude',
            help = 'Latitude to report.')
        self.parser.add_option(
            '-n', '--lon', action = 'store',
            dest = 'longitude',
            help = 'Longitude to report.')
        self.parser.add_option(
            '-a', '--altitude', action = 'store',
            dest = 'altitude',
            help = 'Altitude to report.')
        self.parser.add_option(
            '-e', '--heading', action = 'store',
            dest = 'heading',
            help = 'Heading to report.')
        self.parser.add_option(
            '-s', '--speed', action = 'store',
            dest = 'speed',
            help = 'Speed to report.')

    def get(self):
        """Get the processed command line options.
        """
        (options, args) = self.parser.parse_args()

        if len(args):
            print("Other arguments!")
            sys.exit(1)

        options.port = int(options.port)

        options.drone_id = int(options.id)

        options.latitude = float(options.latitude)
        options.latitude = float(options.latitude)
        options.longitude = float(options.longitude)
        options.altitude = float(options.altitude)
        options.heading = float(options.heading)
        options.speed = float(options.speed)

        return options

vehicle = connect('udp:10.0.2.15:14553',baud=57600,wait_ready=True)

while 1<100:
  drone_id = 1
  location = vehicle.location.global_relative_frame
  latitude = float(location.lat)
  longitude = float(location.lon)
  altitude = float(location.alt)
  heading = float(vehicle.heading)
  speed = float(vehicle.groundspeed)


  package = build_package(drone_id,
                                latitude, longitude, altitude,
                                heading, speed)

  print package
  # package = build_package(options.drone_id,
  #                               options.latitude, options.longitude, options.altitude,
  #                               options.heading, options.speed)

  send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  # send_socket.sendto(package, ("dronweb.dronsystems-dev.com", 21000))
  send_socket.sendto(package, ('109.228.50.56', 21000))
  time.sleep(10)

  pass



print " Global Location: %s" % vehicle.location.global_frame
print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print " Local Location: %s" % vehicle.location.local_frame
print " Attitude: %s" % vehicle.attitude
print " Velocity: %s" % vehicle.velocity
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " EKF OK?: %s" % vehicle.ekf_ok
print " Groundspeed: %s" % vehicle.groundspeed    # settable

