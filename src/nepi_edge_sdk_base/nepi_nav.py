#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI navigation utility functions include
# 1) NavPose request functions
# 2) NavPose conversion utility functions

import os
import rospy
import numpy as np
import math
import time
import sys
import tf
import random
from pygeodesy.ellipsoidalKarney import LatLon


# try and import geoid height calculation module and databases
GEOID_DATABASE_FILE='/opt/nepi/databases/geoids/egm2008-2_5.pgm' # Ignored if PyGeodesy module or Geoids Database is not available
FALLBACK_GEOID_HEIGHT_M = 0.0 # Ignored if if PyGeodesy module or Geoids Database are available
file_loaded = False
if os.path.exists(GEOID_DATABASE_FILE):
  try:
    import pygeodesy
    GEOID_DATABASE_FILE=GEOID_DATABASE_FILE
    rospy.loginfo("NEPI_NAV: Loading Geoids Database from: " + GEOID_DATABASE_FILE)
    ginterpolator = pygeodesy.GeoidKarney(GEOID_DATABASE_FILE)
    file_loaded = True
  except rospy.ServiceException as e:
    rospy.loginfo("NEPI_NAV: Geoids database failed to import: %s"%e)
if file_loaded is False:
  def ginterpolator(single_position):
    return FALLBACK_GEOID_HEIGHT_M

from std_msgs.msg import Bool, String, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.msg import NavPose
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest



#######################
# NavPose Request Functions

def get_navpose_response():
  try:
    get_navpose_service = rospy.ServiceProxy(self.NAVPOSE_SERVICE_NAME, NavPoseQuery)
    navpose_response = get_navpose_service(NavPoseQueryRequest())
  except rospy.ServiceException as e:
    rospy.loginfo("NEPI_NAV: Service call failed: %s"%e)
    time.sleep(1)
    rospy.signal_shutdown("Service call failed")
  return navpose_response

def get_navpose_heading_deg(navpose_response):
  # Set current heading in degrees
  current_heading_deg = navpose_response.nav_pose.heading.heading
  return current_heading_deg

def get_navpose_orientation_enu_degs(navpose_response):
  # Set current orientation vector (roll, pitch, yaw) in degrees enu frame
  pose_enu_o = navpose_response.nav_pose.odom.pose.pose.orientation
  xyzw_enu_o = list([pose_enu_o.x,pose_enu_o.y,pose_enu_o.z,pose_enu_o.w])
  rpy_enu_d = convert_quat2rpy(xyzw_enu_o)
  current_orientation_enu_degs = [rpy_enu_d[0],rpy_enu_d[1],rpy_enu_d[2]]
  return current_orientation_enu_degs

def get_navpose_orientation_ned_degs(navpose_response):
  # Set current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
  pose_enu_o = navpose_response.nav_pose.odom.pose.pose.orientation
  xyzw_enu_o = list([pose_enu_o.x,pose_enu_o.y,pose_enu_o.z,pose_enu_o.w])
  rpy_enu_d = convert_quat2rpy(xyzw_enu_o)
  yaw_ned_d = convert_yaw_enu2ned(rpy_enu_d[2])
  rpy_ned_d = [rpy_enu_d[0],rpy_enu_d[1],yaw_ned_d]
  current_orientation_ned_degs = [rpy_ned_d[0],rpy_ned_d[1],rpy_ned_d[2]]
  return current_orientation_ned_degs

def get_navpose_position_enu_m(navpose_response):
  # Set current position vector (x, y, z) in meters enu frame
  pose_enu_p = navpose_response.nav_pose.odom.pose.pose.position
  current_position_enu_m = [pose_enu_p.x, pose_enu_p.y, pose_enu_p.z]
  return current_position_enu_m

def get_navpose_position_ned_m(navpose_response):
  # Set current position vector (x, y, z) in meters ned frame
  pose_enu_p = navpose_response.nav_pose.odom.pose.pose.position
  current_position_ned_m = [pose_enu_p.y, pose_enu_p.x, -pose_enu_p.z]
  return current_position_ned_m


def get_navpose_location_wgs84_geo(navpose_response): 
  # Set current location vector (lat, long, alt) in geopoint data with AMSL height
  fix_wgs84 = navpose_response.nav_pose.fix
  current_location_wgs84_geo =  [fix_wgs84.latitude,fix_wgs84.longitude,fix_wgs84.altitude]
  return current_location_wgs84_geo


def get_navpose_location_amsl_geo(navpose_response):  
  # Set current location vector (lat, long, alt) in geopoint data with AMSL height
  geoid_height = get_navpose_geoid_height(navpose_response)
  fix_wgs84 = navpose_response.nav_pose.fix
  current_location_amsl_geo =  [fix_wgs84.latitude,fix_wgs84.longitude,(fix_wgs84.altitude + geoid_height)]
  return current_location_amsl_geo

def get_navpose_geoid_height(navpose_response):
  # Set current location vector (lat, long, alt) in geopoint data with WGS84 height
  fix_wgs84 = navpose_response.nav_pose.fix
  single_position=LatLon(fix_wgs84.latitude,fix_wgs84.longitude)
  geoid_height = ginterpolator(single_position)
  current_geoid_height =  geoid_height
  return current_geoid_height
  


#######################
# NavPose Conversion Functions

def get_navpose_geoid_height_at_geopoint(geopoint):
  single_position=LatLon(geopoint.latitude,geopoint.longitude)
  geoid_height = ginterpolator(single_position)
  current_geoid_height =  geoid_height
  return current_geoid_height
  
def convert_amsl_to_wgs84(geopoint_in):
  geoid_height = get_navpose_geoid_height_at_geopoint(geopoint_in)
  geopoint_out = GeoPoint()
  geopoint_out.latitude = geopoint_in.latitude
  geopoint_out.longitude = geopoint_in.longitude
  geopoint_out.altitude = geopoint_in.altitude - geoid_height
  return geopoint_out
  
def convert_wgs84_to_amsl(geopoint_in):
  geoid_height = get_navpose_geoid_height_at_geopoint(geopoint_in)
  geopoint_out = GeoPoint()
  geopoint_out.latitude = geopoint_in.latitude
  geopoint_out.longitude = geopoint_in.longitude
  geopoint_out.altitude = geopoint_in.altitude + geoid_height
  return geopoint_out

### Function to Convert Quaternion Attitude to Roll, Pitch, Yaw Degrees
def convert_quat2rpy(xyzw_attitude):
  rpy_attitude_rad = tf.transformations.euler_from_quaternion(xyzw_attitude)
  rpy_attitude_deg = np.array(rpy_attitude_rad) * 180/math.pi
  roll_deg = rpy_attitude_deg[0] 
  pitch_deg = rpy_attitude_deg[1] 
  yaw_deg = rpy_attitude_deg[2]
  return rpy_attitude_deg

### Function to Convert Roll, Pitch, Yaw Degrees to Quaternion Attitude
def convert_rpy2quat(rpy_attitude_deg):
  roll_rad = math.radians(rpy_attitude_deg[0])
  pitch_rad = math.radians(rpy_attitude_deg[1]) 
  yaw_rad = math.radians(rpy_attitude_deg[2])
  #xyzw_attitude = tf.transformations.quaternion_from_euler(roll_rad,pitch_rad,yaw_rad,axes="sxyz")
  xyzw_attitude = tf.transformations.quaternion_from_euler(pitch_rad, yaw_rad, roll_rad, axes="ryzx")
  return xyzw_attitude

### Function to Convert Yaw NED to Yaw ENU
def convert_yaw_ned2enu(yaw_ned_deg):
  yaw_enu_deg = 90-yaw_ned_deg
  if yaw_enu_deg < -180:
    yaw_enu_deg = 360 + yaw_enu_deg
  elif yaw_enu_deg > 180:
    yaw_enu_deg = yaw_enu_deg - 360
  return yaw_enu_deg

### Function to Convert Yaw ENU to Yaw NED
def convert_yaw_enu2ned(yaw_enu_deg):
  yaw_ned_deg =  90-yaw_enu_deg
  if yaw_ned_deg < -180:
    yaw_ned_deg = 360 + yaw_ned_deg
  elif yaw_ned_deg > 180:
    yaw_ned_deg = yaw_ned_deg - 360
  return yaw_ned_deg


### Function to Convert Yaw from Body to ENU Frame
def convert_yaw_body2enu(yaw_body_deg,cur_heading_deg):
  cur_yaw_enu_deg = - cur_heading_deg
  if cur_yaw_enu_deg > 180: # Convert to +-180
    cur_yaw_enu_deg = cur_yaw_enu_deg - 360
  yaw_enu_deg =  cur_yaw_enu_deg + yaw_body_deg
  return yaw_enu_deg

### Function to Convert Point from Body to ENU Frame
def convert_point_body2enu(point_body_m,yaw_enu_deg):
  point_bearing_enu_deg = yaw_enu_deg + math.degrees(math.atan2(point_body_m[1],point_body_m[0]))
  point_bearing_enu_rad = math.radians(point_bearing_enu_deg)
  xy_body_m = math.sqrt(point_body_m[0]**2 + point_body_m[1]**2)
  x_enu_m = xy_body_m * math.cos(point_bearing_enu_rad)
  y_enu_m = xy_body_m * math.sin(point_bearing_enu_rad)
  point_enu_m = [x_enu_m,y_enu_m,point_body_m[2]]
  return point_enu_m
  
### Function to Convert from ENU Frame to Point from Body
def convert_point_enu2body(point_enu_m,yaw_enu_deg):
  point_bearing_body_deg = yaw_enu_deg + math.degrees(math.atan2(point_enu_m[1],point_enu_m[0]))
  point_bearing_body_rad = math.radians(point_bearing_body_deg)
  xy_enu_m = math.sqrt(point_enu_m[0]**2 + point_enu_m[1]**2)
  x_body_m = xy_enu_m * math.cos(point_bearing_body_rad)
  y_body_m = xy_enu_m * math.sin(point_bearing_body_rad)
  point_body_m = [x_body_m,y_body_m,point_enu_m[2]]
  return point_body_m


### Function to Convert Yaw from Body to NED Frame
def convert_yaw_body2ned(yaw_body_deg,cur_heading_deg):
  cur_yaw_ned_deg = cur_heading_deg
  if cur_yaw_ned_deg > 180: # Convert to +-180
    cur_yaw_ned_deg = cur_yaw_ned_deg - 360
  yaw_ned_deg =  cur_yaw_ned_deg + yaw_body_deg
  return yaw_ned_deg

### Function to Convert Point from Body to NED Frame
def convert_point_body2ned(point_body_m,yaw_ned_deg):
  point_bearing_ned_deg = yaw_ned_deg - math.degrees(math.atan2(point_body_m[1],point_body_m[0]))
  point_bearing_ned_rad = math.radians(point_bearing_ned_deg)
  xy_body_m = math.sqrt(point_body_m[0]**2 + point_body_m[1]**2)
  x_ned_m = xy_body_m * math.cos(point_bearing_ned_rad)
  y_ned_m = xy_body_m * math.sin(point_bearing_ned_rad)
  point_ned_m = [x_ned_m,y_ned_m,-point_body_m[2]]
  return point_ned_m

  
### Function to get new latlong at body relative point
def get_geopoint_at_body_point(cur_geopoint_geo, cur_bearing_enu_deg, point_body_m):
  # cur_geopoint_geo is list [Lat,Long,Alt] with Alt passed through
  earth_radius_km = 6378.137
  earth_circ_m = np.float64(2 * math.pi * earth_radius_km*1000)
  # Calculate bearing in NED frame
  point_body_m[1] = - point_body_m[1] # adjust for y = right frame from body y left frame.
  cur_bearing_ned_deg = convert_yaw_enu2ned(cur_bearing_enu_deg)
  point_bearing_ned_deg = cur_bearing_ned_deg + math.degrees(math.atan2(point_body_m[1],point_body_m[0]))
  point_bearing_ned_rad = math.radians(point_bearing_ned_deg)
  # Calculate distances NED frame
  delta_body_m = math.sqrt(point_body_m[0]**2+point_body_m[1]**2)
  delta_x_ned_m = delta_body_m * math.cos(point_bearing_ned_rad) # north:pos,south:neg
  delta_y_ned_m = delta_body_m * math.sin(point_bearing_ned_rad) # east:pos,west:neg
  # Calculate New Lat Position
  cur_lat = cur_geopoint_geo.latitude
  m_per_lat = np.float64(earth_circ_m/360)
  delta_lat = delta_x_ned_m / m_per_lat
  new_lat = cur_lat + delta_lat
  # Calculate New Long Position
  cur_long = cur_geopoint_geo.longitude
  m_per_long = m_per_lat * math.cos(math.radians(cur_lat)) 
  delta_long = delta_y_ned_m / m_per_long
  new_long = cur_long + delta_long
  #rospy.loginfo(point_body_m)
  #rospy.loginfo(cur_bearing_ned_deg)
  #rospy.loginfo(point_bearing_ned_deg)
  #rospy.loginfo(delta_x_ned_m)
  #rospy.loginfo(delta_y_ned_m)
  #rospy.loginfo(cur_geopoint_geo.latitude)
  #rospy.loginfo(new_lat)
  #rospy.loginfo(cur_geopoint_geo.longitude)
  #rospy.loginfo(new_long)


  # Return New Geo Position
  new_geopoint_geo=GeoPoint()
  new_geopoint_geo.latitude = new_lat
  new_geopoint_geo.longitude = new_long
  new_geopoint_geo.altitude = cur_geopoint_geo.altitude
  return  new_geopoint_geo


  ### Function to get new latlong at body relative point
def get_geopoint_at_enu_point(cur_geopoint_geo, point_enu_m):
  # cur_geopoint_geo is list [Lat,Long,Alt] with Alt passed through
  earth_radius_km = 6378.137
  earth_circ_m = np.float64(2 * math.pi * earth_radius_km*1000)
  # Calculate point in NED frame
  delta_x_ned_m = point_enu_m[1] # north:pos,south:neg
  delta_y_ned_m = point_enu_m[0] # east:pos,west:neg
  # Calculate New Lat Position
  cur_lat = cur_geopoint_geo.latitude
  m_per_lat = np.float64(earth_circ_m/360)
  delta_lat = delta_x_ned_m / m_per_lat
  new_lat = cur_lat + delta_lat
  # Calculate New Long Position
  cur_long = cur_geopoint_geo.longitude
  m_per_long = m_per_lat * math.cos(math.radians(cur_lat)) 
  delta_long = delta_y_ned_m / m_per_long
  new_long = cur_long + delta_long
  #rospy.loginfo(point_enu_m)
  #rospy.loginfo(delta_x_ned_m)
  #rospy.loginfo(delta_y_ned_m)
  #rospy.loginfo(cur_geopoint_geo.latitude)
  #rospy.loginfo(new_lat)
  #rospy.loginfo(cur_geopoint_geo.longitude)
  #rospy.loginfo(new_long)


  # Return New Geo Position
  new_geopoint_geo=GeoPoint()
  new_geopoint_geo.latitude = new_lat
  new_geopoint_geo.longitude = new_long
  new_geopoint_geo.altitude = cur_geopoint_geo.altitude + point_enu_m[2]
  return  new_geopoint_geo


### Function to get distance between two geo latlong locations
def distance_geopoints(geopoint1,geopoint2):
  lat1 = math.radians(geopoint1[0])
  lat2 = math.radians(geopoint2[0])
  lon1 = math.radians(geopoint1[1])
  lon2 = math.radians(geopoint2[1])
  # Haversine formula 
  dlon = (lon2 - lon1)
  dlat = (lat2 - lat1)
  a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
  c = 2 * math.asin(math.sqrt(a))

  # Radius of earth in kilometers. Use 3956 for miles
  r = 6371
  xy_m=c*r*1000
  alt_m = abs(geopoint1[2]-geopoint2[2])
  distance_m = math.sqrt(alt_m**2 + xy_m**2)
  #rospy.loginfo("NEPI_NAV: Moving : " + "%.2f" % (xy_m) + " meters in xy plane")
  #rospy.loginfo("NEPI_NAV: Moving : " + "%.2f" % (alt_m) + " meters in z axis")
  #rospy.loginfo("NEPI_NAV: Moving : " + "%.2f" % (distance_m) + " total meters")
 
  # calculate the result
  return(distance_m)
  
### Function to get point between two geo latlong locations
def point_from_geopoints(geopoint1,geopoint2,current_heading):
  lat1 = math.radians(geopoint1[0])
  lat2 = math.radians(geopoint2[0])
  lon1 = math.radians(geopoint1[1])
  lon2 = math.radians(geopoint2[1])
  # Haversine formula 
  dlon = (lon2 - lon1)
  dlat = (lat2 - lat1)
  a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
  c = 2 * math.asin(math.sqrt(a))

  # Radius of earth in kilometers. Use 3956 for miles
  r = 6371
  xy_m=c*r*1000
  delta_x_ned_m = xy_m * math.cos(point_bearing_ned_rad) # north:pos,south:neg
  delta_y_ned_m = xy_m * math.sin(point_bearing_ned_rad) # east:pos,west:neg
  delta_alt_m = abs(geopoint1[2]-geopoint2[2])

  return delta_x_ned_m,delta_y_ned_m,delta_alt_m


