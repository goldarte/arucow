import math
import rospy
from clever import srv
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import tf2_ros
import time

rospy.init_node('cow_despasturer')

# create proxy service
navigate = rospy.ServiceProxy('/navigate', srv.Navigate)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
landing = rospy.ServiceProxy('/land', Trigger)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10.0)
aruco_frame = 'aruco_0'

# copter parameters

speed = 0.5
z = 1.2
frame = 'map'
yaw_rate = math.radians(20)

# target parameters

target_x = 0.0
target_y = 0.0
target_z = 0.0
target_yaw = 0.0
target_timestamp = rospy.Time()
target_frame = frame

target_x_old = 0.0
target_y_old = 0.0
target_z_old = 0.0
target_yaw_old = 0.0

#functions

def get_distance(x1, y1, z1, x2, y2, z2):
	return math.sqrt((x1-x2)**2 + (y1-y2)**2 +(z1-z2)**2)

def height_adjust (zp, sp = 1, tolerance = 0.2):
	start = get_telemetry()
	print(navigate(z=zp, speed=sp, yaw=float('nan'), frame_id='body', auto_arm=True))
	while True:
		telem = get_telemetry()
		delta = abs(abs(telem.z - start.z)-zp)
		#print delta
		if delta < tolerance:
			break
		rospy.sleep(0.1)			

def flight_to_point(xp, yp, zp, yaw, sp, frame_id, tolerance = 0.2):
	print navigate(frame_id=frame_id, x=xp, y=yp, z=zp, speed=sp, yaw = yaw)
	while True:
		telem = get_telemetry(frame_id=frame_id)
		if get_distance(xp, yp, zp, telem.x, telem.y, telem.z) < tolerance:
			break
		rospy.sleep(0.1)

def pose_callback(data):
    global target_x, target_y, target_z, target_timestamp, target_yaw
    target_x = data.pose.position.x
    target_y = data.pose.position.y
    target_z = data.pose.position.z
    r, p, target_yaw = euler_from_quaternion([data.pose.orientation.x,
                                        data.pose.orientation.y,
                                        data.pose.orientation.z,
                                        data.pose.orientation.w])
    target_timestamp = data.header.stamp
    target_frame = data.header.frame_id
    #print target_x, target_y, target_z, target_timestamp, target_yaw


# flight program
rospy.Subscriber("/destination_pose", PoseStamped, pose_callback)

height_adjust(z, speed)

delta = 0.0000001
cow_detected = False

while not rospy.is_shutdown():

    telem = get_telemetry(frame_id='map')
    current_yaw = telem.yaw
    yaw_to_target = math.atan2(target_y-telem.y, target_x-telem.x)
    yaw_rate_signed = math.copysign(yaw_rate, math.pi - abs(abs(yaw_to_target - current_yaw) - math.pi))
    print yaw_rate_signed

    navigate(x=0, y=0, z=0, yaw=float('nan'), yaw_rate=yaw_rate, frame_id='body')

    while (rospy.Time.now() - target_timestamp).to_sec() > 0.5 or not cow_detected:
        if abs(target_x-target_x_old)>delta and abs(target_y-target_y_old)>delta and abs(target_z-target_z_old)>delta and abs(target_yaw-target_yaw_old)>delta:
            print('The cow is detected')
            cow_detected = True
        #print(rospy.Time.now(), target_timestamp)
        print("Searching cow...")
        rate.sleep()

    set_position(frame_id='body')

    if cow_detected:  

        time.sleep(1.)

        #print(rospy.Time.now(), target_timestamp)

        if (rospy.Time.now() - target_timestamp).to_sec() < 0.5:
            if abs(target_x-target_x_old)>delta and abs(target_y-target_y_old)>delta and abs(target_z-target_z_old)>delta and abs(target_yaw-target_yaw_old)>delta:
                print('The cow is in x={}, y={}, z={}'.format(target_x, target_y, target_z))
                target_x_old = target_x
                target_y_old = target_y
                target_z_old = target_z
                target_yaw_old = target_yaw
                flight_to_point(xp=target_x,yp=target_y,zp=z,yaw=target_yaw,sp=speed,frame_id=target_frame)
        
        cow_detected = False

    #flight_to_point(xp=target_x,yp=target_y,zp=target_z,yaw=target_yaw,sp=speed,frame_id=target_frame)
