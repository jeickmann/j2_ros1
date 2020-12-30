import rospy

import serial
import struct

from math import cos, sin, pi

import tf2_ros
import transformations
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

class J2Controller():

    def __init__(self):
        self.readReady = False
        self.msgBuf = bytearray()
        self.lticks = 0
        self.rticks = 0
        self.lPower = 0.0
        self.rPower = 0.0
        self.frameMillis = 0
        self.lastlTicks = 0
        self.lastrTicks = 0
        self.posX = 0.0
        self.posY = 0.0
        self.heading = 0.0
        self.dx = 0.0
        self.dr = 0.0
        self.lastFrameMillis = 0
        self.posInitialized = False

        rospy.init_node('j2_controller')

        self.subscription = rospy.Subscriber(
            'cmd_vel',
            Twist,
            self.listener_callback)

        self.subscription  # prevent unused variable warning

        self.odomPublisher = rospy.Publisher('odom', Odometry, queue_size=10)
        self.lticksPublisher = rospy.Publisher('lticks', Int32, queue_size=10)
        self.rticksPublisher = rospy.Publisher('rticks', Int32, queue_size=10)

        self.ser = serial.Serial('/dev/ttyUSB0', 115200)  # open serial port1
        #timer_period = 0.01  # seconds
        #pubTimer_period = 0.05
        #self.timer = self.create_timer(timer_period, self.checkSerial)
        #self.pubTimer = self.create_timer(pubTimer_period, self.pubOdom)
        self.rate = rospy.Rate(10) # 10hz

    def listener_callback(self, msg):
        cmd = struct.pack("ff", msg.linear.x, -msg.angular.z)
        self.ser.write(cmd)

    def checkSerial(self):
        msgLength = 16

        while(self.ser.in_waiting > 0):
            if( not self.readReady):
                #spin until startMarker
                startQ = self.ser.read(1)
                if(startQ == b"\xFE"):
                    self.readReady = True
                    self.msgBuf = bytearray()
            
            if(self.readReady):
                self.msgBuf += self.ser.read(1)
                if(len(self.msgBuf) == msgLength):
                    (self.lticks, self.rticks, self.lPower, self.rPower, self.frameMillis) = struct.unpack("<hhffi", self.msgBuf)
                    self.updateOdom()
                    self.readReady = False

    def updateOdom(self):
        TICKS_PER_METER = 1176.0
        WHEEL_SEPARATION = 0.170

        dT = (self.frameMillis - self.lastFrameMillis) / 1000.0

        lVel = (self.lticks - self.lastlTicks) / TICKS_PER_METER
        rVel = (self.rticks - self.lastrTicks) / TICKS_PER_METER

        self.dx = lVel / dT
        self.dr = rVel / dT

        self.lastlTicks = self.lticks
        self.lastrTicks = self.rticks
        self.lastFrameMillis = self.frameMillis

        if(self.posInitialized):
            linearVel  = (lVel + rVel) * 0.5
            angularVel = (rVel - lVel) / WHEEL_SEPARATION

            direction = self.heading + angularVel

            self.posX += linearVel * cos(direction)
            self.posY += linearVel * sin(direction)
            self.heading += angularVel
        else:
            self.posInitialized = True

    def pubOdom(self):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.posX
        t.transform.translation.y = self.posY
        t.transform.translation.z = 0.0
        #q = transformations.quaternion_from_euler(0, 0, self.heading)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sin( self.heading / 2 )
        t.transform.rotation.w = cos( self.heading / 2 )

        br.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.posX
        odom.pose.pose.position.y = self.posY
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sin( self.heading / 2 )
        odom.pose.pose.orientation.w = cos( self.heading / 2 )
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        
        self.odomPublisher.publish(odom)

        lTicksMsg = Int32()
        lTicksMsg.data = self.lticks
        rTicksMsg = Int32()
        rTicksMsg.data = self.rticks

        self.lticksPublisher.publish(lTicksMsg)
        self.rticksPublisher.publish(rTicksMsg)
    

def main(args=None):
    j2controller = J2Controller()
    while not rospy.is_shutdown():
        j2controller.checkSerial()
        j2controller.pubOdom()
        j2controller.rate.sleep()

if __name__ == '__main__':
    main()