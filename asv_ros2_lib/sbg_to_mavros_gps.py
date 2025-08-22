import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgGpsPos, SbgGpsVel, SbgUtcTime, SbgGpsHdt
from mavros_msgs.msg import GPSINPUT
from std_msgs.msg import Header
import datetime
import math

class SbgToMavrosGps(Node):
    def __init__(self):
        super().__init__('sbg_to_mavros_gps')
        
        self.latest_pos = None
        self.latest_vel = None
        self.latest_utc_time = None
        self.latest_hdt = None

        self.sbg_pos_sub = self.create_subscription(
            SbgGpsPos,
            '/sbg/gps_pos',
            self.pos_callback,
            10)
        
        self.sbg_vel_sub = self.create_subscription(
            SbgGpsVel,
            '/sbg/gps_vel',
            self.vel_callback,
            10)
        
        self.sbg_utc_time_sub = self.create_subscription(
            SbgUtcTime,
            '/sbg/utc_time',
            self.utc_time_callback,
            10)

        self.sbg_hdt_sub = self.create_subscription(
            SbgGpsHdt,
            '/sbg/gps_hdt',
            self.hdt_callback,
            10)
        
        self.mavros_gps_pub = self.create_publisher(
            GPSINPUT,
            '/mavros/gps_input/gps_input',
            10)
        #self.timer = self.create_timer(0.2, self.try_publish)

    def pos_callback(self, pos_msg: SbgGpsPos):
        self.latest_pos = pos_msg
        self.try_publish()

    def vel_callback(self, vel_msg: SbgGpsVel):
        self.latest_vel = vel_msg
        self.try_publish()

    def utc_time_callback(self, time_msg: SbgUtcTime):
        self.latest_utc_time = time_msg
        self.try_publish()

    def hdt_callback(self, hdt_msg: SbgGpsHdt):
        self.latest_hdt = hdt_msg
        self.try_publish()

    def map_fix_type(self, sbg_fix_type: int) -> int:
        if sbg_fix_type in (0, 1):
            return 1  # GPS_FIX_TYPE_NO_FIX
        elif sbg_fix_type == 2:
            return 3  # GPS_FIX_TYPE_3D_FIX (or 2 for 2D_FIX if you want)
        elif sbg_fix_type in (3, 4, 5):
            return 4  # GPS_FIX_TYPE_DGPS
        elif sbg_fix_type == 6:
            return 5  # GPS_FIX_TYPE_RTK_FLOATR
        elif sbg_fix_type == 7:
            return 6  # GPS_FIX_TYPE_RTK_FIXEDR
        elif sbg_fix_type in (8, 9):
            return 8  # GPS_FIX_TYPE_PPP
        elif sbg_fix_type == 10:    
            return 7  # GPS_FIX_TYPE_STATIC
        else:
            return 0  # GPS_FIX_TYPE_NO_GPS (unknown or invalid)

    def utc_to_gps_week(self, year: int, month: int, day: int) -> int:
        gps_epoch = datetime.datetime(1980, 1, 6)
        current = datetime.datetime(year, month, day)
        delta = current - gps_epoch
        return delta.days // 7
    

    def try_publish(self):
        
        if self.latest_pos is None or self.latest_vel is None or self.latest_utc_time is None or self.latest_hdt is None:
            self.get_logger().warn('Waiting for complete GPS data...')
            return
        
        gps_msg = GPSINPUT()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps'

        gps_msg.fix_type = self.map_fix_type(self.latest_pos.status.type)
        gps_msg.gps_id = 0 # After keeping 0, we can only see the changes in pixhawk (mavros and mavproxy) when kept 1, did not work
        gps_msg.ignore_flags = 0 # Not sure about this either
        gps_msg.time_week_ms = self.latest_pos.gps_tow
        gps_msg.time_week = self.utc_to_gps_week(self.latest_utc_time.year, 
                                                  self.latest_utc_time.month, self.latest_utc_time.day)
        
        gps_msg.lat = int(self.latest_pos.latitude * 1e7)
        gps_msg.lon = int(self.latest_pos.longitude * 1e7)
        gps_msg.alt = float(self.latest_pos.altitude)
        
        # DOP (if available)
        gps_msg.hdop = float(getattr(self.latest_pos, 'hdop', 0.0))
        gps_msg.vdop = float(getattr(self.latest_pos, 'vdop', 0.0))
        
        gps_msg.vn = self.latest_vel.velocity.x
        gps_msg.ve = self.latest_vel.velocity.y
        gps_msg.vd = self.latest_vel.velocity.z
        
        gps_msg.speed_accuracy = math.sqrt(self.latest_vel.velocity_accuracy.x**2 + self.latest_vel.velocity_accuracy.y**2 + self.latest_vel.velocity_accuracy.z**2)
        gps_msg.horiz_accuracy = math.sqrt(self.latest_pos.position_accuracy.x**2 + self.latest_pos.position_accuracy.y**2)
        gps_msg.vert_accuracy = self.latest_pos.position_accuracy.z
        
        gps_msg.satellites_visible = self.latest_pos.num_sv_used
        gps_msg.yaw = int(round(self.latest_hdt.true_heading*100))  #unit: centi-degrees
        self.mavros_gps_pub.publish(gps_msg)
        self.get_logger().info('Publishing MAVROS topic from SBG data...')

def main(args=None):
    rclpy.init(args=args)
    node = SbgToMavrosGps()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
