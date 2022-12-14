import rclpy
from rclpy.node import Node 
from ros_openpose.msg import AltMarkerArray, Frame
from std_msgs.msg import Float64
from happymimi_msgs.srv import SetFloat, SetFloatResponse
from happymimi_recognition_msgs.srv import PositionEstimator, PositionEstimatorRequest











class HeightEstimation(Node):
    def __init__(self):
        super().__init__('HeightEstimation')
        self.srv = self.create_service(SetFloat, '/person_feature/height_estimation', self.main)
        self.sub = self.create_subscription(Frame, '/frame', self.openPoseCB)
        self.position_estimate = self.create_client(PositioEstimator, '/detect/depth')
        self.pub = self.create_publisher(Float64, '/servo/head', 10)

        self.pose_res = Frame()

    def openPoseCB(self, res):
        self.pose_res = res

    def main(self, _):
        height = SetFloatResponse(data=-1)
        self.head_pub.publish(-25.0)
        rospy.sleep(2.5)

        pose = self.pose_res
        if len(pose.persons)==0: return height

        center_x = pose.persons[0].bodyParts[0].pixel.y
        center_y = pose.persons[0].bodyParts[0].pixel.x
        if center_x==0 and center_y==0:
            center_x = pose.persons[0].bodyParts[15].pixel.y
            center_y = pose.persons[0].bodyParts[15].pixel.x
            if center_x==0 and center_y==0:
                center_x = pose.persons[0].bodyParts[16].pixel.y
                center_y = pose.persons[0].bodyParts[16].pixel.x
                if center_x==0 and center_y==0:
                    return height
        if center_x<0: center_x=0
        if center_x>479: center_x=479
        if center_y<0: center_y=0
        if center_y>639: center_y=639

        self.position_estimation.wait_for_service(timeout_sec=1.0)
        p_e_req = PositionEstimatorRequest()
        p_e_req.center_x = int(center_x)
        p_e_req.center_y = int(center_y)
        p_e_res = self.position_estimate(p_e_req).point

        height.data = p_e_res.z*100 + 30
        print(p_e_res.z*100)
        return height

if __name__ == '__main__':
    rclpy.init()
    height_estimation = HeightEstimation()
    rospy.spin(height_estimation)

