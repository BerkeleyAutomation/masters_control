from core import RigidTransform
from geometry_msgs.msg import Pose, Point, Quaternion

def ros_pose_to_T(ros_pose, from_frame, to_frame):
    translation = [ros_pose.position.x, ros_pose.position.y, ros_pose.position.z]
    rotation = [ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z]

    T = RigidTransform(rotation=rotation, translation=translation, from_frame=from_frame, to_frame=to_frame)
    return T

def T_to_ros_pose(T):
    point = Point(T.translation[0], T.translation[1], T.translation[2])
    T_quat = T.quaternion
    quaternion = Quaternion(T_quat[1], T_quat[2], T_quat[3], T_quat[0])
    ros_pose = Pose(point, quaternion)
    return ros_pose

def str_str_service_wrapper(service):
    def wrapper(req, data=""):
        return service(req, data).res
    return wrapper
