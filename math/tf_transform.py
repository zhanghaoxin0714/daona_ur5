import math3d as m3d


def calcu_tcp_pose_rotVec(tcp_pose_m3d, tool_delta_pos_m3d):
    """
    intput type m3d.Transform() params
    calulate tcp ending pose by tcp current pose and delta pose
    """

    t = tcp_pose_m3d * tool_delta_pos_m3d
    return t.pose_vector


def RotVec_2_transformMatrix(rotvec):
    """
    a rotvet to matrix 4x4
    """
    rotm = m3d.Transform(rotvec)
    return rotm


def calcu_tcp_mv_in_tcp_coord(tcp_start_pose_m3d, tcp_end_pose_m3d):
    """
      calcu tcp_mv,by end_pose and start_pose, return mv_rotVec in start pose coordination
    """
    t = tcp_start_pose_m3d.inverse * tcp_end_pose_m3d
    return t.pose_vector


if __name__ == "__main__":
    rotvec = [1, 1, 1, -1, -2, 3]
    tool_delta = RotVec_2_transformMatrix(rotvec);
    print(rotvec)

    tcp_pose = m3d.Transform([1, 2, 3, 1, 1, 1])
    print(calcu_tcp_pose_rotVec(tcp_pose, tool_delta))

    tcp_pose_end = m3d.Transform([1, 1, 1, 1, 1, 1])
    print('mv_pose', calcu_tcp_mv_in_tcp_coord(tcp_pose, tcp_pose_end))
