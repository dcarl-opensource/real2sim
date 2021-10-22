from zzz_common.kinematics import get_frenet_state
from zzz_common.geometry import dense_polyline2d
from zzz_driver_msgs.msg import RigidBodyStateStamped, FrenetSerretState2D
import numpy as np
import sys

# load reference line from map as ref_line.txt
raw_ref = np.loadtxt("/".join(__file__.split("/")[0:-1]) + '/' + 'ref_line.txt', delimiter=',')
ref_path = dense_polyline2d(raw_ref, 1)
# zzz bug here
ref_tangent = np.zeros(len(ref_path))

class Object:
    obj_id = 0

    def __init__(self, id, classid, k, l):
    #def __init__(self, id, classid, ref_path, ref_tangent):
        self.id = id
        self.carla_id = self.obj_id
        Object.obj_id += 1
        self.classid = classid
        self.timestamp = []
        self.x_pos = []
        self.y_pos = []
        self.yaw = []
        self.s = []
        self.d = []
        self.v = []
        self.ref_path = ref_path
        self.ref_tangent = ref_tangent


    def add_info(self, time, x_pos, y_pos, yaw, velocity):
        # fill the perception gap(data between 2 message is more than 0.05)
        # if len(self.timestamp) > 0 and time - self.timestamp[-1] > 0.05:
        #     for i in range(1, int((time - self.timestamp[-1])/0.05)):
        #         self.timestamp.append(self.timestamp[-1] + 0.05 * i)
        #         self.x_pos.append(x_pos)
        #         self.y_pos.append(y_pos)
        #         self.yaw.append(yaw)
        self.timestamp.append(time)
        self.x_pos.append(x_pos)
        self.y_pos.append(y_pos)
        self.yaw.append(yaw)
        self.v.append(velocity)

    def updatesd(self, msg):

        fstate = get_frenet_state(msg, self.ref_path, self.ref_tangent)

        self.s.append(fstate.s)
        self.d.append(fstate.d)

    def __add__(self, other):
        self.timestamp.extend(other.timestamp)
        self.x_pos.extend(other.x_pos)
        self.y_pos.extend(other.y_pos)
        self.yaw.extend(other.yaw)
        self.s.extend(other.s)
        self.d.extend(other.d)

        return self

    def interpolate_infomation(self):
        for i in range(1, len(self.timestamp) - 1, 2):
            self.s[i] = (self.s[i+1] - self.s[i-1])/2 + self.s[i-1]
            self.d[i] = (self.d[i+1] - self.d[i-1])/2 + self.d[i-1]

    def dump(self):
        output_data = []
        # fill the perception gap(data between 2 message is more than 0.05)
        for i in range(len(self.timestamp)):
            output_data.append([self.timestamp[i], self.v[i], self.s[i], self.d[i], self.x_pos[i], self.y_pos[i]])
        np.savetxt("/".join(__file__.split("/")[0:-2]) + "/data/raw/" + str(self.id), output_data, fmt="%.2f,%.8f,%.8f,%.8f,%.8f,%.8f", delimiter=',', newline='\n')
        self.check_data()
        return self.id, self.classid
    
    def check_data(self):
        for i in range(1, len(self.timestamp)):
            # assert 0.039 <= self.timestamp[i] - self.timestamp[i-1] <= 0.061, "EV {} Time at {} is not sequential, time gap is {}".format(self.id, i, self.timestamp[i] - self.timestamp[i-1])
            if not 0.039 <= self.timestamp[i] - self.timestamp[i-1] <= 0.061:
                print("self id", self.id, "error at : ", i)

class EgoState:
    #def __init__(self, ref_path, ref_tangent):
    def __init__(self, k, l):
        self.timestamp = []
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.s = []
        self.d = []
        self.s_trend = []
        self.d_trend = []
        self.ref_path = ref_path
        self.ref_tangent = ref_tangent

    def updatesd(self, msg):
        fstate = get_frenet_state(msg.state, self.ref_path, self.ref_tangent)
        self.s.append(fstate.s)
        self.d.append(fstate.d)

    def dump(self):
        output_data = []
        for i in range(len(self.timestamp)):
            output_data.append([self.timestamp[i], self.v[i], self.yaw[i], self.s[i], self.d[i], self.x[i], self.y[i]])
        np.savetxt("/".join(__file__.split("/")[0:-2]) + "/data/raw/" + "ego_vehicle", output_data, fmt="%.2f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f", delimiter=',', newline='\n')
        self.check_data()

    def check_data(self):
        for i in range(1, len(self.timestamp)):
            # assert 0.039 <= self.timestamp[i] - self.timestamp[i-1] <= 0.061, "EV {} Time at {} is not sequential, time gap is {}".format(self.id, i, self.timestamp[i] - self.timestamp[i-1])
            if not 0.039 <= self.timestamp[i] - self.timestamp[i-1] <= 0.061:
                print("error at : ", i)