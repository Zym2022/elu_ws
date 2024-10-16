#!/home/zju/anaconda3/envs/yolov8D3/bin/python

import rospy
import ctypes
import cv2
from scipy.spatial.transform import Rotation
from time import sleep
import socket
import threading

from cpt_dtc_convert import Mapping2DImageToDepthMap

class Py2Cpp(ctypes.Structure):
    _fields_ = [
        ('template_list_path', ctypes.c_char_p),
        
        ('px', ctypes.c_float),
        ('py', ctypes.c_float),
        ('pz', ctypes.c_float),
        ('qx', ctypes.c_float),
        ('qy', ctypes.c_float),
        ('qz', ctypes.c_float),
        ('qw', ctypes.c_float),
    ]

class Var2Py(ctypes.Structure):
    _fields_ = [
        ('best_fitness_score', ctypes.c_float),

        ('px', ctypes.c_float),
        ('py', ctypes.c_float),
        ('pz', ctypes.c_float),
        ('qx', ctypes.c_float),
        ('qy', ctypes.c_float),
        ('qz', ctypes.c_float),
        ('qw', ctypes.c_float),
    ]

pub_list = [0, 0, 0, 0, 0, 0, 0, 0]
rec_list = [0, 0.48331189155578613, -0.0859934613108635, 0.3883483111858368, -0.0303883, -0.9994133, 0.0010912, -0.015764]
state = -2
recieved = 0

def udp_server(s):
    global pub_list
    global state
    global recieved
    while True:
        if state == pub_list[0] and recieved != 0:
            sleep(0.5)
            continue

        if recieved == 0:
            recieved = 1 if len(rec_list) != 0 else 0

        s.sendto((str(pub_list[0]) + ' ' + str(pub_list[1])+ ' ' + str(pub_list[2])+ ' ' + str(pub_list[3])+ ' ' + str(pub_list[4])+ ' ' + str(pub_list[5])+ ' ' + str(pub_list[6])+ ' ' + str(pub_list[7])) .encode('utf-8'), udp_addr_server)
        print("sending: ", str(pub_list[0]) + ' ' + str(pub_list[1])+ ' ' + str(pub_list[2])+ ' ' + str(pub_list[3])+ ' ' + str(pub_list[4])+ ' ' + str(pub_list[5])+ ' ' + str(pub_list[6])+ ' ' + str(pub_list[7]))
        state = pub_list[0]
        sleep(1)

def udp_client(s):
    global recieved
    global rec_list
    while True:
        recv_data = s.recvfrom(1024)
        # print("[From %s:%d]:%s" % (recv_data[1][0], recv_data[1][1], recv_data[0].decode("utf-8")))
        rec_list =  [float(x) for x in recv_data[0].split()]
        sleep(1.0)


if __name__ == '__main__':

    # load shared library from cpp
    # so = ctypes.CDLL("/home/zju/Mech-Mind-Samples/yolomech/build/libtemplate_alignment.so")
    so = ctypes.CDLL("/home/zju/yolomech_ws/devel/lib/libtemplate_alignment.so")
    py2cpp = Py2Cpp(b"/home/zju/realsense_ws/template_pcd/template_list.txt", \
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    so.set_py2cpp.argtypes = [ctypes.POINTER(Py2Cpp)]
    so.set_py2cpp(ctypes.byref(py2cpp))

    # udp
    udp_addr_server = ('10.3.0.96', 9999)
    udp_socket_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    

    udp_addr_client = ('', 7777)
    udp_socket_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket_client.bind(udp_addr_client)
    
    
    thread_list = []
    t1 = threading.Thread(target=udp_client, args=(udp_socket_client, ))
    thread_list.append(t1)
    t2 = threading.Thread(target=udp_server, args=(udp_socket_server, ))
    thread_list.append(t2)
 
    for t in thread_list:
        t.setDaemon(True)
        t.start()
    

    while True:
        # when the size of the hole in picture is large enough
        # if camera_xyz[-1] < 250:
        pub_list[0] = 0

        sleep(2)

        # check if the robot has stopped
        if rec_list[0] != -1:
            cv2.waitKey(500)
            continue

        # euler = [rec_list[4], rec_list[5], rec_list[6]]
        # quat = Rotation.from_euler('ZYX', euler, degrees=False).as_quat()

        # send end pose to cpp
        # print(rec_list[1], rec_list[2], rec_list[3])
        py2cpp = Py2Cpp(b"/home/zju/realsense_ws/template_pcd/template_list.txt", \
                        rec_list[1], rec_list[2], rec_list[3], \
                        rec_list[4], rec_list[5], rec_list[6], rec_list[7])
                        # quat[3], quat[0], quat[1], quat[2])
        so.set_py2cpp(ctypes.byref(py2cpp))

        # save point cloud from depth image as pcd file
        point_cloud_file = "detect_pc.pcd"
        a = Mapping2DImageToDepthMap(point_cloud_file)
        a.main()
        print("Aligning the target to template ...")
        so.TemplateAlign(b"/home/zju/Mech-Mind-Samples/yolomech/python/detect_pc.pcd")
        break
    
    # get results from cpp
    so.get_var2py.restype = ctypes.POINTER(Var2Py)
    var2py_ptr = so.get_var2py()
    var2py = var2py_ptr.contents
    
    pub_list = [2, round(var2py.px, 4), round(var2py.py, 4), round(var2py.pz, 4), \
                round(var2py.qx, 4), round(var2py.qy, 4), round(var2py.qz, 4), round(var2py.qw, 4)]
    # pub_list = [2, round(var2py.px, 4), round(var2py.py, 4), round(var2py.pz, 4), \
    #             0.        , 0.76604444, 0.        , 0.64278761]

    # continue to send messages to robot
    while True:
        continue