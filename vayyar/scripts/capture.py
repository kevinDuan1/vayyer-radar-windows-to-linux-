#! C:\Users\maps\AppData\Local\Programs\Python\Python37 python.exe
# -*- coding: utf-8 -*-

import numpy as np
from struct import unpack_from
from std_msgs.msg import String
import json
from websocket import create_connection
import time
import os
import rospy
# from topic_test.msg import Radar #message for for radar
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

output_dir='D:\\data\\points'
foldername=str(time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime()))
# output_= input(f'Waht is the save folder in {output_dir}?\n')
output_=foldername

os.mkdir(output_dir + str(output_) )
os.mkdir(output_dir + str(output_) + '\\mmwave')

#thermal_dir=output_dir + str(output_) + '\\thermal\\'
mmwave_dir=output_dir + str(output_) + '\\mmwave\\'

# ---------------------------------Vayyar setup-----------------------------------
DTYPES = {
        0: np.int8,
        1: np.uint8,
        2: np.int16,
        3: np.uint16,
        4: np.int32,
        5: np.uint32,
        6: np.float32,
        7: np.float64,
    }
ASCII_RS = '\u001e'
ASCII_US = '\u001f'

def to_message(buffer):
    """ parse MatNet messages from JSON / Vayyar internal binary format """
    if isinstance(buffer, str):
        return json.loads(buffer)
    seek = 0
    fields_len = unpack_from('i', buffer, seek + 4)[0]
    fields_split = unpack_from(str(fields_len) + 's', buffer, seek + 8)[0].decode('utf8').split(ASCII_RS)
    msg = {'ID': fields_split[0], 'Payload': dict.fromkeys(fields_split[1].split(ASCII_US))}
    seek += 8 + fields_len
    for key in msg['Payload']:
        seek += np.int32().nbytes
        dtype = DTYPES[np.frombuffer(buffer, np.int32, 1, seek).item()]
        seek += np.int32().nbytes
        ndims = np.frombuffer(buffer, np.int32, 1, seek).item()
        seek += np.int32().nbytes
        dims = np.frombuffer(buffer, np.int32, ndims, seek)
        seek += ndims * np.int32().nbytes
        data = np.frombuffer(buffer, dtype, np.prod(dims), seek)
        seek += np.prod(dims) * dtype().nbytes
        msg['Payload'][key] = data.reshape(dims) if ndims else data.item()
    return msg


def setup_mmwave():
    # """ connect to server and echoing messages """
    listener = create_connection("ws://127.0.0.1:1234/")
    # retrieve current configuration
    listener.send(json.dumps({
        'Type': 'COMMAND',
        'ID': 'SET_PARAMS',
        'Payload': {
        # 'Cfg.MonitoredRoomDims': [-2, 2, 0.5, 5, 0, 3],# Room dimensions
        'Cfg.MonitoredRoomDims': [-3, 3, 0.5, 6, 0, 3],# Room dimensions
        'Cfg.Common.sensorOrientation.mountPlane':'xz',#xy - ceiling, xz - wall
        'Cfg.Common.sensorOrientation.transVec(3)': [1.5],# Height of sensor
        'Cfg.imgProcessing.substractionMode':7, #6-AMTI,7-MTI,2-Initial,0-NS.
        'Cfg.TargetProperties.MaxPersonsInArena': 2.0,
        'Cfg.TargetProperties.PersonRadius': 0.6,
        'MPR.save_dir': mmwave_dir, # Saved records directory
        'MPR.read_from_file': 0.0, # 1 – To play records
        'MPR.save_to_file': 1.0, # 1 – To save raw data
        'MPR.save_image_to_file': 0.0, # 1 – To save image data
        'Cfg.OutputData.save_to_file': 0.0, # 1 – To save log data
    }
    }))

    # set outputs for each frame
    listener.send(json.dumps({
        'Type': 'COMMAND',
        'ID': 'SET_OUTPUTS',
        'Payload': {
            # 'binary_outputs': ['I', 'Q', 'pairs', 'freqs', 'LocationMatrix', 'RawPoints','rawImage_XYZ', 'rawImage_XY', 'rawImage_XZ', 'rawImage_YZ', 'NumOfPeople'], 
            'binary_outputs': ['I', 'Q', 'pairs', 'freqs', 'RawPoints'], 
            # 'json_outputs': ['rawImage_XY']
            'json_outputs': []
        }
    }))

    # Start the engine - only if WebGUI isn't present
    listener.send(json.dumps({
        'Type': 'COMMAND', 
        'ID': 'START',  
        'Payload': {}
    }))

    # request for binary and json data.
    listener.send(json.dumps({'Type': 'QUERY', 'ID': 'BINARY_DATA'}))
    listener.send(json.dumps({'Type': 'QUERY', 'ID': 'JSON_DATA'}))
    print("Running! Waiting for messages...")
    return listener


# ----------------------------thermal setup-------------------------------------------
"""
device_index = 2
cap = cv2.VideoCapture(device_index+cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
"""
# -------------------------------------------------------------------------------------------


def main():
    rospy.init_node("vayyar_radar")
    # pub = rospy.Publisher("point_clouds", String, queue_size=10)
    # rate = rospy.Rate(15) #publish frequency
    listener = setup_mmwave()
    # msg = Radar()
    pub = rospy.Publisher("radar_output", PointCloud2, queue_size=10)
    field = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('I', 12, PointField.FLOAT32, 1),
             ]
    hz = 10
    rate = rospy.Rate(hz)
    header = Header()
    header.frame_id = "map"
    while not rospy.is_shutdown():
        t0 = time.time()
        buffer = listener.recv()
        header.stamp = rospy.Time.now()
        current_time=int(time.time()*(10**9))
        data = to_message(buffer)
        dic = dict()
        if data['ID'] == 'BINARY_DATA':
            for key, item in enumerate(data['Payload']):
                dic[item] = data['Payload'][item]
            listener.send(json.dumps({'Type': 'QUERY', 'ID': 'BINARY_DATA'}))
            msg = point_cloud2.create_cloud(header, field, dic['RawPoints'])
            pub.publish(msg)
            rospy.loginfo(f'sending at {hz} Hz')
            rate.sleep()
            # print(dic.keys())
            # print('the data is', message.data)
            
            #io.savemat(mmwave_dir+str(current_time)+'.mat', dic)

        t1 = time.time()
        # print('capture time : {} fps: {}'.format(t1-t0, 1/(t1-t0)))
        # rate.sleep()
    listener.close()


if __name__ == '__main__':
    main()



