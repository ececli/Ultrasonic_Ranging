import pyaudio
import time
import zmq
import sys
import numpy as np
import twoWayRangingLib_v2 as func
import configparser

# setup zmq pub
context = zmq.Context()
socket = context.socket(zmq.PUB)
# socket.bind("tcp://*:%s" % port)
socket.bind("ipc:///dev/shm/test")


confFile = "UR_pyConfig_v2.conf"
cp = configparser.ConfigParser()
cp.read(confFile)

CHANNELS = cp.getint("MIC","CHANNELS")
RATE = cp.getint("MIC","RATE")
CHUNK = cp.getint("MIC", "CHUNK")
FORMAT_SET = cp.get("MIC","FORMAT")
if FORMAT_SET == "Int":
    FORMAT = pyaudio.paInt32
elif FORMAT_SET == "Float":
    FORMAT = pyaudio.paFloat32
else:
    FORMAT = pyaudio.paInt32
    print("Unsupport Format. Have been Changed to Int32.")


CHUNK = 512 



COUNT = 0
prev1 = 0
threshold = 1.5*CHUNK/RATE




def callback(in_data, frame_count, time_info, status):
    global COUNT, prev1
    global socket
    COUNT += 1
    time_info['status'] = status
    time_info['count'] = COUNT
    time_info['mic'] = in_data
    socket.send_pyobj(time_info)
    time_info['mic']=None

    mic = np.frombuffer(in_data, dtype=np.int32)
    mic = mic>>14
    print(COUNT, mic.mean())

    dt = time_info['input_buffer_adc_time']-prev1
    # TS.append(dt)
    prev1 = time_info['input_buffer_adc_time']
    # print(dt)
    if (status>0):
        print("Abnormal status: ", COUNT, status, time_info)
    if dt>threshold:
        print("dt > threshold: ", COUNT, dt, threshold, time_info)
    buf = bytes(in_data)
    return (buf, pyaudio.paContinue)

    # return (buf, pyaudio.paAbort)

def record():
    p = pyaudio.PyAudio()
    DEV_INDEX = func.findDeviceIndex(p)
    if DEV_INDEX == -1:
        print("Error: No Mic Found!")
        exit(1)
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    stream_callback=callback,
                    input_device_index = DEV_INDEX,
                    frames_per_buffer=CHUNK)

    print("Start recording")
    stream.start_stream()
    while stream.is_active():
        time.sleep(1)



    stream.stop_stream()
    stream.close()
    p.terminate()



if __name__ == '__main__':
    print('#' * 80)
    print('Press Ctrl+C to stop the recording')
    print('#' * 80)
    record()
