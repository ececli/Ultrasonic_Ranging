import zmq
import pyaudio
import numpy as np
import time


# pub mic data out.

FORMAT = pyaudio.paInt32
RATE = 64000
CHANNELS = 1
CHUNK=64

dt = np.dtype([('counter', 'i4'), ('status', 'i4'), ('timestamp', 'f8')])
header = np.rec.array(np.zeros(1, dtype=dt))


COUNT = 0
prev1 = 0
threshold = 1.5*CHUNK/RATE

def findDeviceIndex(p):
    DEV_INDEX = -1
    for ii in range(0, p.get_device_count()):
        dev = p.get_device_info_by_index(ii)
        if 'snd_rpi_i2s' in dev['name']:
            DEV_INDEX = dev['index']
    return DEV_INDEX


def callback(in_data, frame_count, time_info, status):
    global COUNT, prev1, header
    global socket
    COUNT += 1
    buf = bytes(in_data)
    header.counter  = COUNT
    header.status = status
    header.timestamp = time_info['input_buffer_adc_time']
    raw_header = header.tobytes()
    socket.send(raw_header+buf)

    diffTS = time_info['input_buffer_adc_time']-prev1
    prev1 = time_info['input_buffer_adc_time']
    if (status>0):
        print("Abnormal status: ", COUNT, status, time_info)
    if diffTS>threshold:
        print("Diff TS > threshold: ", COUNT, diffTS, threshold, time_info)
    return (buf, pyaudio.paContinue)



def record():

    p = pyaudio.PyAudio()
    DEV_INDEX = findDeviceIndex(p)
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
    try:
        while stream.is_active():
            time.sleep(1)
    except KeyboardInterrupt:
        stream.stop_stream()
        stream.close()
        p.terminate()
        print("Terminated by User")




if __name__ == '__main__':


    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    # socket.bind("tcp://*:%s" % port)
    socket.bind("ipc:///dev/shm/mic_data")
    time.sleep(1)
    record()
