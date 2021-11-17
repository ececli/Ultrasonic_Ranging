import sys
import zmq
import time
import numpy as np
import configparser
from scipy import signal
import RPi.GPIO as GPIO
import pyaudio
from cffi import FFI

# version 5 is the debugging version which write data to files.


ffibuilder = FFI()

ffibuilder.set_source("_goertzel",
   r""" // passed to the real C compiler,
        // contains implementation of things declared in cdef()
        #include <math.h>

        // We can also define custom wrappers or other functions
        // here (this is an example only):
        // double test_cos(double x) {
        //    return cos(x);
        // }
        #define RING_SIZE 512
        
        void sg_block(int32_t *x, int offset, double* z, double *Pxx, double c) {
            int block_size = 64;
            int window_size = 256; 
            // Do index math just once... 
            int32_t *curr = x+offset;
            int32_t *past = x + ((offset - window_size) & (RING_SIZE-1));
            double z0;
            double z1 = z[0];
            double z2 = z[1];
            
            for (int idx=0; idx<block_size; idx++) {
                // z0 = x[offset + idx] - x[((offset + idx - window_size)&(512-1))] + c*z1 - z2;
                z0 = *curr++ - *past++ + c*z1 - z2;
                z2 = z1;
                z1 = z0;
                Pxx[idx] = z2*z2 + z1*z1 - c * z1*z2;
            }
            z[0] = z1;
            z[1] = z2;
        }
    """,
    libraries=['m'])   # or a list of libraries to link with
    # (more arguments like setup.py's Extension class:
    # include_dirs=[..], extra_objects=[..], and so on)



ffibuilder.cdef("""
    // declarations that are shared between Python and C
    // double test_cos(double x);
    void sg_block(int32_t *x, int offset, double* z, double *Pxx, double c);
""")


ffibuilder.compile(verbose=True)


import _goertzel



# Allocated all memory needed to run algorithm
z = np.zeros(2)
ring = np.zeros(512, np.int32)
result = np.zeros(1280000)
Pxx = np.zeros(64)
# Point to the beginning of the ring buffer
write_addr = 0
# Calculate parameters for the goertzel algorithm
k=100; window_size=256
w = 2*np.pi*k/window_size;
c = 2*np.cos(w);

# Get pointers to the addresses in memory for the output and results
ptr_ring = _goertzel.ffi.cast("int32_t *", ring.ctypes.data)
ptr_z = _goertzel.ffi.cast("double *", z.ctypes.data)
ptr_Pxx = _goertzel.ffi.cast("double *", Pxx.ctypes.data)



fd = open('rec3_64.dat', 'rb')
raw = fd.read()
fd.close()


start = time.time()
for count in range(int(len(raw)/4/64)):
    block = np.frombuffer(raw[(count*256):((count+1)*256)], dtype=np.int32)
    block = block >> 14
    if (len(block)==0):
        break
    ring[(write_addr):(write_addr+64)] = block
    _goertzel.lib.sg_block(ptr_ring, write_addr, ptr_z, ptr_Pxx, c)
    write_addr += 64
    write_addr &= (512-1)
    result[count*64:(count+1)*64] = Pxx
print(time.time()-start)
