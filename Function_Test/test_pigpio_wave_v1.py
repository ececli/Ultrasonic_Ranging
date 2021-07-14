import time
import pigpio
 

bit_time = 50 # the bit time in the unit of microseconds
file_name = 'wave_file'
gpios=[]
pulses=[]
 
with open(file_name, 'r') as f:
   edges=[]
   num_bits = 0
   for line in f:
      if len(line) > 1 and line[0] != '#':
         fields = line.split()
         gpio = int(fields[0])
         gpios.append(gpio)
         levels = int(fields[1], 2)
         edges.append([1<<gpio, levels])
         if len(fields[1]) > num_bits:
            num_bits = len(fields[1])
 
for bit in range(num_bits-1, -1, -1):
   on = 0
   off = 0
   for e in edges:
      if (1<<bit) & e[1]:
         on |= e[0]
      else:
         off |= e[0]
   pulses.append((on, off, bit_time))
 
pi = pigpio.pi()
if not pi.connected:
    exit()
 
for g in gpios:
    pi.set_mode(g, pigpio.OUTPUT)
 
wf = []
for p in pulses:
    wf.append(pigpio.pulse(p[0], p[1], p[2]))
 
pi.wave_clear()
pi.wave_add_generic(wf)
 
wid = pi.wave_create()
if wid >= 0:
    pi.wave_send_once(wid)

    pi.wave_delete(wid)
 
pi.stop()