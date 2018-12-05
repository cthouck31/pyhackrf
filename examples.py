from pyhackrf import pyhackrf
import time
import ctypes

hackrf = pyhackrf.HackRf()

if hackrf.is_open == False:
    hackrf.setup()
    hackrf.set_freq(93.1e6)
    hackrf.set_sample_rate(1e6)
    hackrf.set_amp_enable(False)
    hackrf.set_lna_gain(16)
    hackrf.set_vga_gain(20)    

def callback_fun(hackrf_transfer):
    array_type = (ctypes.c_byte*hackrf_transfer.contents.valid_length)
    values = ctypes.cast(hackrf_transfer.contents.buffer, ctypes.POINTER(array_type)).contents
    #iq data here
    iq = hackrf.packed_bytes_to_iq(values)
    print len(iq)
    return 0

hackrf.start_rx_mode(callback_fun)

while 1:
    time.sleep(1)
