from ctypes import *
import logging
import os
import numpy as np

import logging
logging.basicConfig()
logger = logging.getLogger("PyHackrf")
logger.setLevel(logging.DEBUG)

# Get path to the installed 'libhackrf.so' library.
from __init__ import LIBHACKRF_PATH, LIBHACKRF_SO

try:
    from itertools import izip
except ImportError:
    logger.debug("Could not import \'izip\' from \'itertools\', using \'zip\'.")
    izip = zip

# Get opened shared library.
libhackrf = LIBHACKRF_SO

# Enumeration helper.
def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)


# Vendor request IDs ().
HackRfVendorRequest =\
    enum(
        HACKRF_VENDOR_REQUEST_SET_TRANSCEIVER_MODE          = 1,
        HACKRF_VENDOR_REQUEST_MAX2837_WRITE                 = 2,
        HACKRF_VENDOR_REQUEST_MAX2837_READ                  = 3,
        HACKRF_VENDOR_REQUEST_SI5351C_WRITE                 = 4,
        HACKRF_VENDOR_REQUEST_SI5351C_READ                  = 5,
        HACKRF_VENDOR_REQUEST_SAMPLE_RATE_SET               = 6,
        HACKRF_VENDOR_REQUEST_BASEBAND_FILTER_BANDWIDTH_SET = 7,
        HACKRF_VENDOR_REQUEST_RFFC5071_WRITE                = 8,
        HACKRF_VENDOR_REQUEST_RFFC5071_READ                 = 9,
        HACKRF_VENDOR_REQUEST_SPIFLASH_ERASE                = 10,
        HACKRF_VENDOR_REQUEST_SPIFLASH_WRITE                = 11,
        HACKRF_VENDOR_REQUEST_SPIFLASH_READ                 = 12,
        HACKRF_VENDOR_REQUEST_CPLD_WRITE                    = 13,
        HACKRF_VENDOR_REQUEST_BOARD_ID_READ                 = 14,
        HACKRF_VENDOR_REQUEST_VERSION_STRING_READ           = 15,
        HACKRF_VENDOR_REQUEST_SET_FREQ                      = 16,
        HACKRF_VENDOR_REQUEST_AMP_ENABLE                    = 17,
        HACKRF_VENDOR_REQUEST_BOARD_PARTID_SERIALNO_READ    = 18,
        HACKRF_VENDOR_REQUEST_SET_LNA_GAIN                  = 19,
        HACKRF_VENDOR_REQUEST_SET_VGA_GAIN                  = 20,
        HACKRF_VENDOR_REQUEST_SET_TXVGA_GAIN                = 21,
        HACKRF_VENDOR_REQUEST_ANTENNA_ENABLE                = 23,
        HACKRF_VENDOR_REQUEST_SET_FREQ_EXPLICIT             = 24,
        # USB_WCID_VENDOR_REQ                                = 25,
        HACKRF_VENDOR_REQUEST_INIT_SWEEP                    = 26,
        HACKRF_VENDOR_REQUEST_OPERACAKE_GET_BOARDS          = 27,
        HACKRF_VENDOR_REQUEST_OPERACAKE_SET_PORTS           = 28,
        HACKRF_VENDOR_REQUEST_SET_HW_SYNC_MODE              = 29,
        HACKRF_VENDOR_REQUEST_RESET                         = 30,
        HACKRF_VENDOR_REQUEST_OPERACAKE_SET_RANGES          = 31,
        HACKRF_VENDOR_REQUEST_CLKOUT_ENABLE                 = 32,
        HACKRF_VENDOR_REQUEST_SPIFLASH_STATUS               = 33,
        HACKRF_VENDOR_REQUEST_SPIFLASH_CLEAR_STATUS         = 34,
        HACKRF_VENDOR_REQUEST_OPERACAKE_GPIO_TEST           = 35,
    )


HackRfConstants = \
    enum(
        LIBUSB_ENDPOINT_IN  = 0x80,
        LIBUSB_ENDPOINT_OUT = 0x00,
        HACKRF_DEVICE_OUT   = 0x40,
        HACKRF_DEVICE_IN    = 0xC0,
        HACKRF_USB_VID      = 0x1d50,
        HACKRF_USB_PID      = 0x6089
    )


HackRfError = \
    enum(
        HACKRF_SUCCESS                      = 0,
        HACKRF_TRUE                         = 1,
        HACKRF_ERROR_INVALID_PARAM          = -2,
        HACKRF_ERROR_NOT_FOUND              = -5,
        HACKRF_ERROR_BUSY                   = -6,
        HACKRF_ERROR_NO_MEM                 = -11,
        HACKRF_ERROR_LIBUSB                 = -1000,
        HACKRF_ERROR_THREAD                 = -1001,
        HACKRF_ERROR_STREAMING_THREAD_ERR   = -1002,
        HACKRF_ERROR_STREAMING_STOPPED      = -1003,
        HACKRF_ERROR_STREAMING_EXIT_CALLED  = -1004,
        HACKRF_ERROR_OTHER                  = -9999,
        # Python defaults to returning none
        HACKRF_ERROR                        = None
    )


HackRfTransceiverMode = \
    enum(
        HACKRF_TRANSCEIVER_MODE_OFF      = 0,
        HACKRF_TRANSCEIVER_MODE_RECEIVE  = 1,
        HACKRF_TRANSCEIVER_MODE_TRANSMIT = 2
    )


# 'hackrf_device' structure.
class hackrf_device(Structure):
    pass


# 'hackrf_transfer' structure.
class hackrf_transfer(Structure):
        _fields_ = \
            [
                ("hackrf_device", POINTER(hackrf_device)),
                ("buffer",        POINTER(c_byte)),
                ("buffer_length", c_int),
                ("valid_length",  c_int),
                ("rx_ctx",        c_void_p),
                ("tx_ctx",        c_void_p)
            ]


# Required data structures.
_libusb_device_handle = c_void_p
_pthread_t = c_ulong


# Define callback function pointer.
_hackrf_callback = CFUNCTYPE(c_int, POINTER(hackrf_transfer))


# Complete definition of 'hackrf_device'.
hackrf_device._fields_ = \
        [
            ("usb_device",              POINTER(_libusb_device_handle)),
            ("transfers",               POINTER(POINTER(hackrf_transfer))),
            ("callback",                _hackrf_callback),
            ("transfer_thread_started", c_int),
            ("transfer_thread",         _pthread_t),
            ("transfer_count",          c_uint32),
            ("buffer_size",             c_uint32),
            ("streaming",               c_int),
            ("rx_ctx",                  c_void_p),
            ("tx_ctx",                  c_void_p)
        ]


#####################################
# Function definitions.
#####################################

# extern ADDAPI int ADDCALL hackrf_init();
libhackrf.hackrf_init.restype = c_int
libhackrf.hackrf_init.argtypes = []

# extern ADDAPI int ADDCALL hackrf_exit();
libhackrf.hackrf_exit.restype = c_int
libhackrf.hackrf_exit.argtypes = []

# extern ADDAPI int ADDCALL hackrf_open(hackrf_device** device);
libhackrf.hackrf_open.restype = c_int
libhackrf.hackrf_open.argtypes = [POINTER(POINTER(hackrf_device))]

# extern ADDAPI int ADDCALL hackrf_close(hackrf_device* device);
libhackrf.hackrf_close.restype = c_int
libhackrf.hackrf_close.argtypes = [POINTER(hackrf_device)]

# extern ADDAPI int ADDCALL hackrf_start_rx(hackrf_device* device,
# hackrf_sample_block_cb_fn callback, void* rx_ctx);
libhackrf.hackrf_start_rx.restype = c_int
libhackrf.hackrf_start_rx.argtypes = [POINTER(hackrf_device), _hackrf_callback, c_void_p]

# extern ADDAPI int ADDCALL hackrf_stop_rx(hackrf_device* device);
libhackrf.hackrf_stop_rx.restype = c_int
libhackrf.hackrf_stop_rx.argtypes = [POINTER(hackrf_device)]

# extern ADDAPI int ADDCALL hackrf_start_tx(hackrf_device* device,
# hackrf_sample_block_cb_fn callback, void* tx_ctx);
libhackrf.hackrf_start_tx.restype = c_int
libhackrf.hackrf_start_tx.argtypes = [POINTER(hackrf_device), _hackrf_callback, c_void_p]

# extern ADDAPI int ADDCALL hackrf_stop_tx(hackrf_device* device);
libhackrf.hackrf_stop_tx.restype = c_int
libhackrf.hackrf_stop_tx.argtypes = [POINTER(hackrf_device)]

# extern ADDAPI int ADDCALL hackrf_is_streaming(hackrf_device* device);
libhackrf.hackrf_is_streaming.restype = c_int
libhackrf.hackrf_is_streaming.argtypes = [POINTER(hackrf_device)]

# extern ADDAPI int ADDCALL hackrf_max2837_read(hackrf_device* device,
# uint8_t register_number, uint16_t* value);
libhackrf.hackrf_max2837_read.restype = c_int
libhackrf.hackrf_max2837_read.argtypes = [
    POINTER(hackrf_device), c_uint8, POINTER(c_uint16)]

# extern ADDAPI int ADDCALL hackrf_max2837_write(hackrf_device* device,
# uint8_t register_number, uint16_t value);
libhackrf.hackrf_max2837_write.restype = c_int
libhackrf.hackrf_max2837_write.argtypes = [POINTER(hackrf_device), c_uint8, c_uint16]

# extern ADDAPI int ADDCALL hackrf_si5351c_read(hackrf_device* device,
# uint16_t register_number, uint16_t* value);
libhackrf.hackrf_si5351c_read.restype = c_int
libhackrf.hackrf_si5351c_read.argtypes = [
    POINTER(hackrf_device), c_uint16, POINTER(c_uint16)]

# extern ADDAPI int ADDCALL hackrf_si5351c_write(hackrf_device* device,
# uint16_t register_number, uint16_t value);
libhackrf.hackrf_si5351c_write.restype = c_int
libhackrf.hackrf_si5351c_write.argtypes = [POINTER(hackrf_device), c_uint16, c_uint16]

# extern ADDAPI int ADDCALL
# hackrf_set_baseband_filter_bandwidth(hackrf_device* device, const
# uint32_t bandwidth_hz);
libhackrf.hackrf_set_baseband_filter_bandwidth.restype = c_int
libhackrf.hackrf_set_baseband_filter_bandwidth.argtypes = [
    POINTER(hackrf_device), c_uint32]

# extern ADDAPI int ADDCALL hackrf_rffc5071_read(hackrf_device* device,
# uint8_t register_number, uint16_t* value);
libhackrf.hackrf_rffc5071_read.restype = c_int
libhackrf.hackrf_rffc5071_read.argtypes = [
    POINTER(hackrf_device), c_uint8, POINTER(c_uint16)]

# extern ADDAPI int ADDCALL hackrf_rffc5071_write(hackrf_device*
# device, uint8_t register_number, uint16_t value);
libhackrf.hackrf_rffc5071_write.restype = c_int
libhackrf.hackrf_rffc5071_write.argtypes = [POINTER(hackrf_device), c_uint8, c_uint16]

# extern ADDAPI int ADDCALL hackrf_spiflash_erase(hackrf_device*
# device);
libhackrf.hackrf_spiflash_erase.restype = c_int
libhackrf.hackrf_spiflash_erase.argtypes = [POINTER(hackrf_device)]

# extern ADDAPI int ADDCALL hackrf_spiflash_write(hackrf_device*
# device, const uint32_t address, const uint16_t length, unsigned char*
# const data);
libhackrf.hackrf_spiflash_write.restype = c_int
libhackrf.hackrf_spiflash_write.argtypes = [
    POINTER(hackrf_device), c_uint32, c_uint16, POINTER(c_ubyte)]

# extern ADDAPI int ADDCALL hackrf_spiflash_read(hackrf_device* device,
# const uint32_t address, const uint16_t length, unsigned char* data);
libhackrf.hackrf_spiflash_read.restype = c_int
libhackrf.hackrf_spiflash_read.argtypes = [
    POINTER(hackrf_device), c_uint32, c_uint16, POINTER(c_ubyte)]

# extern ADDAPI int ADDCALL hackrf_cpld_write(hackrf_device* device,
#         unsigned char* const data, const unsigned int total_length);
libhackrf.hackrf_cpld_write.restype = c_int
libhackrf.hackrf_cpld_write.argtypes = [POINTER(hackrf_device), POINTER(c_ubyte), c_uint]

# extern ADDAPI int ADDCALL hackrf_board_id_read(hackrf_device* device,
# uint8_t* value);
libhackrf.hackrf_board_id_read.restype = c_int
libhackrf.hackrf_board_id_read.argtypes = [POINTER(hackrf_device), POINTER(c_uint8)]

# extern ADDAPI int ADDCALL hackrf_version_string_read(hackrf_device*
# device, char* version, uint8_t length);
libhackrf.hackrf_version_string_read.restype = c_int
libhackrf.hackrf_version_string_read.argtypes = [POINTER(hackrf_device), POINTER(c_char), c_uint8]

# extern ADDAPI int ADDCALL hackrf_set_freq(hackrf_device* device,
# const uint64_t freq_hz);
libhackrf.hackrf_set_freq.restype = c_int
libhackrf.hackrf_set_freq.argtypes = [POINTER(hackrf_device), c_uint64]

# extern ADDAPI int ADDCALL hackrf_set_freq_explicit(hackrf_device* device,
#         const uint64_t if_freq_hz, const uint64_t lo_freq_hz,
#         const enum rf_path_filter path);,
# libhackrf.hackrf_set_freq_explicit.restype = c_int
# libhackrf.hackrf_set_freq_explicit.argtypes = [c_uint64,
# c_uint64, ]

# extern ADDAPI int ADDCALL
# hackrf_set_sample_rate_manual(hackrf_device* device, const uint32_t
# freq_hz, const uint32_t divider);
libhackrf.hackrf_set_sample_rate_manual.restype = c_int
libhackrf.hackrf_set_sample_rate_manual.argtypes = [
    POINTER(hackrf_device), c_uint32, c_uint32]

# extern ADDAPI int ADDCALL hackrf_set_sample_rate(hackrf_device*
# device, const double freq_hz);
libhackrf.hackrf_set_sample_rate.restype = c_int
libhackrf.hackrf_set_sample_rate.argtypes = [POINTER(hackrf_device), c_double]

# extern ADDAPI int ADDCALL hackrf_set_amp_enable(hackrf_device*
# device, const uint8_t value);
libhackrf.hackrf_set_amp_enable.restype = c_int
libhackrf.hackrf_set_amp_enable.argtypes = [POINTER(hackrf_device), c_uint8]

# extern ADDAPI int ADDCALL
# hackrf_board_partid_serialno_read(hackrf_device* device,
# read_partid_serialno_t* read_partid_serialno);
libhackrf.hackrf_board_partid_serialno_read.restype = c_int
libhackrf.hackrf_board_partid_serialno_read.argtypes = [POINTER(hackrf_device)]

# extern ADDAPI int ADDCALL hackrf_set_lna_gain(hackrf_device* device,
# uint32_t value);
libhackrf.hackrf_set_lna_gain.restype = c_int
libhackrf.hackrf_set_lna_gain.argtypes = [POINTER(hackrf_device), c_uint32]

# extern ADDAPI int ADDCALL hackrf_set_vga_gain(hackrf_device* device,
# uint32_t value);
libhackrf.hackrf_set_vga_gain.restype = c_int
libhackrf.hackrf_set_vga_gain.argtypes = [POINTER(hackrf_device), c_uint32]

# extern ADDAPI int ADDCALL hackrf_set_txvga_gain(hackrf_device*
# device, uint32_t value);
libhackrf.hackrf_set_txvga_gain.restype = c_int
libhackrf.hackrf_set_txvga_gain.argtypes = [POINTER(hackrf_device), c_uint32]

# extern ADDAPI int ADDCALL hackrf_set_antenna_enable(hackrf_device*
# device, const uint8_t value);
libhackrf.hackrf_set_antenna_enable.restype = c_int
libhackrf.hackrf_set_antenna_enable.argtypes = [POINTER(hackrf_device), c_uint8]

# extern ADDAPI const char* ADDCALL hackrf_error_name(enum hackrf_error errcode);
# libhackrf.hackrf_error_name.restype = POINTER(c_char)
# libhackrf.hackrf_error_name.argtypes = []

# extern ADDAPI const char* ADDCALL hackrf_board_id_name(enum hackrf_board_id board_id);
# libhackrf.hackrf_board_id_name.restype = POINTER(c_char)
# libhackrf.hackrf_board_id_name.argtypes = []

# extern ADDAPI const char* ADDCALL hackrf_filter_path_name(const enum rf_path_filter path);
# libhackrf.hackrf_filter_path_name.restype = POINTER(c_char)
# libhackrf.hackrf_filter_path_name.argtypes = []

# extern ADDAPI uint32_t ADDCALL
# hackrf_compute_baseband_filter_bw_round_down_lt(const uint32_t
# bandwidth_hz);
libhackrf.hackrf_compute_baseband_filter_bw_round_down_lt.restype = c_uint32
libhackrf.hackrf_compute_baseband_filter_bw_round_down_lt.argtypes = [c_uint32]

# extern ADDAPI uint32_t ADDCALL
# hackrf_compute_baseband_filter_bw(const uint32_t bandwidth_hz);
libhackrf.hackrf_compute_baseband_filter_bw.restype = c_uint32
libhackrf.hackrf_compute_baseband_filter_bw.argtypes = [c_uint32]


class HackRf(object):
    __JELLYBEAN__ = 'Jellybean'
    __JAWBREAKER__ = 'Jawbreaker'
    __ONE__ = 'HackRF ONE'
    NAME_LIST = [__JELLYBEAN__, __JAWBREAKER__, __ONE__]

    def __init__(self, setup=True):
        """
        Create pointer to 'hackrf_device' and ready the object for calls.

        Args:
            setup (bool): Attempt to setup ('setup()') the device on initialization (default = True).
        """
        self.device = POINTER(hackrf_device)()
        self.callback = None
        self.is_open = False

        if setup:
            ret = self.setup()
            if ret != HackRfError.HACKRF_SUCCESS:
                logger.error("Failed to auto-setup device.")

    def __del__(self):
        """
        Automatic management / destruction of device.
        """
        if self.is_open:
            self.exit()

    def setup(self):
        """
        Initialize 'libhackrf' library.

        Returns:
            Status code from 'libhackrf'.
        """
        if not self.is_open:
            ret = libhackrf.hackrf_init()
            return self.open()
        else:
            return HackRfError.HACKRF_SUCCESS

    def exit(self):
        """
        Gracefully disconnect / destroy the 'hackrf_device' if it is in open state.

        Returns:
            Status code from 'libhackrf'.
        """
        ret = self.close()
        libhackrf.hackrf_exit()
        return ret

    def open(self):
        """
        Open the 'hackrf_device'.

        Returns:
            Status code from 'libhackrf'.
        """
        if self.is_open:
            logger.debug("Device already open.")
            return HackRfError.HACKRF_SUCCESS

        ret = libhackrf.hackrf_open(self.device)
        if ret == HackRfError.HACKRF_SUCCESS:
            self.is_open = True
            logger.debug('Successfully opened HackRf device.')
        else:
            logger.error('No Hack Rf Detected (%d)!' % ret)

        return ret

    def close(self):
        """
        Close the 'hackrf_device' and set appropriate states.

        Returns:
            Status code from 'libhackrf'.
        """
        if self.is_streaming():
            self.stop_rx_mode()

        ret = libhackrf.hackrf_close(self.device)
        if ret == HackRfError.HACKRF_SUCCESS:
            self.is_open = False
            logger.debug('Successfully closed HackRf device.')
        else:
            logger.error('Failed to close!')

        return ret

    def start_rx_mode(self, set_callback, obj=None):
        """
        Register RX callback function.

        Args:
            set_callback (CFUNCTYPE(c_int, POINTER(hackrf_transfer))):
                Callback function to be called when a new sample buffer arrives.
                Can be defined in pure Python or connected to C function via 'ctypes'.

                int (*callback)(hackrf_transfer *transfer);

            obj (ctypes.c_void_p): Void pointer to object used in callback, if desired (default = None).

        Returns:
            Status code from 'libhackrf'.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR

        self.callback = _hackrf_callback(set_callback)
        ret = libhackrf.hackrf_start_rx(self.device, self.callback, obj)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully started HackRf in Receive Mode.')
        else:
            logger.error('Failed to start HackRf in Receive Mode.')

        return ret

    def stop_rx_mode(self):
        """
        Stop RX mode.

        Returns:
            Status code from 'libhackrf'.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR

        ret = libhackrf.hackrf_stop_rx(self.device)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully stop HackRf in Recieve Mode')
        else:
            logger.error('Failed to stop HackRf in Recieve Mode')

        return ret

    def start_tx_mode(self, set_callback):
        """
        Register TX callback function.

        Args:
            set_callback (CFUNCTYPE(c_int, POINTER(hackrf_transfer))):
                Callback function to be called when a new sample buffer arrives.
                Can be defined in pure Python or connected to C function via 'ctypes'.

                int (*callback)(hackrf_transfer *transfer);

        Returns:
            Status code from 'libhackrf'.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR

        self.callback = _hackrf_callback(set_callback)
        ret =  libhackrf.hackrf_start_tx(self.device, self.callback, None)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully start HackRf in Transfer Mode')
        else:
            logger.error('Failed to start HackRf in Transfer Mode')

        return ret

    def stop_tx_mode(self):
        """
        Stop TX mode.

        Returns:
            Status code from 'libhackrf'.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR


        ret = libhackrf.hackrf_stop_tx(self.device)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully stop HackRf in Transfer Mode')
        else:
            logger.error('Failed to stop HackRf in Transfer Mode')

        return ret

    def board_id_read(self):
        """
        Get board ID from device.

        Returns:
            Retrieved board ID (-1 on failure).
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return None

        value = c_uint8()
        ret = libhackrf.hackrf_board_id_read(self.device, byref(value))
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully got Board Id')
            return value.value

        logger.error('Failed to get Board Id')
        return -1

    def version_string_read(self): #POINTER(c_char), c_uint8
        """
        Get version string from board.

        Returns:
            Retrieved version string ('None' on failure).
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return None

        version = create_string_buffer(20)
        lenth = c_uint8(20)
        ret = libhackrf.hackrf_version_string_read(self.device, version, lenth)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully got HackRf Version String')
            return version.value

        logger.error('Failed to get Version String')
        return None

    def set_freq(self, freq_hz):
        """
        Set front-end RF frequency (in Hz).

        Args:
            freq_hz (float): Desired frequency to tune to in [Hz].
                             Converted from 'float' to 'int' in this function for
                             compatibility with 'libhackrf' call.

        Returns:
             Status code from 'libhackrf'.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR

        try:
            # Attempt to convert to an integer.
            freqHz = int(freq_hz)
        except Exception as e:
            logger.error("Failed to convert to integer: %s." % str(e))
            return HackRfError.HACKRF_ERROR

        ret = libhackrf.hackrf_set_freq(self.device, freqHz)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set frequency with value [%d]', freqHz)
        else:
            logger.error('Error setting frequency with value [%d]', freqHz)

        return ret

    def is_streaming(self):
        """
        Check if device is currently streaming IQ.

        Return:
            Boolean indicating the device is streaming streaming.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return False

        ret = libhackrf.hackrf_is_streaming(self.device)
        return (ret == HackRfError.HACKRF_TRUE)

    def set_lna_gain(self, gain):
        """
        Set the LNA (low-noise amplifier) gain (in dB).

        Args:
            gain (float): Front-end LNA gain desired (range: 0-40 [dB] in 8 [dB] steps).
                          Rounded to closest 8 [dB] step and converted from 'float' to
                          'int' for compatibility with 'libhackrf' call.

        Returns:
              Status code from 'libhackrf'.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR

        # Convert to integer.
        try:
            gainDB = int(gain+0.5)
        except Exception as e:
            logger.error("Failed to convert to integer: %s." % str(e))
            return HackRfError.HACKRF_ERROR
        # Get closest gain.
        gainDB = ((gainDB+4) / 8) * 8
        # Set LNA gain.
        ret = libhackrf.hackrf_set_lna_gain(self.device, gainDB)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set LNA gain to [%d]', gainDB)
        else:
            logger.error('Failed to set LNA gain to [%d]', gainDB)

        return ret

    def set_vga_gain(self, gain):
        """
        Set the RX VGA (variable-gain amplifier) gain (in dB).

        Args:
            gain (float): VGA gain desired (range: 0-62 [dB] in 2 [dB] steps).
                          Rounded to closest 2 [dB] step and converted from 'float' to
                          'int' for compatibility with 'libhackrf' call.

        Returns:
            Status code from 'libhackrf'.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR

        # Convert to integer.
        try:
            gainDB = int(gain+0.5)
        except Exception as e:
            logger.error("Failed to convert to integer: %s." % str(e))
            return HackRfError.HACKRF_ERROR
        # Get closest gain.
        gainDB = ((gainDB+1) / 2) * 2
        # Set RX VGA gain.
        ret = libhackrf.hackrf_set_vga_gain(self.device, gainDB)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set VGA gain to [%d]', gainDB)
        else:
            logger.error('Failed to set VGA gain to [%d]', gainDB)

        return ret

    def set_txvga_gain(self, gain):
        """
        Set the TX VGA (variable-gain amplifier) gain (in dB).

        Args:
            gain (float): TX VGA gain desired (range: 0-47 [dB] in 1 [dB] steps).
                          Rounded to closest 1 [dB] step and converted from 'float' to
                          'int' for compatibility with 'libhackrf' call.

        Returns:
            Status code from 'libhackrf'.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR

        # Convert to integer.
        try:
            gainDB = int(gain+0.5)
        except Exception as e:
            logger.error("Failed to convert to integer: %s." % str(e))
            return HackRfError.HACKRF_ERROR
        # Set TX VGA gain.
        ret = libhackrf.hackrf_set_txvga_gain(self.device, gainDB)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set TXVGA gain to [%d]', gainDB)
        else:
            logger.error('Failed to set TXVGA gain to [%d]', gainDB)

        return ret

    def set_antenna_enable(self, enable=True):
        """
        Set antenna enable flag.

        Args:
            enable (bool): Boolean indicating the enable flag value (default = True).

        Returns:
            Status code from 'libhackrf'.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR

        val = int(enable)
        ret =  libhackrf.hackrf_set_antenna_enable(self.device, val)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set antenna_enable')
        else:
            logger.error('Failed to set antenna_enable')

        return ret

    def set_sample_rate(self, rate):
        """
        Set sample rate of device (in Hz).

        Args:
            rate (float): Desired sample rate (in Hz).
                          Converted from 'float' to 'int' in this function for
                          compatibility with 'libhackrf' call.

        Returns:
            Tuple containing status code from 'libhackrf' and set rate (float).
            Example:    (HackRfError.HACKRF_SUCCESS, 8e6)
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR

        # Convert to integer.
        try:
            rateHz = int(rate+0.5)
        except Exception as e:
            logger.error("Failed to convert to integer: %s." % str(e))
            return HackRfError.HACKRF_ERROR
        # Set sample rate.
        ret = libhackrf.hackrf_set_sample_rate(self.device, rateHz)
        if ret != HackRfError.HACKRF_SUCCESS:
            logger.error('Error setting Sample Rate with Frequency [%d]', rateHz)
        else:
            logger.debug('Successfully set Sample Rate with Frequency [%d]', rateHz)

        return ret

    def set_amp_enable(self, enable=True):
        """
        Enable amplifier port (allows use of external amplifiers).

        Args:
            enable (bool): Boolean indicating the enable flag value (default = True).

        Results:
             Status code from 'libhackrf'.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR

        value = int(enable)
        # Set amplifier enable flag.
        ret =  libhackrf.hackrf_set_amp_enable(self.device, value)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set amp')
        else:
            logger.error('Failed to set amp')

        return ret

    def set_baseband_filter_bandwidth(self, bw_hz):
        """
        Set baseband filter bandwidth (in Hz).

        Args:
            bw_hz (float): Filter bandwidth (in Hz).
                           Converted from 'float' to 'int' in this function for
                           compatibility with 'libhackrf' call.

        Results:
             Status code from 'libhackrf'.
        """
        if not self.is_open:
            logger.error("HackRf not opened.")
            return HackRfError.HACKRF_ERROR

        # Convert to integer.
        try:
            bwHz = int(bw_hz+0.5)
        except Exception as e:
            logger.error("Failed to convert to integer: %s." % str(e))
            return HackRfError.HACKRF_ERROR

        ret = libhackrf.hackrf_set_baseband_filter_bandwidth(self.device, bwHz)
        if ret != HackRfError.HACKRF_SUCCESS:
            logger.error('Failed to set Baseband Filter Bandwidth with value [%d]', bwHz)
        else:
            logger.debug('Successfully set Baseband Filter Bandwidth with value [%d]', bwHz)

        return ret

    def max2837_read(self, register_number, value):
        pass
        return libhackrf.hackrf_max2837_read(self.device, register_number, value)

    def max2837_write(self, register_number, value):
        pass
        return libhackrf.hackrf_max2837_weite(self.device, register_number, value)

    def si5351c_read(self, register_number, value):
        pass
        return libhackrf.hackrf_si5351c_read(self.device, register_number, value)

    def si5351c_write(self, register_number, value):
        pass
        return libhackrf.hackrf_si5351c_write(self.device, register_number, value)

    def rffc5071_read(self, register_number, value):
        pass
        return libhackrf.hackrf_rffc5071_read(self.device, register_number, value)

    def rffc5071_write(self, register_number, value):
        pass
        return libhackrf.hackrf_rffc5071_write(self.device, register_number, value)

    def spiflash_erase(self):
        pass
        return libhackrf.hackrf_spiflash_erase(self.device)

    def spiflash_write(self, address, length, data):
        pass
        return libhackrf.hackrf_spiflash_write(self.device, address, length, data)

    def spiflash_read(self, address, length, data):
        pass
        return libhackrf.hackrf_spiflash_read(self.device, address, length, data)

    def cpld_write(self, data, total_length):
        pass
        return libhackrf.hackrf_cpld_write(self.device, data, total_length)

    def set_sample_rate_manual(self, freq_hz, divider):
        pass
        return libhackrf.hackrf_set_sample_rate_manual(self.device, freq_hz, divider)

    def compute_baseband_filter_bw_round_down_lt(self, bandwidth_hz):
        pass
        return libhackrf.hackrf_compute_baseband_filter_bw_round_down_lt(bandwidth_hz)

    def compute_baseband_filter_bw(self, bandwidth_hz):
        pass
        return libhackrf.hackrf_compute_baseband_filter_bw(bandwidth_hz)

    # def hackrf_set_freq_explicit(self, if_freq_hz, lo_freq_hz, path):
    #     pass
    # return libhackrf.hackrf_set_freq_explicit(if_freq_hz, lo_freq_hz, path)

    # def hackrf_board_partid_serialno_read(self, read_partid_serialno):
    #     pass
    # return libhackrf.hackrf_board_partid_serialno_read(read_partid_serialno)

    # def hackrf_error_name(self, errcode):
    #     pass
    #     return libhackrf.hackrf_error_name(errcode)

    # def hackrf_board_id_name(self, board_id):
    #     pass
    #     return libhackrf.hackrf_board_id_name(board_id)

    # def hackrf_filter_path_name(self, path):
    #     pass
    #     return libhackrf.hackrf_filter_path_name(path)

    # out[0] = (out[0] - 127)*(1.0/128);
    def packed_bytes_to_iq(self, bytes):
        ''' Convenience function to unpack array of bytes to Python list/array
        of complex numbers and normalize range.  size 16*32*512 262 144
        '''
        # use NumPy array
        iq = np.empty(len(bytes)//2, 'complex')
        iq.real, iq.imag = bytes[::2], bytes[1::2]
        iq /= 128.0
        return iq

    def packed_bytes_to_iq_withsize(self, bytes, size):
        ''' Convenience function to unpack array of bytes to Python list/array
        of complex numbers and normalize range.
        '''
        # use NumPy array
        iq = np.empty(size , 'complex')
        bytes2 = bytes[0:size * 2]
        iq.real, iq.imag = bytes2[::2], bytes2[1::2]
        iq /= 128.0
        return iq
