# NAV-TIMEGAL (0x01 0x25)
# UTC Time Solution
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 37

uint32 i_tow             # GPS time of week of the navigation epoch [ms]

uint32 gal_tow           # Galileo time of week (rounded to seconds) [s]
int32 f_gal_tow           # Fractional part of the Galileo time of week [ns]
int16 gal_wno            # Galileo week number
int8 leap_s              # Galileo leap seconds [s]

uint8 valid             # Validity Flags
uint8 VALID_GAL_TOW = 1       # Valid galTow and fGalTow
uint8 VALID_GAL_WEEK = 2       # Valid galWno
uint8 VALID_LEAP_S = 4        # Valid leapS

uint32 t_acc             # Time Accuracy Estimate [ns]