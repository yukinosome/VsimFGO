# NAV-SIG (0x01 0x43)
# Signal Information
# 
# This message displays information about signals 
# currently tracked by the receiver
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 67

uint32 i_tow             # GPS Millisecond time of week [ms]
uint8 version           # Message version
uint8 num_sigs           # Number of signals

uint8[2] reserved0      # Reserved

NavSIGSV[] sv