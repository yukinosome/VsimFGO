# NAV-ORB (0x01 0x34)
# GNSS Orbit Database Info
#
# Status of the GNSS orbit database knowlede
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 52

uint32 i_tow             # GPS Millisecond time of week [ms]
uint8 version           # Message version
uint8 num_sv             # Number of SVs in database
uint8[2] reserved0      # Reserved

NavORBSV[] sv
