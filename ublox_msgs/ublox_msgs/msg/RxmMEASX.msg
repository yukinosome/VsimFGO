# RXM-MEASX (0x02 0x14)
# Satellite measurements for RRLP

# ...
#

uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 20

uint8 version           # Message version

uint8[3] reserved0      # Reserved

uint32 gps_tow           # GPS Measurement ref time [ms]
uint32 glo_tow           # GLONASS Measurement ref time [ms]
uint32 bds_tow           # BeiDou Measurement ref time [ms]

uint8[4] reserved1      # Reserved

uint32 qzss_tow          # QZSS Measurement ref time [ms]
uint16 gps_tow_acc        # GPS Measurement ref time accuracy [ms]
uint16 glo_tow_acc        # GLONASS Measurement ref time accuracy [ms]
uint16 bds_tow_acc        # BeiDou Measurement ref time accuracy [ms]

uint8[2] reserved2     # Reserved

uint16 qzss_tow_acc       # QZSS Measurement ref time accuracy [ms]
uint8 num_sv             # Number of satellites

uint8 flags             # Flags
uint8 FLAGS_TOWSET_NO = 0
uint8 FLAGS_TOWSET_YES = 1

uint8[8] reserved3      # Reserved

RxmMEASXSV[] sv