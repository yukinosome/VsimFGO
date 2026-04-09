# see message NavSIG
#

uint8 gnss_id            # GNSS Identifier
uint8 sv_id              # Satellite identifier
uint8 sig_id             # Signal identifier
uint8 freq_id            # GLONASS frequency slot
int16 pr_res             # Pseudorange residual [m]
uint8 cno               # Carrier-to-noise density ratio [dBHz]

uint8 quality_ind        # Signal quality indicator
uint8 QUALITYIND_NOSIGNAL = 0
uint8 QUALITYIND_SEARCHING = 1
uint8 QUALITYIND_ACQUIRED = 2
uint8 QUALITYIND_UNUSABLE = 3
uint8 QUALITYIND_CODELOCKED = 4
uint8 QUALITYIND_CODECARRIERLOCKED = 5

uint8 corr_source        # Correction source
uint8 CORRSOURCE_NOCORR = 0
uint8 CORRSOURCE_SBAS = 1
uint8 CORRSOURCE_BEIDOU = 2
uint8 CORRSOURCE_RTCM2 = 3
uint8 CORRSOURCE_RTCM3OSR = 4
uint8 CORRSOURCE_RTCM3SSR = 5
uint8 CORRSOURCE_QZSS = 6

uint8 iono_model         # Ionospheric model used
uint8 IONOMODEL_NOMODEL = 0
uint8 IONOMODEL_KLOBGPS = 1
uint8 IONOMODEL_SBAS = 2
uint8 IONOMODEL_KLOBBD = 3
uint8 IONOMODEL_DUALFREQ = 8

uint16 sig_flags          # Signal related flags
uint16 SIGFLAGS_UNKNOWN = 0
uint16 SIGFLAGS_HEALTHY = 1
uint16 SIGFLAGS_UNHEALTHY = 2
uint16 SIGFLAGS_PRSMOOTHED = 4
uint16 SIGFLAGS_PRUSED = 8
uint16 SIGFLAGS_CRUSED = 16
uint16 SIGFLAGS_DOUSED = 32
uint16 SIGFLAGS_PRCORRUSED = 64
uint16 SIGFLAGS_CRCORRUSED = 128
uint16 SIGFLAGS_DOCORRUSED = 256

uint8[4] reserved1      # Reserved