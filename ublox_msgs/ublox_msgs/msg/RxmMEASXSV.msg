# See message RXM-MEASX
#

uint8 gnss_id            # GNSS Id
uint8 sv_id              # Satellite Id
uint8 correction        # Carrier noise ratio

uint8 mpath_indic        # multipath index
uint8 MPATHINDIC_NO = 0
uint8 MPATHINDIC_LOW = 1
uint8 MPATHINDIC_MEDIUM = 2
uint8 MPATHINDIC_HIGH = 3

int32 doppler_ms         # Doppler measurement [m/s]
int32 doppler_hz         # Doppler measurement [Hz]
uint16 whole_chips       # Whole value of code phase measurement
uint16 frac_chips        # Fractional value of code phase measurement
uint32 code_phase        # Code phase [ms]
uint8 int_code_phase      # Integer part of code phase [ms]
uint8 pseu_range_rmse_rr   # pseudorange RMS error index
uint8[2] reserved4      # Reserved