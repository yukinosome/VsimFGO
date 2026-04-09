# See message NAV-ORB

uint8 gnss_id            # GNSS Id
uint8 sv_id              # Satellite Id

uint8 sv_flag            # Information Flags
uint8 SVFLAG_UNKNOWN = 0
uint8 SVFLAG_HEALTHY = 1
uint8 SVFLAG_UNHEALTHY = 2
uint8 VISIBILITY_UNKNOWN = 0
uint8 VISIBILITY_BELOWHORIZON = 4
uint8 VISIBILITY_ABOVEHORIZON = 8
uint8 VISIBILITY_ABOVEELEVATION = 12

uint8 eph               # Ephemeris Data
uint8 EPH_USABILITY_NO = 0          # no longer useable
uint8 EPH_USABILITY_UNKNOWN = 31    # unknown
uint8 EPH_USABILITY_450PLUS = 30    # more than 450 minutes
uint8 EPH_SOURCE_NOTAVAILABLE = 0
uint8 EPH_SOURCE_GNSS = 32    
uint8 EPH_SOURCE_EXTAID = 64

uint8 alm               # Almanac Data
uint8 ALM_USABILITY_NO = 0          # no longer useable
uint8 ALM_USABILITY_UNKNOWN = 31    # unknown
uint8 ALM_USABILITY_450PLUS = 30    # more than 30 days
uint8 ALM_SOURCE_NOTAVAILABLE = 0
uint8 ALM_SOURCE_GNSS = 32    
uint8 ALM_SOURCE_EXTAID = 64

uint8 other_orb               # Ephemeris Data
uint8 OTHERORB_USABILITY_NO = 0          # no longer useable
uint8 OTHERORB_USABILITY_UNKNOWN = 31    # unknown
uint8 OTHERORB_USABILITY_450PLUS = 30    # more than 450 minutes
uint8 OTHERORB_TYPE_NOTAVAILABLE = 0
uint8 OTHERORB_TYPE_ANOFFLINE = 32    # AssistNow Offline
uint8 OTHERORB_TYPE_ANAUTONOMOUS = 64 # AssistNow Autonomous

