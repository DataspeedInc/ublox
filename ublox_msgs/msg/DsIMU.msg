# DS-IMU (0xC4 0x01)
# Dataspeed IMU
#
# See important comments concerning validity of position given in section
# Navigation Output Filters.
#

uint8 CLASS_ID = 196
uint8 MESSAGE_ID = 1

uint8 version
uint8 flags
uint8[2] reserved0

uint32 iTOW             # GPS Millisecond Time of Week [ms]

int32 linAccelX         # X component of linear accelearation [1e-7 m/s^2]
int32 linAccelY         # Y component of linear accelearation [1e-7 m/s^2]
int32 linAccelZ         # Z component of linear accelearation [1e-7 m/s^2]

int32 angVelX           # X component of angular velocity [5e-6 deg/s]
int32 angVelY           # Y component of angular velocity [5e-6 deg/s]
int32 angVelZ           # Z component of angular velocity [5e-6 deg/s]

float64 orientationW    # W component of orientation quaternion
float64 orientationX    # X component of orientation quaternion
float64 orientationY    # Y component of orientation quaternion
float64 orientationZ    # Z component of orientation quaternion

uint32 lAcc             # Linear acceleration accuracy estimate [1e-7 m/s^2]
uint32 aAcc             # Angular velofity accuracy estimate [5e-6 deg/s]
uint32 oAcc             # Orientation accuracy estimate [1e-5 deg]
