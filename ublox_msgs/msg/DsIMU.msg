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

uint32 i_tow            # GPS Millisecond Time of Week [ms]

int32 raw_accel_x         # X component of vehicle body accelearation [1e-7 m/s^2]
int32 raw_accel_y         # Y component of vehicle body accelearation [1e-7 m/s^2]
int32 raw_accel_z         # Z component of vehicle body accelearation [1e-7 m/s^2]

int32 comp_accel_x         # X component of gravity-compensated accelearation [1e-7 m/s^2]
int32 comp_accel_y         # Y component of gravity-compensated accelearation [1e-7 m/s^2]
int32 comp_accel_z         # Z component of gravity-compensated accelearation [1e-7 m/s^2]

int32 ang_vel_x           # X component of angular velocity [5e-6 deg/s]
int32 ang_vel_y           # Y component of angular velocity [5e-6 deg/s]
int32 ang_vel_z           # Z component of angular velocity [5e-6 deg/s]

int32 body_roll           # Roll angle of vehicle body w.r.t. gravity [1e-5 deg]
int32 body_pitch          # Pitch angle of vehicle body w.r.t. gravity [1e-5 deg]

uint32 l_acc             # Linear acceleration accuracy estimate [1e-7 m/s^2]
uint32 a_acc             # Angular velofity accuracy estimate [5e-6 deg/s]
uint32 o_acc             # Orientation accuracy estimate [1e-5 deg]
