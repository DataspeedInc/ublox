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

int32 accel_x         # X component of vehicle body accelearation [1e-7 m/s^2]
int32 accel_y         # Y component of vehicle body accelearation [1e-7 m/s^2]
int32 accel_z         # Z component of vehicle body accelearation [1e-7 m/s^2]

int32 gyro_x           # X component of angular velocity [5e-6 deg/s]
int32 gyro_y           # Y component of angular velocity [5e-6 deg/s]
int32 gyro_z           # Z component of angular velocity [5e-6 deg/s]

int32 vel_x           # X component of INS velocity [1e-6 m/s]
int32 vel_y           # Y component of INS velocity [1e-6 m/s]
int32 vel_z           # Z component of INS velocity [1e-6 m/s]

uint32 vel_x_std    # X velocity standard deviation in m/s
uint32 vel_y_std    # Y velocity standard deviation in m/s
uint32 vel_z_std    # Z velocity standard deviation in m/s

int32 roll           # Roll angle of vehicle body w.r.t. gravity [1e-5 deg]
int32 pitch          # Pitch angle of vehicle body w.r.t. gravity [1e-5 deg]
int32 heading          # Heading angle of vehicle w.r.t. North [1e-5 deg]

uint32 roll_acc             # Linear acceleration accuracy estimate [1e-7 m/s^2]
uint32 pitch_acc             # Angular velofity accuracy estimate [5e-6 deg/s]
uint32 heading_acc             # Orientation accuracy estimate [1e-5 deg]
