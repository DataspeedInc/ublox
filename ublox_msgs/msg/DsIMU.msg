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

int32 lin_accel_x         # X component of linear accelearation [1e-7 m/s^2]
int32 lin_accel_y         # Y component of linear accelearation [1e-7 m/s^2]
int32 lin_accel_z         # Z component of linear accelearation [1e-7 m/s^2]

int32 ang_vel_x           # X component of angular velocity [5e-6 deg/s]
int32 ang_vel_y           # Y component of angular velocity [5e-6 deg/s]
int32 ang_vel_z           # Z component of angular velocity [5e-6 deg/s]

float64 orientation_w    # W component of orientation quaternion
float64 orientation_x    # X component of orientation quaternion
float64 orientation_y    # Y component of orientation quaternion
float64 orientation_z    # Z component of orientation quaternion

uint32 l_acc             # Linear acceleration accuracy estimate [1e-7 m/s^2]
uint32 a_acc             # Angular velofity accuracy estimate [5e-6 deg/s]
uint32 o_acc             # Orientation accuracy estimate [1e-5 deg]
