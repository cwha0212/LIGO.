# LIO local frame ↔ ENU heading (yaw) / local→ENU transform lifecycle.
# TIME-PAIR LIO–GNSS alignment (NMEALIAlign) or indoor reloc (SetInitFromLocalization).
std_msgs/Header header

uint8 STATUS_UNALIGNED=0
# Buffers filling; waiting for motion / 0.3m+ pairs / RMSE gate (icp_tf_ready false)
uint8 STATUS_COLLECTING=1
# icp_tf_ready: local→ENU rotation/translation locked for fusion / ENU topics
uint8 STATUS_LOCKED=2

uint8 status

uint8 SOURCE_NONE=0
uint8 SOURCE_TIME_PAIR_ALIGN=1
uint8 SOURCE_INDOOR_RELOC=2
# Meaningful when status==STATUS_LOCKED
uint8 source

bool icp_tf_ready
float64 yaw_enu_local_rad
# Post-alignment RMSE (m) from TIME-PAIR path; NaN if indoor reloc or unknown
float64 post_rmse_m
