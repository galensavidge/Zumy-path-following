# Zumy-path-following
Contains code to use the TSL1401CL line sensor and a Zumy chassis for simple path following behavior

OPERATION INSTRUCTIONS
1. Calibrate line sensor and adjust LINE_THRESHOLD and SPEED_PWM as necessary
2. Place Zumy with the line sensor over the track and power it on
3. For data collection, connect Zumy to a PC via USB serial once it stops moving

NOTES
1. Works best with DC or other constant lighting. The optitrack table lighting works well.
2. Either light-on-dark or dark-on-light lines work fine, though LINE_THRESHOLD may have to be adjusted depending on line/surface contrast.
