float64 steering_angle
float64 gas_pedal
float64 brake_pedal
float64 clutch
int16 gear
#gears
#reverse -> -1
#park -> 0
#neutral -> 1
#drive / gear 1 -> 1
#and so on
bool handbreak
int16[] lights
#lights        -> index
#left_signal   ->   0
#right_signal  ->   1
#front         ->   2
#hazards       ->   3
#back_driving  ->   4

bool left_signal
bool right_signal
bool hazards
bool front_lights
bool rear_lights

bool manual
