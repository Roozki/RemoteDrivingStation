#define CONTROL_RATE 60.0
#define GEAR_REVERSE -1
#define GEAR_PARKING -2
#define GEAR_NEUTRAL 0
#define GEAR_1 1 //or drive
#define GEAR_2 2 //guessing this is how autoware deals with manual cars
#define GEAR_3 3
#define NUM_LIGHTS 5


// void ManualControlNode::EngineCycle(){
//         float rpm = vehicle.gas_pedal;
//         float out_velocity = 0;
//         if(vehicle.manual){
//         switch (vehicle.curr_gear)
//         {
//         case GEAR_1:
//             out_velocity = vehicle.gas_pedal/4;
//             break;
//         case GEAR_2:
//             out_velocity = vehicle.gas_pedal/2;
//         case GEAR_3:
//             out_velocity = vehicle.gas_pedal;
//         default:
//             out_velocity = 0;
//             break;
//         }
//         }else{
//         switch (vehicle.curr_gear)
//         {
//         case GEAR_1:
//             out_velocity = vehicle.gas_pedal/4;
//             break;
//         case GEAR_2:
//             out_velocity = vehicle.gas_pedal/2;
//         case GEAR_3:
//             out_velocity = vehicle.gas_pedal;
//         default:
//             out_velocity = 0;
//             break;
//         }

//         }
    

// }