/***********************************************************/
/*Constants                                                */
/***********************************************************/

#define V_MOTOR_RATED 16 //rated motor voltage [V]. this must be set to be less than the voltage of the batteries
#define PERIOD_US 100 //define the PWM period in microseconds, currently set for 10kHz, was 40kHz before.
 
/***********************************************************/
/*Pin setup                                               */
/***********************************************************/

//Enable PWM inout to motorcontroller
PwmOut load_motor(p21);                 //PWM for load
PwmOut DUT_bat_motor(p22);              //PWM for battery   
PwmOut DUT_sc_motor(p24);               //PWM for discharging SC

DigitalOut bat_control_pin(p27);                //relay switch  
DigitalOut cap_control_pin(p28);                //relay switch
DigitalOut recharge_cap_control_pin(p29);       //relay switch 

//Motor direction control on motor controller switch 2
//DigitalOut DUT_direction(p5);
//DigitalOut load_direction(p8);

/***********************************************************/
/*Subroutines                                              */
/***********************************************************/

//initialize PWM
void init_pwm () 
{
    //set the PWM channel periods
    //Battery for Drive Motor
    DUT_bat_motor.period_us(PERIOD_US);
    
    //Capacitor
    DUT_sc_motor.period_us(PERIOD_US);
    
    //Battery for Load Motor
    load_motor.period_us(PERIOD_US);
    
    //initialize duty cycle to 0 to make sure motors are off
    DUT_bat_motor = 0;
    DUT_sc_motor = 0;
    load_motor = 0;
}

/*
Set PWM duty cycle
 - Motors are connected so that they will always oppose each other.
 - Duty cycle (a float from 0-1) determines the voltage applied to the motor terminals as a percentage of the rated voltage. eg. if duty is 0.5 and rated voltage is 6V, then terminal voltage will be 3V
*/
void set_duty (float DUT_demand, float load_demand) 
{


// 0 - Corresponds to ON
// 1 - Corresponds to OFF

//DUT batt and cap enabled
#ifdef DUT_BATT_CAP_ENABLE
    if (M1 == 0 && M2 == 1) 
     {
        //if cap is currently being used
        bat_control_pin = 1;             //turn off battery power
        cap_control_pin = 0;             //turn on Capacitor discharging
        recharge_cap_control_pin = 1;    //turn off recharging of capacitor
                
        myled3 = 1;
        myled2 = 0;
        myled4 = 0;
                
        v_cap = get_voltage(v_cap_measure_pin); //get voltage of cap
        DUT_demand = DUT_demand;                //*(V_MOTOR_RATED/v_cap); //scale the demand so that the voltage to the motor does not exceed its rated max
        DUT_sc_motor.write(DUT_demand);         //write new DUT duty cycle
      }
      
    else if (M1 == 1 && M2 == 0) 
     { 
        bat_control_pin = 0;            //turn on battery power
        cap_control_pin = 1;            //turn off Capacitor discharging  
        recharge_cap_control_pin = 0;    //turn on recharging of capacitor      
        
        myled2 = 1;
        myled3 = 0;
        myled4 = 0;
      
        //if battery is currently being used
        v_DUT_batt = get_voltage(v_DUT_batt_measure_pin); //get voltage of DUT battery
        DUT_demand = DUT_demand;    //*(V_MOTOR_RATED/v_DUT_batt); //scale the demand so that the voltage to the motor does not exceed its rated max
        DUT_bat_motor.write(DUT_demand); //write new DUT duty cycle
      }
      
    else if (M3 == 1 && M2 == 0 && M1 == 0)
     {
        bat_control_pin = 1;
        cap_control_pin = 1;
        recharge_cap_control_pin = 0;   //turn on recharging of capacitor 
      
        myled4 = !myled4;
        myled2 = 0;
        myled3 = 0;
        v_cap = get_voltage(v_cap_measure_pin); //get voltage of cap
      }
      
     else
     {
        big_error = 1; //error flag
        big_error_led = 1; //turn on error LED
     }

#endif

//DUT batt connected
#ifdef DUT_BATT_ENABLE
    //v_DUT_batt = get_voltage(v_DUT_batt_measure_pin); //get voltage of DUT battery
    DUT_demand = DUT_demand;//*(V_MOTOR_RATED/v_DUT_batt); //scale the demand so that the voltage to the motor does not exceed its rated max
    DUT_bat_motor.write(DUT_demand); //change DUT duty cycle based on demand
#endif

//Load batt connected
#ifdef LOAD_BATT_ENABLE
    //v_load_batt = get_voltage(v_load_batt_measure_pin); //get voltage of load battery
    load_demand = load_demand;  //*(V_MOTOR_RATED/v_load_batt); //scale the demand so that the voltage to the motor does not exceed its rated max
    load_motor.write(load_demand); //change load duty cycle based on demand
#endif

//External Power
#ifdef DUT_EXTERNAL_POWER_ENABLE
    DUT_motor.write(DUT_demand); //change DUT duty cycle based on demand
#endif

#ifdef LOAD_EXTERNAL_POWER_ENABLE
    load_motor.write(load_demand); //change load duty cycle based on demand
#endif

//set motor directions
    //DUT_direction.write(DUT_input_direc);
    //load_direction.write(load_input_direc);
}