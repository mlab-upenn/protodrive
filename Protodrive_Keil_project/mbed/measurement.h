/***********************************************************/
/*Constants                                                */
/***********************************************************/
#define ISENSOR_SLOPE_DUT  0.998    //2.8292 //3.484 //slope of the current sensor in [A/V]
#define Y_INTERCEPT_DUT  -2.491      //-5.862     //-5.244 //sensor voltage at 0 current
#define NUM_HALF_CYCLES 4           //set the number of ticks over which to measure speed
#define WAIT_BEFORE_SPEEDSTOP 7.5   //amount of time to wait before declaring that the motor is not moving. 7.5s corresponds to a speed of 1 rev/min
#define R1 22000        //voltage measurement resistor connected to device under measurement [Ohm]
#define R2 100000        //voltage measurement resistor connected to ground [Ohm]

/***********************************************************/
/*Variables                                                */
/***********************************************************/

float end, begin, current_read, current_read_cap ; 
int current_state;
int count = 0;

Timer speed_timer;

/***********************************************************/
/*Pin setup                                                */
/***********************************************************/
//Encoder
DigitalIn speed_yellow(p6); //Connected to the cathode of TLP424-4 whose o/p(pin 14-collector) is connected to the Yellow Wire from the motor(Hall sensor o/p)

/***********************************************************/
/*Subroutines                                              */
/***********************************************************/

//get the voltage for one of the energy storage devices. Takes pin as a parameter
float get_voltage (AnalogIn& pin) 
{
    float voltage;
    voltage = pin.read();//*3.3*((R1+R2)/R2); //scaling to account for voltage divider
    return voltage;
}

//returns current in amps
#ifdef CURRENT_SENSOR_ON
void get_current() 
{
    current_read = current_sense.read(); //read raw AnalogIn value of current
    current = ISENSOR_SLOPE_DUT * current_read * 3.3 + Y_INTERCEPT_DUT; //scaling to get DUT current in A
    
    current_read_cap = current_sense_cap.read(); //read raw AnalogIn value of current
    current_cap = ISENSOR_SLOPE_DUT * current_read_cap * 3.3 + Y_INTERCEPT_DUT; //scaling to get DUT current in A
}

#endif

//returns speed in rad/sec
void get_speed() 
{
    current_state = speed_yellow; //get the current state of speed_yellow pin (0 or 1)
    speed_timer.start();
    while (speed_yellow == current_state && speed_timer <= WAIT_BEFORE_SPEEDSTOP) {} //wait for value of the speed_yellow to change, indicating the beginning of a new cycle
    if (speed_timer < WAIT_BEFORE_SPEEDSTOP) 
    { 
        //check that the timer is less than WAIT_BEFORE_SPEEDSTOP, to make sure it has not been running for too long. This will happen if speed = 0
        speed_timer.reset(); //reset the timer so that it starts timing from the beginning of the new cycle
        begin = speed_timer.read();
        for (int i = 1; i <= NUM_HALF_CYCLES; i++) 
        {
            //loop to allow timing over a set number of encoder cycles
            current_state = speed_yellow;
            while (speed_yellow == current_state && speed_timer <= WAIT_BEFORE_SPEEDSTOP) {}//wait for speed_yellow pin to change. If it does not change, the loop will exit when speed_timer = WAIT_BEFORE_SPEEDSTOP
        }
        if (speed_timer < WAIT_BEFORE_SPEEDSTOP)
         {
            end = speed_timer.read(); //time at the end of timing NUM_HALF_CYCLES cycles
            speed_timer.stop();
            speed =((60.0/16)*NUM_HALF_CYCLES)/(end-begin); //record speed in rev/min
         } 
        else 
        {
            speed = 0; //speed = 0 if the timer has exceeded WAIT_BEFORE_SPEEDSTOP
        }
    }
     else 
    {
        speed = 0;  //speed = 0 if the timer has exceeded WAIT_BEFORE_SPEEDSTOP
    }
}