/******************************************************************************
***********************           Protodrive            ***********************
*************     Andrew Botelho, William Price, Harsh Jain       *************
******************************************************************************/

#include "mbed.h"
#include "SerialRPCInterface.h"
#include "RPCVariable.h"

/***********************************************************/
/*Constants                                                */
/***********************************************************/
#define MATLAB_INTERFACE //matlab used as an interface
//#define SERIAL_INTERFACE //Terminal used as an interface. Mainly for debugging

#define CURRENT_SENSOR_ON //turn on current sensor
//#define DEBUG
//#define LIGHTS_CURRENT_CONTROL //power flow light control is determined by the current sensor
#define LIGHTS_LOAD_CONTROL //power flow light control is determined by the voltage demand placed on the motors

/**********************Power sources options***************/
/*Instruction: There are 3 power source options for the DUT
and two for the load. Uncomment the option that you want to
use and comment options that you do not want to use. The DUT
and load should each only have one option uncommented.
*/

//DUT
//1. External supply
//#define DUT_EXTERNAL_POWER_ENABLE

//2. Just DUT batt enabled
//#define DUT_BATT_ENABLE

//3. DUT batt and cap enabled
#define DUT_BATT_CAP_ENABLE

//Load
//1. External Supply
//#define LOAD_EXTERNAL_POWER_ENABLE

//2. Load batt enabled
#define LOAD_BATT_ENABLE

/***********************************************************/
/*Global Variables                                         */
/***********************************************************/
float v_DUT_batt, v_load_batt, v_cap,  speed, current, current_cap;
int big_error,regen;
int M1, M2, M3,M4,Cap_charge;
int foo = 1;

//Initial setups
float DUT_input = 0; //DUT duty cycle demand variable
float load_input = 0; //Load duty cycle demand variable

/***********************************************************/
/*Pin Setup                                                */
/***********************************************************/
//mbed LEDS
DigitalOut big_error_led(LED1); //error LED
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);

//Board LEDs
/*
DigitalOut leftArrow(p29); //setting high turns on green left arrow LEDS
DigitalOut greenLine(p28); //setting high turns on green centerline LEDS
DigitalOut orangeLine(p27); //setting high turns on orange centerline LEDS
DigitalOut rightArrow(p26); //setting high turns on orange right arrow LEDS
DigitalOut load_batt_green(p25); //setting high will turn on the load batt green LED
DigitalOut load_batt_orange(p24); //setting high will turn on the load batt orange LED
DigitalOut DUT_batt_green(p10); //setting high will turn on the DUT batt green LED
DigitalOut DUT_batt_orange(p11); //setting high will turn on the DUT batt orange LED

DigitalOut cap_green(p12); //setting high will turn on the cap green LED
DigitalOut cap_orange(p13); //setting high will turn on the cap orange LED
*/

#ifdef CURRENT_SENSOR_ON
AnalogIn current_sense(p15); //current sensor on p15
AnalogIn current_sense_cap(p16); //current sensor on p15
#else
DigitalOut place_holder15(p15); //sets p15 as a DigitalOut that will be set low, to reduce noise on analog in pins
DigitalOut place_holder20(p16); //sets p16 as a DigitalOut that will be set low, to reduce noise on analog in pins
#endif

//sets p16 and p18 as a DigitalOut that will be set low, to reduce noise on analog in pins
DigitalOut place_holder18(p18);

#ifdef LOAD_BATT_ENABLE
AnalogIn v_load_batt_measure_pin(p17); //Analog in from load battery
#endif

#ifdef DUT_BATT_ENABLE
AnalogIn v_DUT_batt_measure_pin(p20); //Analog in from drive battery
DigitalOut place_holder18(p19); //sets p19 as a DigitalOut that will be set low, to reduce noise on analog in pins
#endif

#ifdef DUT_BATT_CAP_ENABLE
AnalogIn v_DUT_batt_measure_pin(p20); //Analog in from drive battery
AnalogIn v_cap_measure_pin(p19); //Analog in from super cap
#endif

DigitalIn kill_switch(p9); //switch to disable running program

//matlab interface
#ifdef MATLAB_INTERFACE
//tie program variables to RPC variables used in Matlab
RPCVariable<float> RPC_DUT_input(&DUT_input, "DUT_input");
RPCVariable<float> RPC_load_input(&load_input, "load_input");
//RPCVariable<float> RPC_DUT_input_direc(&DUT_input_direc, "DUT_input_direc");
//RPCVariable<float> RPC_load_input_direc(&load_input_direc,"load_input_direc");
RPCVariable<float> RPC_current(&current, "current");
RPCVariable<float> RPC_current_cap(&current_cap, "current_cap");
RPCVariable<float> RPC_speed(&speed, "speed");
RPCVariable<float> RPC_v_DUT_batt(&v_DUT_batt, "v_DUT_batt");
RPCVariable<float> RPC_v_load_batt(&v_DUT_batt, "v_load_batt");
RPCVariable<float> RPC_v_cap(&v_DUT_batt, "v_cap");
#endif

/***********************************************************/
/*Terminals                                                */
/***********************************************************/

#ifdef MATLAB_INTERFACE
SerialRPCInterface SerialInterface(USBTX, USBRX);//establich RPC serial connection
#endif

#ifdef SERIAL_INTERFACE
Serial pc(USBTX, USBRX); //Establish serial
#endif

/***********************************************************/
/*Other Includes                                           */
/***********************************************************/

#include "power_sources.h"
#include "measurement.h"
#include "motor_control.h"

/***********************************************************/
/*Subroutines                                              */
/***********************************************************/
/*
//LED control
void all_LEDS_flash()
 {
    //a demo program that just switches LEDs between green and orange

    if (foo == 0)
    {

        leftArrow = 0;
        greenLine = 0;
        orangeLine = 1;
        rightArrow = 1;
        load_batt_green = 0;
        load_batt_orange = 1;
        DUT_batt_green = 0;
        DUT_batt_orange = 1;
        cap_green = 0;
        cap_orange = 1;
        foo = 1;
    }
    else
    {
        leftArrow = 1;
        greenLine = 1;
        orangeLine = 0;
        rightArrow = 0;
        load_batt_green = 1;
        load_batt_orange = 0;
        DUT_batt_green = 1;
        DUT_batt_orange = 0;
        cap_green = 1;
        cap_orange = 0;
        foo = 0;
    }
}

//turn all LEDs off
void all_LEDS_off()
{
    load_batt_green = 1;
    load_batt_orange = 1;
    leftArrow = 1;
    greenLine = 1;
    orangeLine = 1;
    rightArrow = 1;
    DUT_batt_green = 1;
    DUT_batt_orange = 1;
    cap_green = 1;
    cap_orange = 1;
}

//turns on the green LEDs indicating regen
void regen_LEDS() {
    load_batt_green = 1;
    load_batt_orange = 0;
    leftArrow = 0;
    greenLine = 0;
    orangeLine = 1;
    rightArrow = 1;
    if (!relay) {
        DUT_batt_green = 0;
        DUT_batt_orange = 1;
        cap_green = 1;
        cap_orange = 1;
    }
    else {
        DUT_batt_green = 1;
        DUT_batt_orange = 1;
        cap_green = 0;
        cap_orange = 1;
    }
}

//turns on the orange LEDs indicating drive mode (power flowing out of the vehicle)
void drive_LEDS()
{
    load_batt_green = 0;
    load_batt_orange = 1;
    leftArrow = 1;
    greenLine = 1;
    orangeLine = 0;
    rightArrow = 0;
    if (!relay)
    {
        DUT_batt_green = 1;
        DUT_batt_orange = 0;
        cap_green = 1;
        cap_orange = 1;
    }
     else
     {
        DUT_batt_green = 1;
        DUT_batt_orange = 1;
        cap_green = 1;
        cap_orange = 0;
    }
}
*/

/***********************************************************/
/*Main program setup                                       */
/***********************************************************/

int main()
{

    M1 = 0;
    M2 = 1;
    M3 = 0;
    Cap_charge = 0;
    
    Timer timer; //create a timer for the main loop
    init_pwm(); //initialize PWM

    cap_control_pin = 1;
    bat_control_pin = 1;
    recharge_cap_control_pin = 1;
//------------------------------------------------------------------------

    /***********************************************************/
    /*Main Loop                                                */
    /***********************************************************/
    while (1) {
        while (kill_switch == 0) {
            //run demo while waiting for kill switch to be turned on
            //all_LEDS_flash(); //switch LEDs between green and orange
            wait(.5); //wait 0.5s

        }

        // System initializations
        //DUT_direction.write(1); //set forward
        //load_direction.write(0); //set reverse. This ensures both motors will apply a torque on the coupler in the same direction

        //initialize system variables to 0
        speed = 0;
        current = 0;
        current_cap = 0;
        big_error = 0;
        big_error_led = 0;

        //main loop timer variables
        int slow_timer = 0;
        int fast_timer = 0;

        timer.start(); //start loop timer
        //all_LEDS_off(); //turn off LEDs
        wait(0.5); //wait 2s before starting

#ifdef DUT_BATT_ENABLE
        place_holder18 = 0; //set DigitalOut pin low to reduce noise on adjacent AnalogIn pins
#endif

        //set unused Analog pins to ground
#ifndef CURRENT_SENSOR_ON
        place_holder18 = 0; //set DigitalOut pin low to reduce noise on adjacent AnalogIn pins
#endif

        while (kill_switch == 1 && big_error == 0) {
            //while the kill switch is on and there are no errors

            // --- SLOW LOOP (1) Hz ---
            if (timer.read_ms() - slow_timer >= 1000) {
                slow_timer = timer.read_ms(); //read timer value in miliseconds
                big_error = checkBattVoltages(); //check batt voltages to ensure they are in the correct voltage range
                if (big_error == 1) {
                    big_error_led = 1;
                }

                //Measure voltages
#ifdef LOAD_BATT_ENABLE
                v_load_batt = get_voltage(v_load_batt_measure_pin); //measure load batt voltage
#ifdef SERIAL_INTERFACE
                pc.printf("Analog In (p17): %f ", v_load_batt); //print load voltage to terminal
#endif
#endif

#ifdef DUT_BATT_ENABLE
                v_DUT_batt = get_voltage(v_DUT_batt_measure_pin); //measure DUT batt voltage
#ifdef SERIAL_INTERFACE
                pc.printf("Analog In (p15): %f ", current_read); //print DUT voltage to terminal //v_DUT_batt
                //pc.printf("Analog In (p16): %f ", current_read_load); //print DUT voltage to terminal //v_DUT_batt
#endif
#endif

#ifdef DUT_BATT_CAP_ENABLE
                v_cap = get_voltage(v_cap_measure_pin); //measure cap voltage
                v_DUT_batt = get_voltage(v_DUT_batt_measure_pin); //measure DUT batt voltage
#ifdef SERIAL_INTERFACE
                pc.printf("Analog In (p19): %f Analog In (p20): %f Speed: %f Current: %f\r\n", v_cap, v_DUT_batt, speed, current); //print cap voltage, DUT batt voltage, speed, current to terminal
                //pc.printf("Analog In (p19): %f Analog In (p20): %f Speed: %f Current: %f\r\n", current_load); //print cap voltage, DUT batt voltage, speed, current to terminal
#endif
#endif
            }

            // --- FAST LOOP (100 Hz) ---
            if (timer.read_ms() - fast_timer >= 10) {
                fast_timer = timer.read_ms(); //read time in ms
                set_duty(DUT_input,load_input); //set the duty cycle
                get_speed(); //update motor speed

#ifdef CURRENT_SENSOR_ON
                get_current(); //update current
#endif

#ifdef LIGHTS_LOAD_CONTROL //controls the board LEDs based on the value of the DUT and Load motor duty cycle
                //turn on green LEDs if in regen mode, and orange LEDs if in drive mode.
                if ((DUT_input >= load_input) && (DUT_input != 0 || load_input != 0)) {
                    //check if not in regen mode
                    regen = 0; //set regen flag low
                    //drive_LEDS(); //turn on orange LEDs
                } else if ((DUT_input < load_input) && (DUT_input != 0 || load_input != 0)) {
                    //check if in regen mode
                    regen = 1; //set regen flag high
                    //regen_LEDS(); //turn on green LEDs
                } else {
                    // DUT is niether flowing into or out of DUT
                    regen = 0; //set regen flag low
                    //all_LEDS_off();// LEDs off
                }
#endif

#ifdef LIGHTS_CURRENT_CONTROL  //controls the board LEDs based on the value of the DUT and Load motor currents
                //turn on green LEDs if in regen mode, and orange LEDs if in drive mode.
                if (current_DUT > 0) {
                    regen = 0; //set regen flag low
                    //drive_LEDS(); //turn on orange LEDs
                } else if (current_DUT < 0) {
                    regen = 1; //set regen flag high
                    //regen_LEDS(); //turn on green LEDs
                } else {
                    regen = 0; //set regen flag low
                    //all_LEDS_off(); // LEDs off
                }
#endif

#ifdef DUT_BATT_CAP_ENABLE
                superCapControl();
#endif
            }

            // reset timer if 1000 seconds have elapsed
            if (timer.read() >= 1000) {
                timer.reset();
                slow_timer = 0;
                fast_timer = 0;
            }
        }
        //turn off motors
        load_motor = 0;
        DUT_bat_motor = 0;
        DUT_sc_motor = 0;
        //DUT_recharge_sc_motor = 0;
        cap_control_pin = 1;
        bat_control_pin = 1;
        recharge_cap_control_pin = 1;

        timer.stop();
        timer.reset();
    }

//---------------------------------------------------------
}
