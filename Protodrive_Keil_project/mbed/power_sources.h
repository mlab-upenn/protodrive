/*****************************************************************************/
/*Constants                                                                  */
/*****************************************************************************/

#define SCAP_MIN_VOLTAGE 0.40      //Switch to batt below this value (supercap)     
#define SCAP_SWITCHING   0.65
#define SCAP_MAX_VOLTAGE 0.82      //Switch to batt above this value (supercap)    15V
#define BATT_MIN_VOLTAGE 0.70     //Shutoff below this value (battery) 
#define BATT_MAX_VOLTAGE 0.95

/*****************************************************************************/
/*Subroutines                                                                */
/*****************************************************************************/

//Checks battery and supercap voltages, returning 1 if the system needs to be shut down.
int checkBattVoltages()
{
    //Check to see if batteries are out of proper voltage range
#ifdef DUT_BATT_ENABLE
    //check DUT batt
    if ((v_DUT_batt_measure_pin < BATT_MIN_VOLTAGE) || (v_DUT_batt_measure_pin > BATT_MAX_VOLTAGE)) {
        //check if voltage out of range
        return 1;
    }
#endif

#ifdef DUT_BATT_CAP_ENABLE
    //check DUT batt
    if ((v_DUT_batt_measure_pin < BATT_MIN_VOLTAGE) || (v_DUT_batt_measure_pin > BATT_MAX_VOLTAGE)) {
        return 1;
    }

#endif

#ifdef LOAD_BATT_ENABLE
    //check load batt
    if ((v_load_batt_measure_pin < BATT_MIN_VOLTAGE) || ( v_load_batt_measure_pin > BATT_MAX_VOLTAGE)) {
        //check if voltage out of range
        return 1;
    }
#endif
    return 0; //return 0 if all batts are in the correct voltage range
}

#ifdef DUT_BATT_CAP_ENABLE
void superCapControl()
{
    /*Naive Case*/
    /*Discharging - If you have charge in the capacitor use it and then switch it to batteries
      Recharging - If capacitor is depleted and if there is regen, recharge the cap and then
                   recharge the batteries.
      Check for the load duty to be applied when the battery voltage is increased.
    */

    //Discharging Case
    if ((v_cap_measure_pin <= SCAP_MIN_VOLTAGE) && !regen) {  //In this case battery will charge the SC if SC is not full
        //if cap is drained, and regen is not engaged, switch back to battery. Or if cap is at its max voltage, and regen is engaged, switch to batt to prevent overcharging of cap.
        M1 = 1; //switch to battery
        M2 = 0; //switch to cap
        M3 = 1;
        Cap_charge = 1;
    }
    else if((v_cap_measure_pin >= SCAP_SWITCHING) && !regen) { //In this case battery will stop charging the SC if SC is full
        M1 = 0; //switch to battery
        M2 = 1;
        M3 = 0; //stop charging the cap
        Cap_charge = 0;
    }
    else if((v_cap_measure_pin > SCAP_MIN_VOLTAGE) && (v_cap_measure_pin <= SCAP_SWITCHING) && !regen) {
        if(Cap_charge == 0) {
            M1 = 0;
            M2 = 1;
            M3 = 0;
        } else {
            M1 = 1; //switch to battery
            M2 = 0; //switch to cap
            M3 = 1;
        }
    }
    //Super Cap Charging Case
    else if (v_cap_measure_pin <= SCAP_MAX_VOLTAGE && regen) { // || (regen && v_cap_measure_pin <= SCAP_MIN_VOLTAGE))
        M1 = 0;
        M2 = 0;
        M3 = 1;
    }
}
#endif