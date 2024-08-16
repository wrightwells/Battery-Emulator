#include "../include.h"
#ifdef ZERO_14_4_BATTERY
#include "ZERO-14-4-BATTERY.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"


/*----------------------*/
// WIKI
// https://github.com/wrightwells/Battery-Emulator/wiki/Zero-Motorcycle-14.4-Battery 
// The CAN messages have been taken from these repos, standing on the shoulders and all that.
//-- https://github.com/RIAEvangelist/zero-motorcycle-canbus for CAN messages
/*----------------------*/
// Done: 
//-- SOC_Display
//-- batteryAmps
//-- temperatureMax
//-- batteryVoltage

//-- Added entries in the follwing files for the ZERO_14_4_BATTERY
//-- Updated BATTRIES.h 
//-- Updated USER_SETTINGS.h 
//-- Updated webserver.cpp
/*----------------------*/
// TODO:
//-- Status where to find it in the sea of CAN messages
//-- Cell data , it must be in the CAN logs somewhere
//-- how to calculate SOH
//-- allowedDischargePower investigate and set the value
//-- allowedChargePower
/*----------------------*/


/*-------------------UPDATE the USER_SETTINGS.h------------------*/
// Update Battery settings in USER_SETTINGS.h 
// Predefined total energy capacity of the battery in Watt-hours
// change the #define BATTERY_WH_MAX 10600
// Zero Motorcycles call thier battery a 14.4Kw but thats not the nominal value , its more like 104 Amp Hours x 102 Volts (104*102)= 10608 or 10.6kw
// which is accually why they give such low range for the advertised battery size.
// but when it can be used as a house battery when not being ridden , whoop whoop
/*---------------------------------------------------------------*/

/* Do not change code below unless you are sure what you are doing */
static unsigned long previousMillis10 = 0;   // will store last time a 10ms CAN Message was send
static unsigned long previousMillis100 = 0;  // will store last time a 100ms CAN Message was send
static unsigned long previousMillis10s = 0;  // will store last time a 1s CAN Message was send
static uint8_t mprun10r = 0;                 //counter 0-20 for 0x1F2 message
static uint8_t mprun10 = 0;                  //counter 0-3
static uint8_t mprun100 = 0;                 //counter 0-3


// //open contactor.
//openContactor = {0x0, 0x31, 0x4, 0xc3, 0x4D, 0x00, 0x0, 0x0};
CAN_frame ZERO_81 = {.FD = false,
                      .ext_ID = false,
                      .DLC = 8,
                      .ID = 0x81,
                      .data = {0x0, 0x31, 0x4, 0xc3, 0x4D, 0x00, 0x0, 0x0}};


static bool battery_can_alive = false;
static uint16_t SOC_Display = 0;
static uint16_t batterySOH = 100;
static uint16_t CellVoltMax_mV = 3700;
static uint16_t CellVoltMin_mV = 3700;
static uint8_t CellVmaxNo = 0;
static uint8_t CellVminNo = 0;
static uint16_t allowedDischargePower = 0;
static uint16_t allowedChargePower = 0;
static uint16_t batteryVoltage = 0;
static int16_t leadAcidBatteryVoltage = 120;
static int8_t temperatureMax = 0;
static int8_t temperatureMin = 0;
static int16_t batteryAmps = 0;
static uint8_t counter_200 = 0;
static uint8_t checksum_200 = 0;
static uint8_t StatusBattery = 0;
static uint16_t cellvoltages_mv[96];
const float scaleFactor = 0.001; // raw data is in millivolts


void update_values_battery() {  //This function maps all the values fetched via CAN to the correct parameters used for modbus

  datalayer.battery.status.real_soc = (SOC_Display * 10);  //increase SOC range from 0-100.0 -> 100.00

  datalayer.battery.status.soh_pptt = (batterySOH * 100);  //Increase decimals from 100% -> 100.00%

  datalayer.battery.status.voltage_dV = batteryVoltage;

  datalayer.battery.status.current_dA = -batteryAmps;

  datalayer.battery.status.remaining_capacity_Wh = static_cast<uint32_t>(
      (static_cast<double>(datalayer.battery.status.real_soc) / 10000) * datalayer.battery.info.total_capacity_Wh);

  datalayer.battery.status.max_discharge_power_W = allowedDischargePower * 10;

  datalayer.battery.status.max_charge_power_W = allowedChargePower * 10;

  //Power in watts, Negative = charging batt
  datalayer.battery.status.active_power_W =
      ((datalayer.battery.status.voltage_dV * datalayer.battery.status.current_dA) / 100);

  datalayer.battery.status.cell_max_voltage_mV = CellVoltMax_mV;

  datalayer.battery.status.cell_min_voltage_mV = CellVoltMin_mV;

  datalayer.battery.status.temperature_min_dC = temperatureMin * 10;  //Increase decimals, 17C -> 17.0C

  datalayer.battery.status.temperature_max_dC = temperatureMax * 10;  //Increase decimals, 18C -> 18.0C

  if (leadAcidBatteryVoltage < 110) {
    set_event(EVENT_12V_LOW, leadAcidBatteryVoltage);
  }

#ifdef DEBUG_VIA_USB
  Serial.println("Values going to inverter:");
  Serial.print("SOH%: ");
  Serial.print(datalayer.battery.status.soh_pptt);
  Serial.print(", SOC% scaled: ");
  Serial.print(datalayer.battery.status.reported_soc);
  Serial.print(", Voltage: ");
  Serial.print(datalayer.battery.status.voltage_dV);
  Serial.print(", Max discharge power: ");
  Serial.print(datalayer.battery.status.max_discharge_power_W);
  Serial.print(", Max charge power: ");
  Serial.print(datalayer.battery.status.max_charge_power_W);
  Serial.print(", Max temp: ");
  Serial.print(datalayer.battery.status.temperature_max_dC);
  Serial.print(", Min temp: ");
  Serial.print(datalayer.battery.status.temperature_min_dC);
  Serial.print(", BMS Status (3=OK): ");
  Serial.print(datalayer.battery.status.bms_status);

#endif

}

void receive_can_battery(CAN_frame rx_frame) {

  switch (rx_frame.ID) {
 

// ***************************
    // canIDs with several values
    // ***************************
    // 0x0080                       // this canId has no databytes but signals the start of a new "block of data"
    
        // no databytes
   case 0x0080:  
        battery_can_alive = true;
        datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;  // Let system know battery is sending CAN
        break;
//0x0181

        // ?? byte 0 (0)
        // ?? byte 1 (0)
        // ?? byte 2 (0)
        // ?? byte 3 (0)
        // ?? byte 4 (byte 4..5 very eratic) look something that balances between 0..40 and 65535..65524 so plus minus someting)
        // ?? byte 5
        // ?? byte 6 (byte 6..7 very eratic) look something that balances between 0..40 and 65535..65524 so plus minus someting)
        // ?? byte 7

//0x0188

        // byte 0 (always 90)
        // ?? byte 1 (always 0)
        // ?? byte 2 (always 66, but 3 while charging in another session)
        // byte 3 + 4 is number of charge cycles (byte 4 as well (highbyte)
        // byte 5 is be Cel Balance value
        // ?? byte 6 (always 0)
        // ?? byte 7 (always 4 maybe number of bricks ?)

//0x0192

        // ?? byte 0 fixed at 128 during charging
        // ?? byte 1 fixed at 0 during charging
        // ?? byte 2 byte 2,3 and 4 look like charging voltage (byte 2 very erratic, 
        // ?? byte 3 164
        // ??  byte 4 at 1
        // ?? byte 5 fixed at 0 during charging
        // ?? byte 6 very eratic
        // ?? byte 7 very eratic

//0x01C0
    
        // ?? byte 0 (fixed at 32)
        // ?? byte 1 (fixed at 0)
        // byte 2 is Dash Time Hours
        // byte 3 is Dash Time Minutes
        // byte 4 is Dash Time Seconds
        // byte 5 - 7 is number of canbus message SETS received since power on ??

// 0x0206                                                                
// this canID only appears during charging !!

        // ?? byte 0 (second temperature sensor in the batterypack temperature ??) fixed at 18 during charging 30-4-2019
        // ?? byte 1 (fixed at 2 during charging) also later during charging 30-4-2019
        // ?? byte 2 (fixed at 0 during charging) also later during charging 30-4-2019
        // byte 2-3-4 Charger voltage in mV (division by 1000 gets Volts) could be charger voltage at battery because ID0192 byte 2-3-4 is slightly higher
        // ?? byte 6 (fixed at 0)
        // ?? byte 7 (fixed at 0)

//0x240
        // CanID 0x0240 byte 0 = Mode
        // if (bitRead(buf[0], 2) == 1) Mode = "Sport ";        //A5                                        
        //if (bitRead(buf[0], 3) == 1) Mode = "Eco   ";       // A9
        //if (bitRead(buf[0], 4) == 1) Mode = "Custom";    // B1
        // byte 1 = 0 during charging and 15 during run
        // byte 2 and 3 is Speed in kph
        // ?? byte 4 is Power bar on dash ?? (It is power related)
        // ?? byte 5 is Torque bar on dash ?? (It is Torque related)
        // byte 6 is State Of Charge on dash in %
        // no Byte 7


    case 0x240:
        SOC_Display = static_cast<int16_t>(rx_frame.data.u8[6]); //SOC
        
        #ifdef DEBUG_VIA_USB
            Serial.print("SOC=  ");
            Serial.print(rx_frame.data.u8[6]);
            Serial.println("");
        #endif
        
    break;

//0x0281
        // ?? byte 0, 1 is motor RPM (same as canID 0x0340 byte 4 and 5)
        // ?? byte 2 (fixed at 0)
        // ?? byte 3 (fixed at 0)
        // ?? byte 4, 5 is throttle position (around 125 is closed, aroud 1200 is fully open)
        // ?? byte 6 is motor temperature (second time) this time in 1 byte , not 2 as in canid 440
        // ?? byte 7 (fixed at 0)

// 0x0288

// byte 0, 1 Power in Watts
        // ?? byte 2 (fixed at 231) byte 2 .. 7 the same during charging several weeks later
        // ?? byte 3 (fixed at 0)
        // ?? byte 4 (fixed at 50)
        // byte 5 is the rported Ah of the pack (fixed at 114) This id odd because for a 14.4 kWh bike the number should be 124 !
        // ?? byte 6 (fixed at 0)
        // ?? byte 7 (fixed at 18 not any temperature))

//0x0292

        //memcpy(canID292, buf, 8);
        // ?? byte 0 (176 during charging) (byte 0..2 give 116399 (battery cut-off???) all values identical during charging several weeks later (30-4-2019)
        // ?? byte 1 (198 during charging)
        // ?? byte 2 (1 during charging)
        // ?? byte 3 (0 during charging)
        // ?? byte 4 (255 during charging)
        // ?? byte 5 (255 during charging)
        // ?? byte 6 ( 0 during charging)
        // ?? byte 7 (0 during charging)
    
//0x02C0

    // byte 0-5 are the same as 3C0 but updates only on standstill. looks like a ODO acknowledge
    // byte 0, 1 and 2 are ODO in units of 100 mtrs (divided by 10 to get KM)
        // ?? byte 3 (fixed at 0)
        // ?? byte 4 (fixed at 0)
        // ?? byte 5 (fixed at 0)
        // ?? byte 6 (fixed at 16)
        // ?? byte 7 (fixed at 0)

//0x0306

        // ?? byte 0 (18/19 during charging) could be a temperature of sme sort
        // ?? byte 1 (176 during charging) 2 during charging several weeks later 30-42-019
        // ?? byte 2 (198 during charging) 198 during charging several weeks later 30-4-2019
        // ?? byte 3 (1 during charging) (byte 1..3 give 116399)
        // ?? byte 4 (0 during charging)
        // ?? byte 5 (255 during charging)
        // ?? byte 6 (255 during charging)
        // ?? byte 7 0 during charging)

//0x0308

        // ?? byte 0 (fixed at 34 during standstill and during charging
        // ?? byte 1 (fixed at 2 during standstill at 99% SOC)
        // ?? byte 2 (fixed at 95) fixed at 131 during charging several weeks later
        // ?? byte 3 (fixed at 0)
        // ?? byte 4 (fixed at 101) 168 during charging several weeks later (71% SOC)
        // ?? byte 5 (fixed at 245) 207 during charging several weeks later (71% SOC)
        // ?? byte 6 (fixed at 3) 5 during charging several weeks later
        // ?? byte 7 (fixed at 0)

//0x340

        // byte 0, 1 is Trip 1
        // ?? byte 2 (always 0) byte 2,3 and 7 the same during charging several weeks later
        // ?? byte 3 (always 0)
        // byte 4, 5 is motor RPM
        // byte 6 (error code on dash, 44 is killswitch, 45 is kickstand etc)
        // ?? byte 7 (always 0)

//0x0381

        // ?? byte 0 (95/96 during charging) 177-179 during charging several weeks later
        // ?? byte 1 (6 during charging, also several weeks later)
        // ?? byte 3 (always 0 during charging)
        // ?? byte 4 (always 0 during charging)
        // ?? byte 6 (100..102 during charging) 183..184 during charging several weeks later
        // ?? byte 7 (always 6 during charging) also 6 several weeks later during charging
    
//0x0388

        // ?? byte 0 (very random ?? 0-25 when speed = 0 , goes higher/lower when powering or regen)
        // ?? byte 1 (very random ?? 246-248 when speed = 0, goes higher/lower when powering or regen)
        // ?? byte 2 (always 15, later it was 14 when charging)
        // byte 3, 4 and 5 are Vbatt in units of mV (divide by 1000 to get Volts)
        // ?? byte 6 (always zero, including charging)
        // ?? byte 7 (increases with torque/power and is negative with regen : could be power/torque/battery or motor amps, around 245,246 during charging, 248 249 during charging several weeks later


	case 0x0388: {
		uint32_t voltage = (rx_frame.data.u8[3] << 16) | 
						(rx_frame.data.u8[4] << 8)  | 
						(rx_frame.data.u8[5]);

		batteryVoltage = static_cast<uint16_t>(voltage * scaleFactor);

		#ifdef DEBUG_VIA_USB
			Serial.print("Voltage = ");
			Serial.print(batteryVoltage); 
			Serial.println("");
		#endif
	}
	break;



//0x03C0

        // byte 0, 1 and 2 are ODO in units of 100 mtrs (divided by 10 to get KM)
        // ?? byte 3 (0)
        // ?? byte 4 (0)
        // ?? byte 5 (0)
        // no byte 6,7


//0x0406    // occasionally 18  16  39  41  29 156 241   2

        // ?? byte 0 (18/19 during charging) same several weeks later during charging
        // ?? byte 1 (16 during charging) same several weeks later during charging
        // ?? byte 2 (39 during charging) same several weeks later during charging
        // ?? byte 3 1 during charging)
        // ?? byte 4 (0 during charging)
        // ?? byte 5 (255 during charging)
        // ?? byte 6 (255 during charging)
        // ?? byte 7 (0 during charging)
    
//0x0408

        // byte 0 (always 0 during charging)
        // byte 1 is battery temp in celcius
        // byte 2 (always 20 during charging)
        // byte 3 battery amps, both when charging and
        // byte 5 seems to be the current number of Ah in the battery (fixed at 131, 66 during charging, later 118)
        // byte 6 (fixed at 0)
        // byte 7 (fixed at 255)

    case 0x0408:
        
        batteryAmps = (rx_frame.data.u8[3]); // AMPS
        
        #ifdef DEBUG_VIA_USB
            Serial.print("AMPS=  ");
            Serial.print(batteryAmps);
            Serial.println("");
        #endif      
        
        temperatureMax = (rx_frame.data.u8[1]); //TEMPERATURE

        #ifdef DEBUG_VIA_USB
            Serial.print("TEMPERATURE=  ");
            Serial.print(temperatureMax);
            Serial.println("");
        #endif
        
    break;


//0x0440

        // byte 0, 1,2 is Trip 2 in units of 10 mtrs (divided by 100 to get KM) maybe byte 3 as well
        // byte 3 (fixed at 0)
        // byte 4, 5 is range in units of 10 mtrs (divided by 100 to get KM)
        // byte 6, 7 is motortemperature in 0.01 degrees but only 2200, 2300, 2400 (divsion by 100 is immediate integer value)
    
  //0x0481

        // ?? byte 0 (0 during charging) same several weeks later during charging
        // ?? byte 1 (0) same several weeks later during charging
        // ?? byte 2 (226) same several weeks later during charging
        // ?? byte 3 (2) same several weeks later during charging
        // ?? byte 4 (0)  // byte 4-7 are fixed 0 at standstill but when rear wheel is moved values go negative (4+5 seem to be a pair and 6+7 as well)
        // ?? byte 5 (0)
        // ?? byte 6 (0)
        // ?? byte 7 (0)
    
//0x0488

        // ?? byte 0 (0-7 during charging)
        // ?? byte 1 22-25 during charging (battery temperature ??)
        // ?? byte 2 100-213 during charging
        // ?? byte 3 counts up and down during charging
        // ?? byte 4 counts up and down during charging
        // ?? byte 5 fixed at 0 during charging
        // ?? byte 6 fixed at 24-31 during charging (charger or battery temperature ??)
        // ?? byte 7 fixed at 15 during charging
    
// 0x0501

        // ?? byte 0 (0 during charging) byte 0 and 1 seem a pair, when regen value of byte 1 goes to 255
        // ?? byte 1 (0) same several weeks later during charging
        // ?? byte 2 (178)same several weeks later during charging
        // ?? byte 3 (2)same several weeks later during charging
        // ?? byte 4 (147) same several weeks later during charging
        // ?? byte 5 (2) same several weeks later during charging
        // ?? byte 6 (0) same several weeks later during charging
        // ?? byte 7 (116) same several weeks later during charging
    
//0x0506)

        // ?? byte 0 (8 during charging)
        // ?? byte 1 (168 during charging)
        // ?? byte 2 (65 during charging)
        // ?? byte 3 (1 during charging)
        // ?? byte 4 (18 during charging)
        // ?? byte 5 (0 during charging)
        // ?? byte 6 (113/114 during charging)
        // ?? byte 7 (0 during charging)
    
//0x0508

        // ?? byte 0,1 ,2 are number of seconds used or maybe even lifetime possibly bytes 3 and on also included (at feb-2019 bytes are (xxx,223,114,92,10,0,75,0) on apr 30 2019 (56,132,200,92,10,0,75,0)
        // ?? byte 1 (204) on 21-3-2019_
        // ?? byte 2 (147)
        // ?? byte 3 (92)
        // ?? byte 4 (10)
        // ?? byte 5 (0)
        // ?? byte 6 (75)
        // ?? byte 7 (0)

//0x0540

         // ?? byte 0 minutes to go for charging (possibly also byte 1 for values > 256 minutes


//0x0588

        // ?? byte 0 (various values, but only just at startup)
        // ?? byte 1 (various values, but only just at startup)
        // ?? byte 2 (0)
        // ?? byte 3 (0)
        // ?? byte 4 (0)
        // ?? byte 5 (0)
        // ?? byte 6 (0)
        // ?? byte 7 (0)

//0x0608

    //bytes 0 and 1 give a value at startup but not after
        // ?? byte 0 (various values, but only just at startup)
        // ?? byte 1 (various values, but only just at startup)
        // ?? byte 2 (0)
        // ?? byte 3 (0)
        // ?? byte 4 (0)
        // ?? byte 5 (0)
        // ?? byte 6 (0)
        // ?? byte 7 (0)

//0x0701

        // ?? byte 0 only 1 byte long always 5, acknowledge or closure of data messages
        // no byte 1-7

    
  }
}

void send_can_battery() {
    if (battery_can_alive) {
        unsigned long currentMillis = millis();
        //Send 10ms message
        if (currentMillis - previousMillis10 >= INTERVAL_10_MS) {
            // Check if sending of CAN messages has been delayed too much.
            if ((currentMillis - previousMillis10 >= INTERVAL_10_MS_DELAYED) && (currentMillis > BOOTUP_TIME)) {
            set_event(EVENT_CAN_OVERRUN, (currentMillis - previousMillis10));
            } else {
            clear_event(EVENT_CAN_OVERRUN);
            }
            previousMillis10 = currentMillis;

            //ESP32Can.CANWriteFrame(&ZERO_14_4_81); //don't want to write to the battery yet, not sure when its needed
        }
        // Send 100ms CAN Message
        if (currentMillis - previousMillis100 >= INTERVAL_100_MS) {
            previousMillis100 = currentMillis;

            //ESP32Can.CANWriteFrame(&ZERO-14.4_100);
        }
    }
}

void setup_battery(void) {  // Performs one time setup at startup
#ifdef DEBUG_VIA_USB
  Serial.println("ZERO 14.4 battery selected");
#endif
  datalayer.battery.info.number_of_cells = 112;
  datalayer.battery.info.max_design_voltage_dV = 1160;  // Over this charging is not possible 114Ah x 116V
  datalayer.battery.info.min_design_voltage_dV = 950;  // Under this discharging is disabled 114Ah x 95V
}
#endif
