#include "../include.h"
#ifdef ZERO_14_4_BATTERY
#include "ZERO-14-4-BATTERY.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"


#ifdef LogToSD
#include "../devboard/utils/LogToSD.h"
#include <FS.h>
char msgString[4];
String logData = "";
/*
#define BUFFER_SIZE 512
char dataBuffer[BUFFER_SIZE];
size_t bufferIndex = 0; 

void addToBuffer(const String &logData);
void flushBufferToSD();
void receive_can_battery();
*/
#endif


/*----------------------*/
// WIKI
// https://github.com/wrightwells/Battery-Emulator/wiki/Zero-Motorcycle-14.4-Battery 
// The CAN messages have been taken from these repos, standing on the shoulders and all that.
//-- https://github.com/RIAEvangelist/zero-motorcycle-canbus for CAN messages
/*----------------------*/
// Done: 
//-- battery_SOC
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
//change the #define BATTERY_MAX_DISCHARGE_AMP 100
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
static uint16_t battery_SOC = 0;
static uint16_t batterySOH = 99;
static uint16_t CellVoltMax_mV = 11600;
static uint16_t CellVoltMin_mV = 9500;
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
const float scaleFactor = 1000; // raw data is in millivolts


void update_values_battery() {  //This function maps all the values fetched via CAN to the correct parameters used for modbus

  datalayer.battery.status.real_soc = (battery_SOC * 100);  //increase SOC range from 0-100.0 -> 100.00

  datalayer.battery.status.soh_pptt = (batterySOH * 100);  //Increase decimals from 100% -> 100.00%

  datalayer.battery.status.voltage_dV = batteryVoltage;

  datalayer.battery.status.current_dA = batteryAmps;

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


#ifdef DEBUG_VIA_USB
  Serial.println("Values going to inverter:");
  Serial.print("SOH%: ");
  Serial.print(datalayer.battery.status.soh_pptt);
  Serial.print(", SOC% scaled: ");
  Serial.print(datalayer.battery.status.real_soc);
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
#ifdef LogToSD
addToBuffer(
    "Values going to inverter: " +
    String("SOH%: ") + String(datalayer.battery.status.soh_pptt) +
    ", SOC% scaled: " + String(datalayer.battery.status.real_soc) +
    ", Voltage: " + String(datalayer.battery.status.voltage_dV) +
    ", Max discharge power: " + String(datalayer.battery.status.max_discharge_power_W) +
    ", Max charge power: " + String(datalayer.battery.status.max_charge_power_W) +
    ", Max temp: " + String(datalayer.battery.status.temperature_max_dC) +
    ", Min temp: " + String(datalayer.battery.status.temperature_min_dC) +
    ", BMS Status (3=OK): " + String(datalayer.battery.status.bms_status)
);
#endif 
}

void receive_can_battery(CAN_frame rx_frame) {
logData = "";
  switch (rx_frame.ID) {
 

// ***************************
    // canIDs with several values
    // ***************************
    // 0x0080                       // this canId has no databytes but signals the start of a new "block of data"
    
        // no databytes
   case 0x0080:  {
        battery_can_alive = true;
        datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;  // Let system know battery is sending CAN
	}
  break;


//Pack Discharge Current    : -13A look for D or -D in CAN



//"DASH_STATUS",0x1C0}
//"CONTROLLER_SPEED_TORQUE",0x181
//"BMS_PACK_STATUS",0x188
//"BMS1_PACK_STATUS",0x189
//"BMS1_PACK_CONFIG",0x289
//ODO_FROM_DASH",0x2C0
//"CONTROLLER_DRIVE_CONTROL",0x205


   case 0x240: {
 
        battery_SOC = static_cast<int16_t>(rx_frame.data.u8[6]); //SOC CORRECT
        //4A = 74
        
                #ifdef DEBUG_VIA_USB
                Serial.print("battery_SOC=  ");
                Serial.print(battery_SOC);
                Serial.println("");
                Serial.println("");		
                #endif
                #ifdef LogToSD
                logData += "240 battery_SOC=  ";
                logData += battery_SOC;
                logData += "\n";
                // Add data to buffer
                addToBuffer(logData);
                #endif
	}
  break;

//"CONTROLLER_RPM_THROTTLE_MOT_TEMP",0x281
//  throttleVoltage.bytes[0]=buf[4];
//  throttleVoltage.bytes[1]=buf[5];


//"BMS_PACK_CONFIG",0x288
/*	case 0x0288:{
                        // Pack size 114A
                       // this give 72 00 which is HEX 72 0 = DEC 114  
                //uint32_t batteryAmps = (static_cast<uint32_t>(rx_frame.data.u8[5])) | 
                //       (static_cast<uint32_t>(rx_frame.data.u8[6]));

        
                #ifdef DEBUG_VIA_USB
                Serial.print("batteryAmps=  ");
                Serial.print(batteryAmps);
                Serial.println("");
                Serial.println("");		
                #endif
                #ifdef LogToSD
                logData += "288 batteryAmps=  ";
                logData += batteryAmps;
                logData += "\n";
                // Add data to buffer
                addToBuffer(logData);
                #endif
	}
  break;
  */

//"DASH_ODO_TO_DASH",0x3C0
//"BMS_PACK_STATS",0x308
//"BMS1_PACK_STATS",0x309


        case 0x340: {

                #ifdef DEBUG_VIA_USB
                Serial.print("ERROR_CODE=  ");
                Serial.print(rx_frame.data.u8[6]);
                Serial.println("");	
                #endif
                #ifdef LogToSD
                logData += "340 ERROR_CODE=  ";
                logData += rx_frame.data.u8[6];
                logData += "\n";
                  // Add data to buffer
                addToBuffer(logData);
                #endif
                //47 the MBB charger is connected but disabled. 
	}
  break;

//"CONTROLLER_TEMP_DIGITAL_IN_18",0x381

//"BMS_CELL_VOLTAGE",0x388
	case 0x0388: {


                //uint16_t voltage = (rx_frame.data.u8[3]) | 
                //   (rx_frame.data.u8[4]) | 
                //   (rx_frame.data.u8[5])  | 
                //   rx_frame.data.u8[6];
                   
                   //388 8 11 05 0F 19 A4 01 00 01 
                   //388 8 1B 07 0F 16 A4 01 00 01 
                   
                   // Vbatt = 0.001 * buf[3] + 0.256 * buf[4] + 65.535 * buf[5]; 
                   //StationaryVolt = Vbatt + BatteryAmps * InternalResistance;


                   //Vbatt = 0.001 * buf[3] = HEX 19 DEC 25 = 0.025
                   // + 0.256 * buf[4] = HEX A4 DEC 164 = 41.984
                   //+ 65.535 * buf[5]; = HEX 01 DEC 1 = 65.535
                   //+ 0  = 107.544

                   //StationaryVolt = Vbatt(107.544) + BatteryAmps(53) * InternalResistance(0.02); // does not help!!


                   //-------------------------

                   //volts.value= zero.voltage(len,buf,canId)*KM_TO_MI/10;      .621371

                   //388 8 11 05 0F 19 A4 01 00 01 
                   //388 8 1B 07 0F 16 A4 01 00 01 

                   //19 A4 01 00
                   //25 164 1 0 

                   //DEC 2516410 

                   //430178560

                   //AC 01 00


                uint16_t voltage = (rx_frame.data.u8[4]) | 
                    (rx_frame.data.u8[5]) | 
                    rx_frame.data.u8[6];
                    // HEX A40100 DEC 10748160 /1000 = 107.4816
                  

              batteryVoltage = static_cast<uint16_t>(voltage) / scaleFactor;

              #ifdef DEBUG_VIA_USB
                Serial.print("batteryVoltage = ");
                Serial.print(batteryVoltage); 
                Serial.println("");
              #endif
                #ifdef LogToSD
                logData += "388 batteryVoltage= ";
                logData += batteryVoltage;
                logData += "\n";
                // Add data to buffer
                addToBuffer(logData);
                #endif
	}
  break;
//"BMS1_CELL_VOLTAGE",0x389


//"BMS_PACK_ACTIVE_DATA",0x408
         case 0x0408:{
          // Battery AH 

          //BatteryTemp = buf[1];   
          //BatteryAmps = buf[3] + 256 * buf[4];  

          //404 batteryAmps=  53
          //408 8 00 14 13 00 00 35 00 FF 
          //408 8 00 17 16 F4 FF 3B 00 FF

          //HEX 00 35
          //DEC 0 53
        
		           batteryAmps = (rx_frame.data.u8[4] | 
                 	rx_frame.data.u8[5]);
 
                #ifdef DEBUG_VIA_USB
                Serial.print("batteryAmps=  ");
                Serial.print(batteryAmps);
                Serial.println("");
                Serial.println("");		
                #endif
                #ifdef LogToSD
                logData += "408 batteryAmps=  ";
                logData += batteryAmps;
                logData += "\n";
                // Add data to buffer
                addToBuffer(logData);
                #endif
	}
  break;

//"BMS1_PACK_ACTIVE_DATA",0x409

//"DASH_STATUS2",0x440}

//"BMS_PACK_TEMP_DATA",0x488
  case 0x0488:{



//  - Lowest Present Pack Temp  :  22 C
//  - Max Pack Temp This Ride   :  23 C
//  - Min Pack Temp This Ride   :  21 C
// look for DEC 23 =  HEX 17
// look for DEC 22 =  HEX 16
// look for DEC 21 =  HEX 15

//also
//- Min Discharge Temp        :  -25 C
// - Max Charge Temp           :  50 C


                temperatureMax = static_cast<int8_t>(rx_frame.data.u8[1]);
                // HEX 12 = DEC 18
                // HEX 13 = DEC 19
                // HEX 15 = DEC 21

                #ifdef DEBUG_VIA_USB
                Serial.print("temperatureMax=  ");
                Serial.print(temperatureMax);
                Serial.println("");
                #endif
                #ifdef LogToSD
                logData += "488 temperatureMax= ";
                logData += temperatureMax;
                logData += "\n";
                  // Add data to buffer
                addToBuffer(logData);
                #endif

                /*temperatureMin = static_cast<int8_t>(rx_frame.data.u8[2]) / 10;
                 // HEX B0 = DEC 176
                 // HEX AF = DEC 175
                 // HEX FF = DEC 255 ERROR during startup
                 // HEX 89 = DEC 137
                */
                temperatureMin = static_cast<int8_t>(rx_frame.data.u8[4]); // looking at CAN [2] is random nut [4] appears to be lower then Max 
                //HEX OF DEC 15 moving up to
                //HEX 10 DEC 16

                #ifdef DEBUG_VIA_USB
                Serial.print("temperatureMin=  ");
                Serial.print(temperatureMin);
                Serial.println("");
                #endif
                #ifdef LogToSD
                logData += "488 temperatureMin= ";
                logData += temperatureMin;
                logData += "\n";
                  // Add data to buffer
                addToBuffer(logData);
                #endif
	}
  break;
//"BMS1_PACK_TEMP_DATA",0x489

//"BMS_CONTROL",0x506  
//"BMS_PACK_TIME",0x508
//  CRate.bytes[0]=buf[4];
//  CRate.bytes[1]=buf[5];
//"BMS1_PACK_TIME",0x509
//"DASH_STATUS3",0x540


    default: 	
    break;

}// end of switch
  #ifdef LogToSD
	Serial.println("");	
	Serial.print(millis());  // Example printout, time, ID, length, data: 7553  1DB  8  FF C0 B9 EA 0 0 2 5D
	Serial.print(" ");
	Serial.print(rx_frame.ID, HEX);
	Serial.print(" ");
	Serial.print(rx_frame.DLC);
	Serial.print(" ");
  
        #ifdef LogToSD
        String logData = String(millis()) + " " + String(rx_frame.ID, HEX) + " " + String(rx_frame.DLC) + " ";
                                #endif

	for (int i = 0; i < rx_frame.DLC; ++i) {
		//Serial.print(rx_frame.data.u8[i], HEX);
		//Serial.print(" ");

    sprintf(msgString, "%.2X", (rx_frame.data.u8[i]));
    Serial.print(msgString);
    Serial.print(" ");

    #ifdef LogToSD
    logData += String(msgString) + " ";
    #endif
		}
	Serial.println("");	
        #ifdef LogToSD
        logData += "\n";
        // Add data to buffer
        addToBuffer(logData);
        //appendFile(SD, "/ZERO_14_4_BATTERY.txt", logData.c_str());
        #endif
 #endif
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
  Serial.println("ZERO 14.4 battery");
#endif

#ifdef LogToSD
  setupLogToSD("/ZERO_14_4_BATTERY.txt");
  //writeFile(SD, "/ZERO_14_4_BATTERY.txt", "******************    Start of new messages        *************************");
#endif

  datalayer.battery.info.number_of_cells = 112;
  datalayer.battery.info.max_design_voltage_dV = 1160;  // Over this charging is not possible 114Ah x 116V
  datalayer.battery.info.min_design_voltage_dV = 950;  // Under this discharging is disabled 114Ah x 95V
}

/*
void addToBuffer(const String &logData) {
  size_t dataLength = logData.length();

  // Check if buffer has enough space
  if (bufferIndex + dataLength >= BUFFER_SIZE) {
    flushBufferToSD();
  }

  // Add data to buffer
  logData.toCharArray(&dataBuffer[bufferIndex], dataLength + 1);
  bufferIndex += dataLength;
}

void flushBufferToSD() {
  if (bufferIndex == 0) return; // Nothing to write

  // Null-terminate the buffer to ensure it's treated as a C-string
  dataBuffer[bufferIndex] = '\0';

  // Use appendFile to write the buffer content
  appendFile(SD, "/ZERO_14_4_BATTERY.txt", dataBuffer);

  // Clear the buffer
  bufferIndex = 0;
}
*/
#endif
