//version:5-32-2024













//LIBRARIES
#include <Preferences.h>

Preferences preferences;
#include <Arduino.h>

#include "soc/rtc_wdt.h"
#include <esp_task_wdt.h>
#include "bms_temp.h"

#include "slaver.h"
#include <adcread.h>
#include "BluetoothSerial.h"
#include "nvs_flash.h"


#include <can_bms.h>       //DALY CANBUS LIBRARY
#include "can_server.h"   // ATES INVERTER LIBRARY






uint16_t temp1 = 0;
uint16_t temp2 = 0;
float prechargeVolt = 0;
float prechargeCounter = 0;
bool prechargeStatus = true;
bool prechargeTrigger = false;
bool FORCE = false;
bool disconnection = false;
int BREAKER_PULSE = 0;
int ANALOG_PULSE = 0;
int CANBUS_PULSE = 0;
int MODBUS_PULSE = 0;
int disconnect_counter = 0;
int AlarmCode = 0;
int DischargeCounter = 0;
bool DischargeOvertake = false;





//PINS
#define GEN_RE  0


uint16_t MODBUSARRAY[200];

//OBJECTS
uint8_t *BMSAR;


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//TASK HANDLES
TaskHandle_t CANBUSTASK;
TaskHandle_t BREAKERCONTROLTASK;
TaskHandle_t MODBUSTASK;
TaskHandle_t BT;
TaskHandle_t ANALOGTASK;


//VARIABLES
double TerminalVoltageArray[17];
double TerminalCurrentArray[17];
double TerminalTempArray[17];
double TerminalSOCArray[17];
double RemainingCapacity[17];
int ChargeStatusArray[17];
int DischargeStatusArray[17];
int AlarmStatusArray[17];
double MaxCellArray[17];
double MinCellArray[17];
float String1Current = 0;
float String1Voltage = 0;
float String1SOC = 0;
float String1CurrentCal = 0;
float String1VoltageCal = 0;
float String1SOCCal = 0;


float String2Current = 0;
float String2Voltage = 0;
float String2SOC = 0;
float String2CurrentCal = 0;
float String2VoltageCal = 0;
float String2SOCCal = 0;






uint8_t datart[8];

float HighVoltageAlarmStart = 538;
float HighVoltageAlarmStop = 530;
float LowVoltageAlarmStart = 485;
float LowVoltageAlarmStop = 490;
float HighTempAlarmStart = 40;
float HighTempAlarmStop = 35;

bool Rack1ChargeRelay = true;
bool Rack1DischargeRelay = true;
bool Bypass1Relay = true;


bool Rack2ChargeRelay = true;
bool Rack2DischargeRelay = true;
bool Bypass2Relay = true;

bool Fan1 = true;
bool OutputDry = true;


bool Rack1ChargeRelayCal = true;
bool Rack1DischargeRelayCal = true;
bool Bypass1RelayCal = true;
bool Fan1Cal = true;
bool OutputDryCal = true;

bool Rack2ChargeRelayCal = true;
bool Rack2DischargeRelayCal = true;
bool Bypass2RelayCal = true;
bool Fan2Cal = true;
bool OutputDryCa2 = true;




bool EnableForce = false;
bool ForcedRack1ChargeRelay = false;
bool ForcedRack1DischargeRelay = false;
bool ForcedBypass1Relay = false;
bool ForcedFan1 = false;
bool ForcedPrecharge = false;
String FirmwareVer = "1.0.0";
int SubID = 1;
int ModuleSize = 10;
bool ChargeRelay = true;
bool DischargeRelay = true;
int Heartbeat = 0;


float String1MaxCell = 0;
float String1MinCell = 500;
int MaxCurrent = 100;



String SerialNumber = "C"; // ENC-DATE-NUMBER

void setup() {
  RXS = 16;
  TXS = 17;











  pinMode(19, OUTPUT); // Charge1
  pinMode(18, OUTPUT);//Discharge1
  pinMode(21, OUTPUT);//Bypass1
  pinMode(22, OUTPUT);//Fan1
  pinMode(5, OUTPUT);//Precharge

  pinMode(26, OUTPUT);//Bypass2
  pinMode(14, OUTPUT);//Charge2
  pinMode(12, OUTPUT);//Discharge2



  Serial.begin(115200);






  Serial.println("nvs is init");
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);



  preferences.begin("my-app", false);
  HighVoltageAlarmStart = preferences.getInt("HVST", 538);
  HighVoltageAlarmStop = preferences.getInt("HVSP", 532);
  LowVoltageAlarmStart = preferences.getInt("LVST", 485);
  LowVoltageAlarmStop = preferences.getInt("LVSP", 490);
  HighTempAlarmStart = preferences.getInt("HTST", 40);
  HighTempAlarmStop = preferences.getInt("HTSP", 35);
  MaxCurrent = preferences.getInt("MaxCurrent", 100);
  SerialNumber = preferences.getString("SN", "ENC1227202300001");
  SubID = preferences.getInt("SubID", 1);
  ModuleSize = preferences.getInt("MS", 10);
  preferences.end();




  Serial.println("-------------");
  Serial.println("HVST:" + String(HighVoltageAlarmStart));
  Serial.println("-------------");



  can_start(250);
  delay(10);
  I2C_0 .begin(SDA_0, SCL_0);
  delay(500);



  //Serial.println("setup output dry:" + String(OutputDry));


  //CREATING TASK FOR CANBUS_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    CANBUSTASK_CODE,   /* Task function. */
    "CANBUSTASK",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    8,           /* priority of the task */
    &CANBUSTASK,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);


  //CREATING TASK FOR RELAYS_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    BREAKERCONTROLTASK_CODE,   /* Task function. */
    "BREAKERCONTROLTASK",     /* name of task. */
    5000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    3,           /* priority of the task */
    &BREAKERCONTROLTASK,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);


  //CREATING TASK FOR MODBUS
  xTaskCreatePinnedToCore(
    MODBUSTASK_CODE,   /* Task function. */
    "MODBUSTASK",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    5,           /* priority of the task */
    &MODBUSTASK,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(50);


  //CREATING TASK FOR BLUETOOTH_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    BT_CODE,   /* Task function. */
    "BT",     /* name of task. */
    9000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &BT,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);


  //CREATING TASK FOR ANALOG READING_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    ANALOGTASK_CODE,   /* Task function. */
    "ANALOGTASK",     /* name of task. */
    6000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &ANALOGTASK,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);
}






void CANBUSTASK_CODE( void * pvParameters ) {
  Serial.println("CANBUS TASK STARTED");


  int StabilityCounterCharge = 0;
  int StabilityCounterDischarge = 0;
  for (;;) {


    CANBUS_PULSE++;


    for (int i = 0; i < 9; i++) {
      BMS_recieve(0x90, i + 1);
      BMS_recieve(0x92, i + 1);
      BMS_recieve(0x91, i + 1);
      BMS_recieve(0x93, i + 1);
      BMS_recieve(0x95, i + 1);
      BMS_recieve(0x98, i + 1);


      if (( abs(BMS_can.current_can - 30000) * 0.1 < 400)  && (abs(BMS_can.discharge_can) < 2) && (abs(BMS_can.discharge_can) < 2) && ((BMS_can.max_cell_temp_can - 40) < 80) && ((BMS_can.max_cell_temp_can - 40) > -5) && (abs(BMS_can.sum_voltage_can * 0.1) < 70)) {
        //
        //      if (( abs(BMS_can.current_can - 30000) * 0.1 < 400) ){


        if (Heartbeat < 256) {
          Heartbeat++;
        }
        else {
          Heartbeat = 0;
        }


        TerminalVoltageArray[i] = BMS_can.sum_voltage_can * 0.1;
        TerminalCurrentArray[i] = (BMS_can.current_can - 30000) * 0.1;
        TerminalTempArray[i] = BMS_can.max_cell_temp_can - 40;
        TerminalSOCArray[i] = BMS_can.SOC_can * 0.1;
        DischargeStatusArray[i] = BMS_can.discharge_can;
        ChargeStatusArray[i] = BMS_can.charge_can;
        AlarmStatusArray[i] = BMS_can.AlarmStatus_can;
        RemainingCapacity[i] = BMS_can.rem_cap_can * 0.001 * 48 * 0.001;
        MaxCellArray[i] = BMS_can.max_cell_volt_can * 0.001;
        MinCellArray[i] = BMS_can.min_cell_volt_can * 0.001;

//
        Serial.println("---- ID:" + String(i + 1) + "Successfull ------");
        Serial.println("ID:" + String(i + 1) + "Voltage:" + String(TerminalVoltageArray[i]));
        Serial.println("ID:" + String(i + 1) + "Current:" + String(TerminalCurrentArray[i]));
        Serial.println("ID:" + String(i + 1) + "Temperature:" + String(TerminalTempArray[i]));
        Serial.println("ID:" + String(i + 1) + "SOC:" + String(TerminalSOCArray[i]));
        Serial.println("ID:" + String(i + 1) + "Charge:" + String(ChargeStatusArray[i]));
        Serial.println("ID:" + String(i + 1) + "Discharge:" + String(DischargeStatusArray[i]));
        Serial.println("ID:" + String(i + 1) + "Alarm:" + String(AlarmStatusArray[i]));
        Serial.println("HeartBeat:" + String(Heartbeat));
        Serial.println("----------");

        AlarmCode = 0;



      }

      else {
        if (i < 8) {
          Heartbeat = 0;
          AlarmStatusArray[i] = 63;
          AlarmCode = 63;
          Serial.println("#" + String(i + 1) + "  ID lost COMMUNICATION!!!!!!!!!!!!!!!!!!!!!!!!");
        }
      }
      delay(100);

    }

        Serial.println("ChargeRack#1:" + String(Rack1ChargeRelay));
        Serial.println("DischargeRack#1:" + String(Rack1DischargeRelay));
        Serial.println("BypassRack#1:" + String(Bypass1Relay));
        Serial.println("Fan#1:" + String(Fan1));
       Serial.println("Precharge#1:" + String(prechargeStatus));
        Serial.println("------------");
        Serial.println("Temp#1:" + String(temp1));
        Serial.println("------------");
        Serial.println("Inverter String SOC:" + String(String1SOC));
        Serial.println("Inverter String Voltage:" + String(String1Voltage));
        Serial.println("Inverter String Current:" + String(String1Current));




    /////Rack #1////////////////////////

    String1VoltageCal = 0;
    for (int i = 0; i < 8; i++) {
      String1VoltageCal = TerminalVoltageArray[i] + String1VoltageCal;
    }
    String1Voltage = String1VoltageCal;

    String1CurrentCal = 0;
    for (int i = 0; i < 8; i++) {
      String1CurrentCal = TerminalCurrentArray[i] + String1CurrentCal;
    }
    String1CurrentCal = String1CurrentCal / 8;
    String1Current = String1CurrentCal;

    String1SOCCal = 0;
    for (int i = 0; i < 8; i++) {
      String1SOCCal = TerminalSOCArray[i] + String1SOCCal;
    }
    String1SOCCal = String1SOCCal / 8;
    String1SOC = String1SOCCal;


    //////////////////////////////inverter slave


    String1MaxCell = 0;
    for (int i = 0; i < 8; i++) {
      if (MaxCellArray[i] > String1MaxCell) {
        String1MaxCell = MaxCellArray[i];
      }
    }


    String1MinCell = 500;
    for (int i = 0; i < 8; i++) {
      if (MinCellArray[i] < String1MinCell) {
        String1MinCell = MinCellArray[i];
      }
    }



    //429V  CURRENT=0;
    //426V CURRENT=10A
    //424V CURRENT=60A
    //422V CURRENT=80A

    Serial.println("String1MaxCell" + String(String1MaxCell));
    Serial.println("String1MinCell" + String(String1MinCell));
    Serial.println("String1SOC" + String(String1SOC));
    Serial.println("String1Voltage" + String(String1Voltage));
    Serial.println("String1Current" + String(String1Current));
    Serial.println("MaxCurrent" + String(MaxCurrent));




    if (String1MaxCell  < 5  && String1MinCell  < 5  && String1SOC < 105  && abs(String1Current) < 200) {
      set_maxvoltage(String1MaxCell, String1MinCell, String1SOC, String1SOC, 0, 0x180150F1);

      //delay(20);
      delay(200);

      if (String1Voltage < 420) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent, MaxCurrent, 0x180250F1);
      }


      else if (String1Voltage >= 420 && String1Voltage < 421 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.9, MaxCurrent, 0x180250F1);
      }

      else if (String1Voltage >= 421 && String1Voltage < 422 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.8, MaxCurrent, 0x180250F1);
      }

      else if (String1Voltage >= 422 && String1Voltage < 423 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.7, MaxCurrent, 0x180250F1);
      }

      else if (String1Voltage >= 423 && String1Voltage < 424 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.6, MaxCurrent, 0x180250F1);
      }

      else if (String1Voltage >= 424 && String1Voltage < 425 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float disc7arge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.5, MaxCurrent, 0x180250F1);
      }

      else if (String1Voltage >= 425 && String1Voltage < 426 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.4, MaxCurrent, 0x180250F1);
      }
      else if (String1Voltage >= 426 && String1Voltage < 427 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.3, MaxCurrent, 0x180250F1);
      }
        else if (String1Voltage >= 427 && String1Voltage < 427.5 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.2, MaxCurrent, 0x180250F1);
      }
        else if (String1Voltage >= 427.5 && String1Voltage < 428 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.1, MaxCurrent, 0x180250F1);
      }
      else if (String1Voltage >= 428 && String1Voltage < 428.5 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.05, MaxCurrent, 0x180250F1);
      }
        else if (String1Voltage >= 428.5 && String1Voltage < 429 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.03, MaxCurrent, 0x180250F1);
      }
          else if (String1Voltage >= 429 && String1Voltage < 429.5 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.02, MaxCurrent, 0x180250F1);
      }
          else if (String1Voltage >= 429.5 && String1Voltage < 430 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0.01, MaxCurrent, 0x180250F1);
      }
      else if (String1Voltage >= 430 ) {
        //set_totalvoltage(float totalvolt, float totalcurrent, float charge_maxcurr, float discharge_maxcurr ,0x180250F1);
        set_totalvoltage(String1Voltage , String1Current , MaxCurrent * 0, MaxCurrent, 0x180250F1);
      }

    }


    delay(200);
    //set_groupnumber(uint8_t maxvolt_grpnm, uint8_t maxvolt_packnm, uint8_t maxvolt_boxnm, uint8_t maxtemp_grpnm, uint8_t maxtemp_packnm, uint8_t maxtemp, 0x180350F1);
    set_groupnumber(1, 3, 4, 1, 3, 25, 0x180350F1);
    delay(200);
    set_groupnumbermin(2, 4, 6, 2, 3, 25,  0x180450F1);

    set_warnings(4, 0, 0, 0, 0 , 0x180650F1);
    delay(200);
    set_warnings2(0, 0, 0, 0, 0, 0x180750F1);




    //////////////////////




  }
}


void MODBUSTASK_CODE( void * pvParameters ) {
  Serial.println("MODBUS TASK STARTED");
  Modbus slave(SubID, Serial2, GEN_RE);
  baudbaud = 9600;
  slave.start();

  for (;;) {


    MODBUS_PULSE++;


    if (temp1 > HighTempAlarmStart) {
      Fan1 = true;
    }
    if (temp1 < HighTempAlarmStop) {
      Fan1 = false;
    }


    for (int i = 0; i < 10; i++) {



      MODBUSARRAY[i] = TerminalVoltageArray[i] * 10;
      MODBUSARRAY[i + 10] = TerminalCurrentArray[i] * 10;
      MODBUSARRAY[i + 20] = TerminalTempArray[i] * 10;
      MODBUSARRAY[i + 30] = MaxCellArray[i] * 100;
      MODBUSARRAY[i + 40] = MinCellArray[i] * 100;
      MODBUSARRAY[i + 50] = TerminalSOCArray[i] * 10;
      MODBUSARRAY[i + 60] = ChargeStatusArray[i];
      MODBUSARRAY[i + 70] = DischargeStatusArray[i];

    }

    MODBUSARRAY[90] = Rack1ChargeRelay;
    MODBUSARRAY[91] = Rack1DischargeRelay;
    MODBUSARRAY[92] = Bypass1Relay;
    MODBUSARRAY[93] = prechargeStatus;
    MODBUSARRAY[94] = Fan1;
    MODBUSARRAY[95] = temp1;
    MODBUSARRAY[96] = temp2;
    MODBUSARRAY[97] = Heartbeat;

    slave.poll( MODBUSARRAY, 100);
    // Serial.println("SubID:"+String(SubID));
    delay(100);
  }
}



void ANALOGTASK_CODE( void * pvParameters ) {
  // Serial.println("BREAKERCONTROLTASK TASK STARTED");

  int CurrentMillis = 0;
  int PreviousMillis = 0;


  for (;;) {

    ANALOG_PULSE++;


    PreviousMillis = millis();

    //ADC READING------------------------------------------
    // Serial.println(result);



    result = i2c_search(0x48);
    delay(10);

    if (result == 1)
    {
      send_config(1);
      // Serial.println("1");
      temp1 = (adc_value / 0.01);
      //      Serial.println(adc_value);
      //      Serial.print("Temp1:");
      //      Serial.println(temp1);


      delay(250);
      send_config(2);
      prechargeVolt = adc_value;

      //      Serial.print("prechargeVolt:");
      //      Serial.println(prechargeVolt);


      delay(250);
      send_config(3);
      temp2 = (adc_value / 0.01);

      //      Serial.println(adc_value);
      //      Serial.print("Temp2:");
      //      Serial.println(temp2);

      //preChargeControl



      if (prechargeVolt > 2 && prechargeTrigger == false ) {
          Serial.println("precharge trigger is given !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        prechargeStatus = true;
        prechargeTrigger = true;
      }

      if (prechargeCounter > 20) {
          Serial.println("precharge is resetted !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        prechargeStatus = false;
        prechargeCounter = 0;
      }

    }

    CurrentMillis = millis();

    if (prechargeStatus) {
      prechargeCounter = prechargeCounter + (CurrentMillis - PreviousMillis) * 0.001;
       Serial.print("precharge counter:");
       Serial.println(prechargeCounter);
    }

  }


}






void BREAKERCONTROLTASK_CODE( void * pvParameters ) {
  // Serial.println("BREAKERCONTROLTASK TASK STARTED");



  float CurrentMillisBreaker = 0;
  float PreviousMillisBreaker = 0;
  bool TakeOver1 = false;
  bool TakeOver2 = false;

  float DischargeCounter2 = 0;



  float CurrentMillisBreaker2 = 0;
  float PreviousMillisBreaker2 = 0;
  bool TakeOver12 = false;
  bool TakeOver22 = false;

  float DischargeCounter22 = 0;



  for (;;) {
    BREAKER_PULSE++;


    //Charge Control#1
    Rack1ChargeRelayCal = true;
    if (String1Voltage < HighVoltageAlarmStart) {

      for (int i = 0; i < 8; i++) {
        Rack1ChargeRelayCal = Rack1ChargeRelayCal && ChargeStatusArray[i];
      }
      Rack1ChargeRelay = Rack1ChargeRelayCal;
    }
    else {
      Serial.println("charge relay#1 is off due to high voltage:" + String(String1Voltage) + ">" + String(HighVoltageAlarmStart));
      Rack1ChargeRelay = false;
    }


    //Discharge Control#1
    if (String1Voltage > LowVoltageAlarmStart) {
      Rack1DischargeRelayCal = true;
      for (int i = 0; i < 8; i++) {
        Rack1DischargeRelayCal = Rack1DischargeRelayCal && DischargeStatusArray[i];
      }
      Rack1DischargeRelay = Rack1DischargeRelayCal;
    }
    else {
      Serial.println("discharge relay#1 is off due to low voltage:" + String(String1Voltage) + ">" + String(LowVoltageAlarmStart));
      Rack1DischargeRelay = false;
    }


    //Take over conditions are defined for Discharge Control


    TakeOver1 = !Rack1DischargeRelay;
    TakeOver2 = String1Voltage < LowVoltageAlarmStart;

    if ((TakeOver1 || TakeOver2)) {




      CurrentMillisBreaker = millis();
      // Serial.println("Current millis:" + String(CurrentMillisBreaker));
      // Serial.println("Diff millis:" + String(CurrentMillisBreaker - PreviousMillisBreaker));


      //      DischargeCounter2 = DischargeCounter2 + (CurrentMillisBreaker - PreviousMillisBreaker);
      //      Serial.println("Counter" + String(DischargeCounter2));



      DischargeCounter2 = DischargeCounter2 + (CurrentMillisBreaker - PreviousMillisBreaker) * 0.001;
      Serial.println("Discharge takeover counter running:" + String(DischargeCounter2));
    }

    if (DischargeCounter2 > 600 && DischargeCounter2 < 690) {
      if (String1Current >= -2) {
        Rack1DischargeRelay = true;
        Serial.println("Discharge contact takenover, remaining time:" + String(960 - DischargeCounter2));
      }
      else {
        Serial.println("Discharge#1 take over is finished due to high load");
        Rack1DischargeRelay = false;
        DischargeCounter2 = 0;
      }
    }

    if (DischargeCounter2 > 690) {
      Serial.println("Discharge#1 take over is finished due to timeout");
      DischargeCounter2 = 0;
      Rack1DischargeRelay = !(TakeOver1 || TakeOver2);
    }

    if (DischargeCounter2 <= 600) {

      Rack1DischargeRelay = !(TakeOver1 || TakeOver2);

    }

    //  Serial.println("DischargeContact:" + String(Rack1DischargeRelay));






    //Take over conditions are defined for Discharge Control







    //Bypass#1 Control
    if (String1Current < 0 && Rack1DischargeRelay && (abs(String1Current) > 20)) {
      Bypass1Relay = true;

      //    Serial.println("bypass relay is on due to high current:" + String(String1Current) + ">" + String(20));
    }
    else if (String1Current > 0 && Rack1ChargeRelay && (abs(String1Current) > 20)) {
      Bypass1Relay = true;

      //   Serial.println("bypass relay is on due to high current:" + String(String1Current) + ">" + String(20));
      //      Serial.println("Bypass is off:");
      //      Serial.println("Current:" + String(String1Current));
      //      Serial.println("Rack1DischargeRelay:" + String(Rack1DischargeRelay));
    }
    else {


      Bypass1Relay = false;
    }



    if (!FORCE) {

      digitalWrite(18, Rack1ChargeRelay);
      digitalWrite(19, Rack1DischargeRelay);
      digitalWrite(21, Bypass1Relay);
      digitalWrite(22, Fan1);
      digitalWrite(5, prechargeStatus);
    }

    else {
      Serial.println("CAUTION !!Breakers are forced!!!!!!!!!!!");
      digitalWrite(18, ForcedRack1ChargeRelay);
      digitalWrite(19, ForcedRack1DischargeRelay);
      digitalWrite(21, ForcedBypass1Relay);
      digitalWrite(22, ForcedFan1);
      digitalWrite(5, ForcedPrecharge);
    }


    PreviousMillisBreaker = millis();
    //Serial.println("Previous millis:" + String(PreviousMillisBreaker));
    delay(250);
  }

}



void BT_CODE( void * pvParameters ) {

  String message = "";
  int param_start = 0;
  int param_end = 0;
  String BTProcessor;
  int param_start2 = 0;
  int param_end2 = 0;
  String BTProcessor2;

  String BT_STRING;
  String BT_REMAINING;



  String BT_STATION = "EN-SUB-" + SerialNumber + "-#" + String(SubID);

  BluetoothSerial SerialBT;
  SerialBT.begin(BT_STATION.c_str()); //Bluetooth device name
  Serial.println(F("The device started, now you can pair it with bluetooth!"));

  for (;;) {

    if (SerialBT.available()) {
      char incomingChar = SerialBT.read();
      if (incomingChar != '\n') {
        message += String(incomingChar);
      }
      else {
        message = "";
      }


      if (message != "") {
        Serial.println(message);

      }









      //SET ID NUMBER ////////////////////////////////
      param_start2 = message.indexOf("SETID");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 5, param_end2);
        SubID = BTProcessor2.toInt();
        SerialBT.println("ID:" + String(SubID));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("SubID", SubID);
        preferences.end();
        SerialBT.println("System will restart");
        delay(1000);

        ESP.restart();
      }

      //GET ID NUMBER ////////////////////////////////
      param_start2 = message.indexOf("GETID");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("ID:" + String(SubID));
        message = "";
      }




      //GET ModuleSize NUMBER ////////////////////////////////
      param_start2 = message.indexOf("GETMS");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("ModuleSize:" + String(ModuleSize));
        message = "";
      }

      //SET ModuleSize NUMBER ////////////////////////////////
      param_start2 = message.indexOf("SETMS");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 5, param_end2);
        ModuleSize = BTProcessor2.toInt();
        SerialBT.println("ModuleSize:" + String(ModuleSize));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("MS", ModuleSize);
        preferences.end();
        SerialBT.println("System will restart");
        delay(1000);

        ESP.restart();
      }









      //GET fIRMWARE NUMBER ////////////////////////////////
      param_start2 = message.indexOf("GETFW");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("FW:" + FirmwareVer);
        message = "";
      }









      //SET SERIAL NUMBER ////////////////////////////////
      param_start2 = message.indexOf("SETSN");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 5, param_end2);
        SerialNumber = BTProcessor2;
        SerialBT.println("SerialNumber:" + String(SerialNumber));
        message = "";

        preferences.begin("my-app", false);
        preferences.putString("SN", SerialNumber);
        preferences.end();
      }


      //get SERIAL ////////////////////////////////
      param_start2 = message.indexOf("GETSN");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        SerialBT.println("SerialNumber:" + SerialNumber);
        message = "";
      }




      //get PULSES ////////////////////////////////
      param_start2 = message.indexOf("GETPS");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        SerialBT.println("ANLG:" + String(ANALOG_PULSE) + "/BRKR:" + String(BREAKER_PULSE) + "/CNBS:" + String(CANBUS_PULSE) + "/MDBS:" + String(MODBUS_PULSE));
        message = "";
      }

      //get voltage setttings ////////////////////////////////
      param_start2 = message.indexOf("GETVS");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";
      }





      //SET High Temp Start ////////////////////////////////
      param_start2 = message.indexOf("HTST");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        HighTempAlarmStart = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("HTST", HighTempAlarmStart);
        preferences.end();
      }

      //SET High Temp Stop ////////////////////////////////
      param_start2 = message.indexOf("HTSP");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        HighTempAlarmStop = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("HTSP", HighTempAlarmStop);
        preferences.end();
      }





      //SET High Voltage Start ////////////////////////////////
      param_start2 = message.indexOf("HVST");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        HighVoltageAlarmStart = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("HVST", HighVoltageAlarmStart);
        preferences.end();
      }

      //SET High Voltage Stop ////////////////////////////////
      param_start2 = message.indexOf("HVSP");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        HighVoltageAlarmStop = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("HVSP", HighVoltageAlarmStop);
        preferences.end();
      }


      //SET Low Voltage Start ////////////////////////////////
      param_start2 = message.indexOf("LVST");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        LowVoltageAlarmStart = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("LVST", LowVoltageAlarmStart);
        preferences.end();
      }


      //SET Low Voltage Stop ////////////////////////////////
      param_start2 = message.indexOf("LVSP");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 4, param_end2);
        LowVoltageAlarmStop = BTProcessor2.toInt();
        SerialBT.println("HVST:" + String(HighVoltageAlarmStart) + "/HVSP:" + String(HighVoltageAlarmStop) + "/LVST:" + String(LowVoltageAlarmStart) + "/LVSP:" + String(LowVoltageAlarmStop) + "/HTST:" + String(HighTempAlarmStart) + "/HTSP:" + String(HighTempAlarmStop));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("LVSP", LowVoltageAlarmStop);
        preferences.end();
      }



      //SET FORCE ////////////////////////////////
      param_start2 = message.indexOf("FC");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        FORCE = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";

        ForcedRack1DischargeRelay = false;
        ForcedRack1ChargeRelay = false;
        ForcedBypass1Relay = false;
        ForcedFan1 = false;
        ForcedPrecharge   = false;
      }

      //SET CHARGE ////////////////////////////////
      param_start2 = message.indexOf("CH");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        ForcedRack1ChargeRelay = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }


      //SET DISCHARGE ////////////////////////////////
      param_start2 = message.indexOf("DC");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        ForcedRack1DischargeRelay = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }

      //SET BYPASS
      ////////////////////////////////
      param_start2 = message.indexOf("BP");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        ForcedBypass1Relay = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }


      //SET FAN
      ////////////////////////////////
      param_start2 = message.indexOf("FN");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        ForcedFan1 = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }



      //SET PRECHARGE
      ////////////////////////////////
      param_start2 = message.indexOf("PC");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 2, param_end2);
        ForcedPrecharge = BTProcessor2.toInt();
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }


      //SET ALL OPEN
      ////////////////////////////////
      param_start2 = message.indexOf("ALLON");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        ForcedRack1ChargeRelay = true;
        ForcedRack1DischargeRelay = true;
        ForcedBypass1Relay = true;
        ForcedFan1 = true;
        ForcedPrecharge = true;
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }



      //SET ALL close
      ////////////////////////////////
      param_start2 = message.indexOf("ALLOFF");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        ForcedRack1ChargeRelay = false;
        ForcedRack1DischargeRelay = false;
        ForcedBypass1Relay = false;
        ForcedFan1 = false;
        ForcedPrecharge = false;
        SerialBT.println("FC:" + String(FORCE) + "CH:" + String(ForcedRack1ChargeRelay) + "DC:" + String(ForcedRack1DischargeRelay) + "BP:" + String(ForcedBypass1Relay) + "FN:" + String(ForcedFan1) + "PC:" + String(ForcedPrecharge));
        message = "";
      }



      //SET MAX CURRENT//////////

      param_start2 = message.indexOf("SETMAXCURRENT");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 13, param_end2);
        MaxCurrent = BTProcessor2.toInt();
        SerialBT.println("MaxCurrent:" + String(MaxCurrent));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("MaxCurrent", MaxCurrent);
        preferences.end();
      }


      //GET MAXCURRENT ////////////////////////////////
      param_start2 = message.indexOf("GETMAXCURRENT");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("MAX CURRENT:" + String(MaxCurrent));
        message = "";
      }



    }
  }
}





void loop() {



}
