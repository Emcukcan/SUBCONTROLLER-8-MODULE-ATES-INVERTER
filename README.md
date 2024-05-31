Max Charge Current Algorithm

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


MODBUS POLLING



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
