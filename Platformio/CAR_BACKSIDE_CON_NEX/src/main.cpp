#include <Arduino.h>
#include <avr/pgmspace.h>

#include "defines.h"
#include "init_functions.h"
#include "variables.h"

//libs
#include "EasyNextionLibrary.h"
#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "GyverHacks.h"
#include <GyverFilters.h>
#include <NonBlockingRtttl.h>            // библиотека пиликалки
#include <GyverTimers.h>
#include <Arduino_FreeRTOS.h>
#include <ArduinoRS485.h> 
#include <ArduinoModbus.h>
#include <timers.h>


#define PJON_INCLUDE_SWBB               
//#define PJON_INCLUDE_ASYNC_ACK true   //
//#define TS_RESPONSE_TIME_OUT 0        //
#include "PJON.h"
PJON<SoftwareBitBang> bus(PJON_MY_ID );


#define TEMPERATURE_PRECISION 9                  // точность измерения температуры 9 бит
OneWire oneWire(ONE_WIRE_PIN);                   // порт шины 1WIRE
DallasTemperature temp_sensors(&oneWire);        // привязка библиотеки DallasTemperature к библиотеке OneWire
DeviceAddress thermometerID_1, thermometerID_2, thermometerID_3;        // резервируем адреса для трёх датчиков

GTimer timerTempSensorsUpdate;      //таймер период обновления датчиков температуры
GTimer timerPumpOffDelay;            //
GTimer timerLowUConverterOffDelay;
GTimer timerConverterShutdownDelay;
GTimer timerPjonTransmittPeriod;
GTimer timerShutdownDelay;
GTimer timerMenuDynamicUpdate;
GTimer timerScreenOffDelay;
GTimer timerInputsUpdate;     // таймер период обновления входов
GTimer timerStartDelay;       // таймер задержки опроса входов после старта
GTimer timerPrxSensorFeedbackDelay;

GFilterRA voltage_filter;

EasyNex myNex(Serial1);

//мелодии
const  char  * bip_1 = " Connect:d=4,o=6,b=2000:4g5,1p,2g";
const  char  * bip_2 = " Disconnect:d=4,o=6,b=2000:4g,1p,2g5";
const  char  * melody_1 = " melody1:d=4,o=7,b=1000:4c,4d,4e,4f,4g,4a,4h";
const  char  * melody_2 = " melody2:d=4,o=7,b=500:4c,4d,4e,4f,4g,4a,4h";
const  char  * melody_3 = " melody3:d=4,o=7,b=50:e";
const  char  * melody_4 = " melody4:d=4,o=5,b=160:1p,e6,8p,e6,8p,e6,8p";

// FreeRTOS functions
void TaskPilikalka( void *pvParameters );
void TaskLoop( void *pvParameters );
void TaskMenuUpdate( void *pvParameters );
void TaskTempSensorsUpdate( void *pvParameters );
void TaskPjonTransmitter( void *pvParameters );
void TaskVoltageMeasurement( void *pvParameters );
void TaskModBusPool(void *pvParameters );
void TaskOwScanner(void *pvParameters );

#if(DEBUG_GENERAL) 
void TaskDebugSerial(void *pvParameters);
#endif


TaskHandle_t TaskPilikalka_Handler;
TaskHandle_t TaskLoop_Handler;
TaskHandle_t TaskMenuUpdate_Handler;
TaskHandle_t TaskTempSensorsUpdate_Handler;
TaskHandle_t TaskPjonTransmitt_Handler;
TaskHandle_t TaskVoltageMeasurement_Handler;
TaskHandle_t TaskModBusPool_Handler;
TaskHandle_t TaskOwScanner_Handler;


//functions
void fnMenuStaticDataUpdate(void);
void fnMenuDynamicDataUpdate(void);
void fnPumpControl(void);
void fnOutputsUpdate(void);  // функция обновления выходов
void fnInputsUpdate(void);   // функция обновления входов
bool fnEEpromInit(void);     // функция загрузки уставок и проверка eeprom при старте
bool fnConverterControl(float voltage, uint8_t mode);
float fnVoltageRead(void);
void pj_receiver_function(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info);
void fnPjonSender(void);
void fnWaterLevelControl(void);
bool fnSensorsPowerControl(void);
bool fnMainPowerControl(void);
uint8_t fnDebounce(uint8_t sample);


//обработчик прерывания от Timer3 (сброс внешнего WDT)
      ISR(TIMER3_A) {
            main_data.wdt_reset_output_state = 1 - main_data.wdt_reset_output_state;
            digitalWrite(WDT_RESET_OUT, main_data.wdt_reset_output_state);
      }

//>>>>>>>>>>>>> SETUP >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void setup() {

  Timer3.setPeriod(WDT_RESET_PERIOD);     // Устанавливаем период таймера 500000 мкс -> 0.5 гц (сброс внешнего WDT)
  Timer3.enableISR(CHANNEL_A);

  fnIOInit();

  digitalWrite(MAIN_SUPPLY_OUT,HIGH);

  #if(DEBUG_GENERAL)    
      Serial.begin(115200);
  #endif


  //меняем скорость Nextion (одноразово)
    /*
      Serial1.begin(9600); // нынешняя скорость / по умолчанию
      Serial1.print("bauds=115200"); // новая скорость
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff); 
    */  

  delay(2000);                       //задержка
  myNex.begin(NEXTION_BAUD_RATE);   // 
  myNex.writeStr("page 12");
  myNex.writeStr("page 12");
  myNex.writeStr("sleep=0");
  delay(500);
 //myNex.writeStr("dim=25");
 //myNex.writeStr("sleep=1");
 //delay(5000);
 //myNex.writeStr("sleep=0");
 

  if(fnEEpromInit()){
      myNex.writeStr("p12t0.txt", "Ok!");

      memcpy(&old_setpoints_data, &setpoints_data, sizeof(Setpoints)); // копирование структур с настройками для отслеживания изменений уставок
  
      // копирование адресов датчиков из структуры уставок которая сохранена в EEPROM
      memcpy(&thermometerID_1, &setpoints_data.sensors_ID_array[setpoints_data.sensors_select_array[INSIDE_SENSOR-1]-1] [0], sizeof(thermometerID_1)); //    
      memcpy(&thermometerID_2, &setpoints_data.sensors_ID_array[setpoints_data.sensors_select_array[OUTSIDE_SENSOR-1]-1] [0], sizeof(thermometerID_2)); //  
      memcpy(&thermometerID_3, &setpoints_data.sensors_ID_array[setpoints_data.sensors_select_array[SPARE_SENSOR-1]-1] [0], sizeof(thermometerID_3)); //  
 
   }
   else
   {
      myNex.writeStr("p12t0.txt", "fault!");
      while (1);           
   }
   

  delay(1500);

  voltage_filter.setCoef(0.1);           // установка коэффициента фильтрации (0.0... 1.0). Чем меньше, тем плавнее фильтр
  voltage_filter.setStep(10);            // установка шага фильтрации (мс). Чем меньше, тем резче фильтр

  temp_sensors.begin();
  temp_sensors.setResolution(thermometerID_1, TEMPERATURE_PRECISION); 
  temp_sensors.setResolution(thermometerID_2, TEMPERATURE_PRECISION);
  temp_sensors.setResolution(thermometerID_3, TEMPERATURE_PRECISION);
  temp_sensors.setWaitForConversion(false);

  timerTempSensorsUpdate.setInterval(TEMP_SENSORS_UPDATE_PERIOD); 
  timerPumpOffDelay.setMode(MANUAL); 
  timerLowUConverterOffDelay.setMode(MANUAL);
  timerLowUConverterOffDelay.setInterval(((uint32_t)setpoints_data.lowUconverter_off_delay) * MINUTE);
  timerConverterShutdownDelay.setMode(MANUAL);
  timerPjonTransmittPeriod.setInterval(setpoints_data.pjon_transmitt_period * SECOND);
  timerShutdownDelay.setMode(MANUAL);
  timerShutdownDelay.setInterval(setpoints_data.shutdown_delay * HOUR);
  timerScreenOffDelay.setMode(MANUAL);
  timerScreenOffDelay.setInterval(10000);
  timerMenuDynamicUpdate.setInterval(MENU_UPDATE_PERIOD);
  timerInputsUpdate.setInterval(INPUTS_UPDATE_PERIOD);
  timerStartDelay.setMode(MANUAL);
  timerStartDelay.setInterval(START_DELAY);
  timerPrxSensorFeedbackDelay.setMode(MANUAL);
  timerPrxSensorFeedbackDelay.setInterval(PRX_SENSOR_FEEDBACK_DELAY);
 
  fnMenuStaticDataUpdate();

  bus.strategy.set_pin(PJON_BUS_PIN);   // выбор пина дя передачи данных
  bus.set_id(setpoints_data.pjon_ID);   //  установка собственного ID
  bus.begin();  //   
  bus.set_receiver(pj_receiver_function);

  ModbusRTUServer.begin(2, main_data.mb_rates[setpoints_data.mb_baud_rate]); // настройка порта в файле RS485.cpp в конце
  // ModbusRTUServer.configureDiscreteInputs(0x00, 10);
  // ModbusRTUServer.configureInputRegisters(0x00, 10);  
  ModbusRTUServer.configureCoils(0x00, 10);
  ModbusRTUServer.configureHoldingRegisters(0x00, 10); 

  pjon_float_sensor_fault_cnt = setpoints_data.pjon_float_fault_timer; //
  pjon_flow_sensor_fault_cnt = setpoints_data.pjon_float_fault_timer;
  
  timerConverterShutdownDelay.setInterval((setpoints_data.converter_shutdown_delay) * MINUTE);


  //********** FreeRTOS tasks ***************************************************
      xTaskCreate(
      TaskPilikalka
      ,  "Pilikalka"  // A name just for humans
      ,  64 // This stack size can be checked & adjusted by reading the Stack Highwater //128
      ,  NULL
      ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  &TaskPilikalka_Handler );

      xTaskCreate(
      TaskLoop
      ,  "Loop"  // A name just for humans
      ,  192  // This stack size can be checked & adjusted by reading the Stack Highwater //544
      ,  NULL
      ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  &TaskLoop_Handler );

      xTaskCreate(
      TaskMenuUpdate
      ,  "MenuUpdate"  // A name just for humans
      ,  160  // This stack size can be checked & adjusted by reading the Stack Highwater //512
      ,  NULL
      ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  &TaskMenuUpdate_Handler );

      xTaskCreate(
      TaskTempSensorsUpdate
      ,  "TempSensorsUpdate"  // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  &TaskTempSensorsUpdate_Handler );

      xTaskCreate(
      TaskPjonTransmitter
      ,  "PjonTransmitt"  // A name just for humans
      ,  120  // This stack size can be checked & adjusted by reading the Stack Highwater // 512
      ,  NULL
      ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  &TaskPjonTransmitt_Handler );

      xTaskCreate(
      TaskVoltageMeasurement
      ,  "VoltageMeasurement"  // A name just for humans
      ,  64  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  &TaskVoltageMeasurement_Handler );

      xTaskCreate(
      TaskModBusPool
      ,  "ModBusPool"  // A name just for humans
      ,  690 // This stack size can be checked & adjusted by reading the Stack Highwater //1000
      ,  NULL
      ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  &TaskModBusPool_Handler );

      
      xTaskCreate(
      TaskOwScanner
      ,  "OwScanner"  // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  &TaskOwScanner_Handler ); 


      #if(DEBUG_GENERAL)
            xTaskCreate(TaskDebugSerial,
                  "Serial",
                  128,
                  NULL, 
                  1,
                  NULL);
      #endif

  myNex.writeStr("page 0");
  myNex.writeStr("page 0");

  //rtttl :: begin (BUZZER, melody_2);   // пиликаем при старте

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void loop() {
 

} //end loop

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//menu static data update
void fnMenuStaticDataUpdate(void){

      switch (myNex.currentPageId) {
            
            case MAIN_PAGE:
                  
                  break;

            case WATER_PAGE:
                                          
                  break;    

            case IOSTATUS_PAGE:

                  break;

            case SETTINGS_PAGE:
                  
                  break;

            case  ONEWIRESET_PAGE:
                                    //обновляем статические параметры страницы
                                    
                                    for (uint8_t j = 0; j < 8; j++)
                                    { 
                                          tempString2 = String(setpoints_data.sensors_ID_array[0][j],HEX);     
                                          tempString += tempString2;
                                          if (j < 7)tempString += ". ";
                                    }

                                    myNex.writeStr("p4t2.txt", tempString);
                                    tempString = ""; tempString2 = "";

                                    for (uint8_t j = 0; j < 8; j++)
                                    {                               
                                          tempString2 = String(setpoints_data.sensors_ID_array[1][j],HEX);     
                                          tempString += tempString2;
                                          if (j < 7)tempString += ". ";
                                    }

                                    myNex.writeStr("p4t3.txt", tempString);
                                    tempString = ""; tempString2 = "";

                                    for (uint8_t j = 0; j < 8; j++)
                                    {                               
                                          tempString2 = String(setpoints_data.sensors_ID_array[2][j],HEX);     
                                          tempString += tempString2;
                                          if (j < 7)tempString += ". ";
                                    }

                                    myNex.writeStr("p4t11.txt", tempString);
                                    tempString = ""; tempString2 = "";

                                   
                  break;

            case  WATERSET_PAGE:
                                   
                  break;

            case  CONVSET_PAGE:

                  break;

            case LIGHTSET_PAGE:

                  break;

            case PJONSET_PAGE:

                  break;

            case ONEWIRESCANNER_PAGE:

                  break;  

            case MODBUSSET_PAGE:

                  break;   

            case BASESET_PAGE:

                  break;             
            

            default:
            break;
  }
}
//*********************************************************************************

//menu dinamic data update
void fnMenuDynamicDataUpdate(void){

      
      switch (myNex.currentPageId) {
            
            case MAIN_PAGE:
                  myNex. writeNum ( F("water.val") , main_data.water_level_liter );  // 
                  myNex. writeNum ( F("OutsideTemp.val") , main_data.outside_temperature );
                  myNex. writeNum ( F("InsideTemp.val") , main_data.inside_temperature );
                  myNex. writeNum ( F("batVolt.val") , main_data.battery_voltage * 10); // main_data.battery_voltage
                  break;

            case WATER_PAGE:
                  if(main_data.pump_output_state) myNex. writeNum (F("nxPumpState.val"), HIGH);
                  else myNex. writeNum (F("nxPumpState.val"), LOW);

                  myNex. writeNum (F("p1n0.val"), main_data.water_level_liter);
                  myNex. writeNum (F("p1n1.val"), main_data.water_level_percent);
                        
            break;    

            case IOSTATUS_PAGE:
                              if(main_data.door_switch_state)myNex.writeNum(F("p2t0.pco"), WHITE);
                              else myNex.writeNum(F("p2t0.pco"), GRAY);
                              if(main_data.proximity_sensor_state)myNex.writeNum(F("p2t1.pco"), WHITE);
                              else myNex.writeNum(F("p2t1.pco"), GRAY);
                              if(main_data.ignition_switch_state)myNex.writeNum(F("p2t2.pco"), WHITE);
                              else myNex.writeNum(F("p2t2.pco"), GRAY);
                              if(main_data.low_washer_water_level)myNex.writeNum(F("p2t3.pco"), WHITE);
                              else myNex.writeNum(F("p2t3.pco"), GRAY);


                              if(main_data.pump_output_state)myNex.writeNum(F("p2t4.pco"), WHITE);
                              else myNex.writeNum(F("p2t4.pco"), GRAY);
                              if(main_data.light_output_state)myNex.writeNum(F("p2t5.pco"), WHITE);
                              else myNex.writeNum(F("p2t5.pco"), GRAY);
                              // если выход конвертера писать третьим то не отображается на экране
                              if(main_data.converter_output_state)myNex.writeNum(F("p2t6.pco"), WHITE);
                              else myNex.writeNum(F("p2t6.pco"), GRAY);
                              //if(main_data.)myNex.writeNum(F("p2t7.pco"), WHITE);
                              //else myNex.writeNum(F("p2t7.pco"), GRAY);

            break;

            case SETTINGS_PAGE:
                  
            break;

            case  ONEWIRESET_PAGE:
                              //обновляем динамические параметры страницы
                              myNex. writeNum(F("p4n0.val"), setpoints_data.sensors_select_array[INSIDE_SENSOR-1]);
                              myNex. writeNum(F("p4n1.val"), setpoints_data.sensors_select_array[OUTSIDE_SENSOR-1]);
                              myNex. writeNum(F("p4n2.val"), setpoints_data.sensors_select_array[SPARE_SENSOR-1]);

                              //меняем цвет уставки если значение изменено но не сохранено в EEPROM
                              if(old_setpoints_data.sensors_select_array[INSIDE_SENSOR-1] != setpoints_data.sensors_select_array[INSIDE_SENSOR-1])myNex.writeNum(F("p4n0.pco"), YELLOW);
                              else myNex.writeNum(F("p4n0.pco"), WHITE);
                              if(old_setpoints_data.sensors_select_array[OUTSIDE_SENSOR-1] != setpoints_data.sensors_select_array[OUTSIDE_SENSOR-1])myNex.writeNum(F("p4n1.pco"), YELLOW);
                              else myNex.writeNum(F("p4n1.pco"), WHITE);
                              if(old_setpoints_data.sensors_select_array[SPARE_SENSOR-1] != setpoints_data.sensors_select_array[SPARE_SENSOR-1])myNex.writeNum(F("p4n2.pco"), YELLOW);
                              else myNex.writeNum(F("p4n2.pco"), WHITE);

                                    switch (current_item)
                                    {
                                    case  1:
                                          myNex.writeNum(F("p4t6.pco"), BLUE);
                                          myNex.writeNum(F("p4t7.pco"), WHITE);
                                          myNex.writeNum(F("p4t12.pco"), WHITE);
                                          variable_value = &setpoints_data.sensors_select_array[INSIDE_SENSOR-1]; 
                                          var_min_value = 1;
                                          var_max_value = 3;                            
                                          break;

                                    case  2:
                                          myNex.writeNum(F("p4t6.pco"), WHITE);
                                          myNex.writeNum(F("p4t7.pco"), BLUE);
                                          myNex.writeNum(F("p4t12.pco"), WHITE);
                                          variable_value = &setpoints_data.sensors_select_array[OUTSIDE_SENSOR-1];
                                          var_min_value = 1;
                                          var_max_value = 3;
                                          break;

                                    case  3:
                                          myNex.writeNum(F("p4t6.pco"), WHITE);
                                          myNex.writeNum(F("p4t7.pco"), WHITE);
                                          myNex.writeNum(F("p4t12.pco"), BLUE);
                                          variable_value = &setpoints_data.sensors_select_array[SPARE_SENSOR-1];
                                          var_min_value = 1;
                                          var_max_value = 3;
                                          break;

                                    default:
                                          myNex.writeNum(F("p4t6.pco"), WHITE);
                                          myNex.writeNum(F("p4t7.pco"), WHITE);
                                          myNex.writeNum(F("p4t12.pco"), WHITE);
                                          variable_value = NULL;
                                          var_min_value = 0;
                                          var_max_value = 0;
                                          break;
                                    }

                  break;

            case  WATERSET_PAGE:
                                    //обновляем динамические параметры страницы
                                    myNex. writeNum(F("p5n0.val"), setpoints_data.pump_off_delay);
                                    myNex. writeNum(F("p5n1.val"), setpoints_data.flow_sensor_correction);
                                    myNex. writeNum(F("p5n2.val"), setpoints_data.water_tank_capacity);
                                    myNex. writeNum(F("p5n4.val"), main_data.water_level_liter);
                                    myNex. writeNum(F("p5n3.val"), timerPumpOffDelay.currentTime() * 0.001);

                                    //меняем цвет уставки если значение изменено но не сохранено в EEPROM
                                    if(old_setpoints_data.pump_off_delay != setpoints_data.pump_off_delay)myNex.writeNum(F("p5n0.pco"), YELLOW);
                                    else myNex.writeNum(F("p5n0.pco"), WHITE);
                                    if(old_setpoints_data.flow_sensor_correction != setpoints_data.flow_sensor_correction)myNex.writeNum(F("p5n1.pco"), YELLOW);
                                    else myNex.writeNum(F("p5n1.pco"), WHITE);
                                    if(old_setpoints_data.water_tank_capacity != setpoints_data.water_tank_capacity)myNex.writeNum(F("p5n2.pco"), YELLOW);
                                    else myNex.writeNum(F("p5n2.pco"), WHITE);

                                    //обновляем пункт управления насосом
                                    if(main_data.pump_output_state)myNex.writeNum(F("p5t4.pco"), GREEN);
                                    else myNex.writeNum(F("p5t4.pco"), WHITE);

                                    switch (current_item)
                                    {
                                    case  1:
                                          myNex.writeNum(F("p5t1.pco"), BLUE);
                                          myNex.writeNum(F("p5t2.pco"), WHITE);
                                          myNex.writeNum(F("p5t3.pco"), WHITE);
                                          variable_value = &setpoints_data.pump_off_delay;
                                          var_min_value = 1;
                                          var_max_value = 60; // 60 секунд
                                          break;

                                    case  2:
                                          myNex.writeNum(F("p5t1.pco"), WHITE);
                                          myNex.writeNum(F("p5t2.pco"), BLUE);
                                          myNex.writeNum(F("p5t3.pco"), WHITE);
                                          variable_value = &setpoints_data.flow_sensor_correction;
                                          var_min_value = 1;
                                          var_max_value = 255;
                                          break;

                                    case  3:
                                          myNex.writeNum(F("p5t1.pco"), WHITE);
                                          myNex.writeNum(F("p5t2.pco"), WHITE);
                                          myNex.writeNum(F("p5t3.pco"), BLUE);
                                          variable_value = &setpoints_data.water_tank_capacity;
                                          var_min_value = 1;
                                          var_max_value = 100; // 100 литров
                                          break;

                                    default:
                                          myNex.writeNum(F("p5t1.pco"), WHITE);
                                          myNex.writeNum(F("p5t2.pco"), WHITE);
                                          myNex.writeNum(F("p5t3.pco"), WHITE);
                                          variable_value = NULL;
                                          var_min_value = 0;
                                          var_max_value = 0;
                                          break;
                                    }

                  break;

            case  CONVSET_PAGE:
                                    //обновляем динамические параметры страницы
                                    myNex. writeNum(F("p6n0.val"), setpoints_data.lowUconverter_off_delay);
                                    myNex. writeNum(F("p6n1.val"), setpoints_data.converter_shutdown_delay);
                                    myNex. writeNum(F("p6n2.val"), setpoints_data.converter_voltage_off);
                                    myNex. writeNum(F("p6n3.val"), setpoints_data.converter_voltage_on);

                                    switch (setpoints_data.convertet_out_mode)
                                    {
                                          case OFF_MODE:
                                                      myNex. writeStr("p6t6.txt", "OFF");
                                                      break;

                                          case ON_MODE:
                                                      myNex. writeStr("p6t6.txt", "ON");
                                                      break;

                                          case AUTO_MODE:
                                                      myNex. writeStr("p6t6.txt", "AUTO");
                                                      break;            
                                          
                                          default:
                                                break;
                                    }

                                    //меняем цвет уставки если значение изменено но не сохранено в EEPROM
                                    if(old_setpoints_data.lowUconverter_off_delay != setpoints_data.lowUconverter_off_delay)myNex.writeNum("p6n0.pco", YELLOW);
                                    else myNex.writeNum(F("p6n0.pco"), WHITE);
                                    if(old_setpoints_data.converter_shutdown_delay != setpoints_data.converter_shutdown_delay)myNex.writeNum(F("p6n1.pco"), YELLOW);
                                    else myNex.writeNum(F("p6n1.pco"), WHITE);
                                    if(old_setpoints_data.converter_voltage_off != setpoints_data.converter_voltage_off)myNex.writeNum(F("p6n2.pco"), YELLOW);
                                    else myNex.writeNum(F("p6n2.pco"), WHITE);
                                    if(old_setpoints_data.converter_voltage_on != setpoints_data.converter_voltage_on)myNex.writeNum(F("p6n3.pco"), YELLOW);
                                    else myNex.writeNum(F("p6n3.pco"), WHITE);
                                    if(old_setpoints_data.convertet_out_mode != setpoints_data.convertet_out_mode)myNex.writeNum(F("p6t6.pco"), YELLOW);
                                    else myNex.writeNum(F("p6t6.pco"), WHITE);                
                                    
                                    switch (current_item)
                                    {
                                          case 1:
                                                myNex.writeNum(F("p6t1.pco"), BLUE);
                                                myNex.writeNum(F("p6t2.pco"), WHITE);
                                                myNex.writeNum(F("p6t3.pco"), WHITE);
                                                myNex.writeNum(F("p6t4.pco"), WHITE);
                                                myNex.writeNum(F("p6t5.pco"), WHITE);
                                                variable_value = &setpoints_data.lowUconverter_off_delay;
                                                var_min_value = 0;
                                                var_max_value = 32; //min
                                                break;

                                          case 2:
                                                myNex.writeNum(F("p6t1.pco"), WHITE);
                                                myNex.writeNum(F("p6t2.pco"), BLUE);
                                                myNex.writeNum(F("p6t3.pco"), WHITE);
                                                myNex.writeNum(F("p6t4.pco"), WHITE);
                                                myNex.writeNum(F("p6t5.pco"), WHITE);
                                                variable_value = &setpoints_data.converter_shutdown_delay;
                                                var_min_value = 1;
                                                var_max_value = 180; // min
                                                break;                                               

                                          case 3:
                                                myNex.writeNum(F("p6t1.pco"), WHITE);
                                                myNex.writeNum(F("p6t2.pco"), WHITE);
                                                myNex.writeNum(F("p6t3.pco"), BLUE);
                                                myNex.writeNum(F("p6t4.pco"), WHITE);
                                                myNex.writeNum(F("p6t5.pco"), WHITE);
                                                variable_value = &setpoints_data.converter_voltage_off;
                                                var_min_value = 40;
                                                var_max_value = 150; 
                                                break;

                                          case 4:
                                                myNex.writeNum(F("p6t1.pco"), WHITE);
                                                myNex.writeNum(F("p6t2.pco"), WHITE);
                                                myNex.writeNum(F("p6t3.pco"), WHITE);
                                                myNex.writeNum(F("p6t4.pco"), BLUE);
                                                myNex.writeNum(F("p6t5.pco"), WHITE);
                                                variable_value = &setpoints_data.converter_voltage_on;
                                                var_min_value = 40;
                                                var_max_value = 150; 
                                                break; 

                                          case 5:
                                                myNex.writeNum(F("p6t1.pco"), WHITE);
                                                myNex.writeNum(F("p6t2.pco"), WHITE);
                                                myNex.writeNum(F("p6t3.pco"), WHITE);
                                                myNex.writeNum(F("p6t4.pco"), WHITE);
                                                myNex.writeNum(F("p6t5.pco"), BLUE);
                                                variable_value = &setpoints_data.convertet_out_mode;
                                                var_min_value = 0;
                                                var_max_value = 2; 
                                                break;           

                                          default:
                                                myNex.writeNum(F("p6t1.pco"), WHITE);
                                                myNex.writeNum(F("p6t2.pco"), WHITE);
                                                myNex.writeNum(F("p6t3.pco"), WHITE);
                                                myNex.writeNum(F("p6t4.pco"), WHITE);
                                                myNex.writeNum(F("p6t5.pco"), WHITE);
                                                variable_value = NULL;
                                                var_min_value = 0;
                                                var_max_value = 0;

                                          break;
                                    }

                  break;

            case LIGHTSET_PAGE:
                              //обновляем динамические параметры страницы
                              myNex. writeNum(F("p7n0.val"), setpoints_data.light_off_delay);

                              switch (setpoints_data.light_out_mode)
                              {
                                    case OFF_MODE:
                                                myNex. writeStr(F("p7t3.txt"), F("OFF") );
                                                break;

                                    case ON_MODE:
                                                myNex. writeStr(F("p7t3.txt"), F("ON") );
                                                break;

                                    case AUTO_MODE:
                                                myNex. writeStr(F("p7t3.txt"), F("AUTO") );
                                                break;            
                                    
                                    default:
                                    break;
                              }

                              //меняем цвет уставки если значение изменено но не сохранено в EEPROM
                              if(old_setpoints_data.light_off_delay != setpoints_data.light_off_delay)myNex.writeNum(F("p7n0.pco"), YELLOW);
                              else myNex.writeNum(F("p7n0.pco"), WHITE);
                              if(old_setpoints_data.light_out_mode != setpoints_data.light_out_mode)myNex.writeNum(F("p7t3.pco"), YELLOW);
                              else myNex.writeNum(F("p7t3.pco"), WHITE); 

                              switch (current_item)
                              {
                                    case 1:
                                          myNex.writeNum(F("p7t1.pco"), BLUE);
                                          myNex.writeNum(F("p7t2.pco"), WHITE);
                                          variable_value = &setpoints_data.light_off_delay;
                                          var_min_value = 0;
                                          var_max_value = 60; 
                                          break;

                                    case 2:
                                          myNex.writeNum(F("p7t1.pco"), WHITE);
                                          myNex.writeNum(F("p7t2.pco"), BLUE);
                                          variable_value = &setpoints_data.light_out_mode;
                                          var_min_value = 0;
                                          var_max_value = 2; 
                                          break;

                                    default:
                                          myNex.writeNum(F("p7t1.pco"), WHITE);
                                          myNex.writeNum(F("p7t2.pco"), WHITE);
                                          variable_value = NULL;
                                          var_min_value = 0;
                                          var_max_value = 0;
                                    break;
                              }

                  break;

            case PJONSET_PAGE:
                              //обновляем динамические параметры страницы
                              myNex. writeNum(F("p8n0.val"), setpoints_data.pjon_ID);
                              myNex. writeNum(F("p8n1.val"), main_data.water_level_liter);
                              myNex. writeNum(F("p8n3.val"), setpoints_data.pjon_float_fault_timer);
                              myNex. writeNum(F("p8n4.val"), setpoints_data.pjon_transmitt_period);


                        switch (pjon_sender_cnt)
                        {
                              case 0:                 //индикация состояния связи
                                          
                                    switch (pjon_TX__float_sensor_response)
                                    {
                                    case  PJON_ACK:
                                                myNex.writeStr(F("p8t11.txt"), F("ACK") );
                                                break;
                                    
                                    case  PJON_NAK:
                                                myNex.writeStr(F("p8t11.txt"), F("NAK") );
                                                break;
                                    
                                    case  PJON_BUSY:
                                                myNex.writeStr(F("p8t11.txt"), F("BUSY") );
                                                break;

                                    case  PJON_FAIL:
                                                myNex.writeStr(F("p8t11.txt"), F("FAIL") );
                                                break;

                                    default:
                                                myNex.writeStr(F("p8t11.txt"), F("NAN") );
                                                break;
                                    }
                              
                              break;

                              case 1:          //индикация состояния связи

                                    switch (pjon_TX__flow_sensor_response)
                                    {
                                    case  PJON_ACK:
                                                myNex.writeStr("p8t13.txt", "ACK");
                                                break;
                                    
                                    case  PJON_NAK:
                                                myNex.writeStr("p8t13.txt", "NAK");
                                                break;
                                    
                                    case  PJON_BUSY:
                                                myNex.writeStr("p8t13.txt", "BUSY");
                                                break;

                                    case  PJON_FAIL:
                                                myNex.writeStr("p8t13.txt", "FAIL");
                                                break;

                                    default:
                                                myNex.writeStr("p8t13.txt", "NAN");
                                                break;
                                    }     
                                    
                              break;      
                        
                              default:
                              break;
                        }


                              if(!flag_pjon_float_sensor_connected){
                                    myNex.writeStr("p8t10.txt", " <-X->");
                              }
                              else
                              {
                                    myNex.writeStr("p8t10.txt", " <--->");
                              }


                              if(!flag_pjon_flow_sensor_connected){
                                    myNex.writeStr("p8t12.txt", " <-X->");
                              }
                              else
                              {
                                    myNex.writeStr("p8t12.txt", " <--->");
                              }


                              //меняем цвет уставки если значение изменено но не сохранено в EEPROM
                              if(old_setpoints_data.pjon_ID != setpoints_data.pjon_ID)myNex.writeNum(F("p8n0.pco"), YELLOW);
                              else myNex.writeNum(F("p8n0.pco"), WHITE);
                              if(old_setpoints_data.pjon_float_fault_timer != setpoints_data.pjon_float_fault_timer)myNex.writeNum(F("p8n3.pco"), YELLOW);
                              else myNex.writeNum(F("p8n3.pco"), WHITE);
                              if(old_setpoints_data.pjon_transmitt_period != setpoints_data.pjon_transmitt_period)myNex.writeNum(F("p8n4.pco"), YELLOW);
                              else myNex.writeNum(F("p8n4.pco"), WHITE);
                              
                              switch (current_item)
                              {
                                    case 1:
                                          myNex.writeNum(F("p8t1.pco"), BLUE);
                                          myNex.writeNum(F("p8t4.pco"), WHITE);
                                          myNex.writeNum(F("p8t5.pco"), WHITE);
                                          variable_value = &setpoints_data.pjon_ID;
                                          var_min_value = 1;
                                          var_max_value = 254; 
                                          break;

                                    case 2:
                                          myNex.writeNum(F("p8t1.pco"), WHITE);
                                          myNex.writeNum(F("p8t4.pco"), BLUE);
                                          myNex.writeNum(F("p8t5.pco"), WHITE);
                                          variable_value = &setpoints_data.pjon_float_fault_timer;
                                          var_min_value = 0;
                                          var_max_value = 255; 
                                          break;

                                    case 3:
                                          myNex.writeNum(F("p8t1.pco"), WHITE);
                                          myNex.writeNum(F("p8t4.pco"), WHITE);
                                          myNex.writeNum(F("p8t5.pco"), BLUE);
                                          variable_value = &setpoints_data.pjon_transmitt_period;
                                          var_min_value = 0;
                                          var_max_value = 255; 
                                          break;

                                    default:
                                          myNex.writeNum(F("p8t1.pco"), WHITE);
                                          myNex.writeNum(F("p8t4.pco"), WHITE);
                                          myNex.writeNum(F("p8t5.pco"), WHITE);
                                          variable_value = NULL;
                                          var_min_value = 0;
                                          var_max_value = 0;
                                    break;
                              }

            break;

            case ONEWIRESCANNER_PAGE:
                                    
                                    myNex.writeNum (F("p9x0.val"), main_data.inside_temperature * 10);
                                    myNex.writeNum (F("p9x1.val"), main_data.outside_temperature * 10);
                                    myNex.writeNum (F("p9x2.val"), main_data.spare_temperature * 10);

            break;  

            case MODBUSSET_PAGE:
                              //обновляем динамические параметры страницы
                              myNex. writeNum(F("p10n0.val"), setpoints_data.mb_slave_ID);

                              switch (setpoints_data.mb_baud_rate)
                              {
                                    case 0:
                                          myNex. writeStr("p10t3.txt", "4800");
                                          break;

                                    case 1:
                                          myNex. writeStr("p10t3.txt", "7200");
                                          break;
                                    
                                    case 2:
                                          myNex. writeStr("p10t3.txt", "9600");
                                          break;

                                    case 3:
                                          myNex. writeStr("p10t3.txt", "19200");
                                          break;

                                    case 4:
                                          myNex. writeStr("p10t3.txt", "38400");
                                          break;

                                    case 5:
                                          myNex. writeStr("p10t3.txt", "57600");
                                          break;
                                    
                                    default:
                                          myNex. writeStr("p10t3.txt", "none");
                                          break;
                              }

                              //меняем цвет уставки если значение изменено но не сохранено в EEPROM
                              if(old_setpoints_data.mb_slave_ID != setpoints_data.mb_slave_ID)myNex.writeNum(F("p10n0.pco"), YELLOW);
                              else myNex.writeNum(F("p10n0.pco"), WHITE);
                              if(old_setpoints_data.mb_baud_rate != setpoints_data.mb_baud_rate)myNex.writeNum(F("p10t3.pco"), YELLOW);
                              else myNex.writeNum(F("p10t3.pco"), WHITE); 

                              switch (current_item)
                              {
                                    case 1:
                                          myNex.writeNum(F("p10t1.pco"), BLUE);
                                          myNex.writeNum(F("p10t2.pco"), WHITE);
                                          variable_value = &setpoints_data.mb_slave_ID;
                                          var_min_value = 1;
                                          var_max_value = 254; 
                                    break;

                                    case 2:
                                          myNex.writeNum(F("p10t1.pco"), WHITE);
                                          myNex.writeNum(F("p10t2.pco"), BLUE);
                                          variable_value = &setpoints_data.mb_baud_rate;
                                          var_min_value = 0;
                                          var_max_value = 5; 
                                    break;

                                    default:
                                          myNex.writeNum(F("p10t1.pco"), WHITE);
                                          myNex.writeNum(F("p10t2.pco"), WHITE);
                                          variable_value = NULL;
                                          var_min_value = 0;
                                          var_max_value = 0;
                                    break;
                              }


            break;   

            case BASESET_PAGE:    
                              //обновляем динамические параметры страницы
                              switch (setpoints_data.buzzer_out_mode)
                              {
                                    case OFF_MODE:
                                                myNex. writeStr(F("p11t6.txt"), F("OFF"));
                                                break;

                                    case ON_MODE:
                                                myNex. writeStr(F("p11t6.txt"), F("ON") );
                                                break;           
                                    
                                    default:
                                    break;
                              }
                              
                              myNex. writeNum(F("p11n0.val"), setpoints_data.shutdown_delay);
                              myNex. writeNum(F("p11n1.val"), setpoints_data.scrreen_off_delay);
                              myNex. writeNum(F("p11n2.val"), setpoints_data.voltage_correction);
                              myNex. writeNum(F("p11n3.val"), setpoints_data.lcd_brightness);
                              

                              //меняем цвет уставки если значение изменено но не сохранено в EEPROM
                              if(old_setpoints_data.buzzer_out_mode != setpoints_data.buzzer_out_mode)myNex.writeNum(F("p11t6.pco"), YELLOW);
                              else myNex.writeNum(F("p11t6.pco"), WHITE);
                              if(old_setpoints_data.shutdown_delay != setpoints_data.shutdown_delay)myNex.writeNum(F("p11n0.pco"), YELLOW);
                              else myNex.writeNum(F("p11n0.pco"), WHITE);
                              if(old_setpoints_data.scrreen_off_delay != setpoints_data.scrreen_off_delay)myNex.writeNum(F("p11n1.pco"), YELLOW);
                              else myNex.writeNum(F("p11n1.pco"), WHITE);
                              if(old_setpoints_data.voltage_correction != setpoints_data.voltage_correction)myNex.writeNum(F("p11n2.pco"), YELLOW);
                              else myNex.writeNum(F("p11n2.pco"), WHITE);
                              if(old_setpoints_data.lcd_brightness != setpoints_data.lcd_brightness)myNex.writeNum(F("p11n3.pco"), YELLOW);
                              else myNex.writeNum(F("p11n3.pco"), WHITE);
                              
                              switch (current_item)
                              {
                                    case 1:
                                          myNex.writeNum(F("p11t1.pco"), BLUE);
                                          myNex.writeNum(F("p11t2.pco"), WHITE);
                                          myNex.writeNum(F("p11t3.pco"), WHITE);
                                          myNex.writeNum(F("p11t4.pco"), WHITE);
                                          myNex.writeNum(F("p11t5.pco"), WHITE);
                                          variable_value = &setpoints_data.buzzer_out_mode;
                                          var_min_value = 0;
                                          var_max_value = 1; 
                                    break;

                                    case 2:
                                          myNex.writeNum(F("p11t1.pco"), WHITE);
                                          myNex.writeNum(F("p11t2.pco"), BLUE);
                                          myNex.writeNum(F("p11t3.pco"), WHITE);
                                          myNex.writeNum(F("p11t4.pco"), WHITE);
                                          myNex.writeNum(F("p11t5.pco"), WHITE);
                                          variable_value = &setpoints_data.shutdown_delay;
                                          var_min_value = 1;
                                          var_max_value = 24; // hours
                                    break;

                                    case 3:
                                          myNex.writeNum(F("p11t1.pco"), WHITE);
                                          myNex.writeNum(F("p11t2.pco"), WHITE);
                                          myNex.writeNum(F("p11t3.pco"), BLUE);
                                          myNex.writeNum(F("p11t4.pco"), WHITE);
                                          myNex.writeNum(F("p11t5.pco"), WHITE);
                                          variable_value = &setpoints_data.scrreen_off_delay;
                                          var_min_value = 1;
                                          var_max_value = 180; // min
                                    break;

                                    case 4:
                                          myNex.writeNum(F("p11t1.pco"), WHITE);
                                          myNex.writeNum(F("p11t2.pco"), WHITE);
                                          myNex.writeNum(F("p11t3.pco"), WHITE);
                                          myNex.writeNum(F("p11t4.pco"), BLUE);
                                          myNex.writeNum(F("p11t5.pco"), WHITE);
                                          variable_value = &setpoints_data.voltage_correction;
                                          var_min_value = 0;
                                          var_max_value = 255; 
                                    break;

                                    case 5:
                                          myNex.writeNum(F("p11t1.pco"), WHITE);
                                          myNex.writeNum(F("p11t2.pco"), WHITE);
                                          myNex.writeNum(F("p11t3.pco"), WHITE);
                                          myNex.writeNum(F("p11t4.pco"), WHITE);
                                          myNex.writeNum(F("p11t5.pco"), BLUE);
                                          variable_value = &setpoints_data.lcd_brightness;
                                          var_min_value = 10;
                                          var_max_value = 100; 
                                    break;

                                    default:
                                          myNex.writeNum(F("p11t1.pco"), WHITE);
                                          myNex.writeNum(F("p11t2.pco"), WHITE);
                                          myNex.writeNum(F("p11t3.pco"), WHITE);
                                          myNex.writeNum(F("p11t4.pco"), WHITE);
                                          myNex.writeNum(F("p11t5.pco"), WHITE);
                                          variable_value = NULL;
                                          var_min_value = 0;
                                          var_max_value = 0;
                                    break;
                              }

            break;             
            

            default:
            break;
  }
}
//*********************************************************************************

// trigger1 определение текущей страницы (не используется)
void  trigger1 () {

//current_page = myNex.readNumber("dp");
//current_page = myNex.readNumber("dp");
}
//*********************************************************************************

//trigger2 инкремент уставок
void  trigger2 (){
*variable_value = *variable_value + 1 ;
if(*variable_value > var_max_value)*variable_value = var_min_value;
flag_value_changed = HIGH;
}
//*********************************************************************************

//trigger3 декремент уставок
void  trigger3 (){
*variable_value = *variable_value - 1;
if(*variable_value < var_min_value)*variable_value = var_max_value;
flag_value_changed = HIGH;
}
//**********************************************************************************

//trigger4 кнопка ENTER (сохранение уставок в EEPROM )
void  trigger4 (){
EEPROM.updateBlock(EEPROM_SETPOINTS_ADDRESS, setpoints_data);
memcpy(&old_setpoints_data, &setpoints_data, sizeof(Setpoints));
timerPjonTransmittPeriod.setInterval(setpoints_data.pjon_transmitt_period * SECOND); // обновление интервала
flag_value_changed = LOW;
}
//**********************************************************************************

// trigger 5 определяет текущий пункт меню на странице настроек
void  trigger5 (){
current_item = myNex.readNumber("currentItem.val");
current_item = myNex.readNumber("currentItem.val");
}
//**********************************************************************************

// trigger 6 ручное включение насоса
void  trigger6 (){

current_item = myNex.readNumber("currentItem.val");
current_item = myNex.readNumber("currentItem.val");    
main_data.pump_output_state = 1 - main_data.pump_output_state;
if(main_data.pump_output_state)timerPumpOffDelay.setInterval(setpoints_data.pump_off_delay * SECOND);
else timerPumpOffDelay.stop();

} 
//**********************************************************************************

// trigger 7 сканнер 1Wire (перенесен в задачу)
void  trigger7 (){

      flag_ow_scan_to_start = TRUE;  
      
} 
//*******************************************************************************

// trigger 8 flow sensor reset
void  trigger8 (){

      if(!bus.update()) {
            bus.send_packet(PJON_WATER_FLOW_SENSOR_ID, "RESET", 5);  //отправляем команду сброса счетчику воды
      }
}
//******************************************************************************

//pump control
void fnPumpControl(void){

      if(main_data.door_switch_state){
            if( (main_data.proximity_sensor_state == HIGH) && (proximity_sensor_old_state == LOW) && (timerPrxSensorFeedbackDelay.isReady()) ){
                  main_data.pump_output_state = 1 - main_data.pump_output_state;
                  timerPrxSensorFeedbackDelay.setInterval(PRX_SENSOR_FEEDBACK_DELAY);
                  if ( !rtttl::isPlaying() ) rtttl :: begin (BUZZER, melody_1);
                  if(main_data.pump_output_state)timerPumpOffDelay.setInterval(setpoints_data.pump_off_delay * SECOND);
                  else timerPumpOffDelay.stop();
            }  
      }
      else
      {
            main_data.pump_output_state = LOW;
            timerPumpOffDelay.stop();
      }
            
      
      proximity_sensor_old_state = main_data.proximity_sensor_state; 

      if(timerPumpOffDelay.isReady()){
            main_data.pump_output_state = LOW;
            timerPumpOffDelay.stop();
      }

      
}
//*******************************************************************************

//Inputs Update 
   void fnInputsUpdate(void) {                  // функция обновления состояния входов (раз в   мсек)

      if( !digitalRead(DOOR_SWITCH_INPUT_1)) inputs_undebounced_sample |= (1<<0);
      else inputs_undebounced_sample &= ~(1<<0);

      if(!digitalRead(PROXIMITY_SENSOR_INPUT_2)) inputs_undebounced_sample |= (1<<1);
      else inputs_undebounced_sample &= ~(1<<1);

      if(digitalRead(IGNITION_SWITCH_INPUT_3))inputs_undebounced_sample |= (1<<2);
      else inputs_undebounced_sample &= ~(1<<2);

      if(!digitalRead(LOW_WASHER_WATER_LEVEL_INPUT_4))inputs_undebounced_sample |= (1<<3);
      else inputs_undebounced_sample &= ~(1<<3);

      inputs_debounced_state = fnDebounce(inputs_undebounced_sample);

      main_data.door_switch_state = (inputs_debounced_state & (1<<0));
      main_data.proximity_sensor_state = (inputs_debounced_state & (1<<1));
      main_data.ignition_switch_state = (inputs_debounced_state & (1<<2));
      main_data.low_washer_water_level = (inputs_debounced_state & (1<<3));

 /*
      main_data.door_switch_state = !digitalRead(DOOR_SWITCH_INPUT_1);           
      main_data.proximity_sensor_state = !digitalRead(PROXIMITY_SENSOR_INPUT_2);
      main_data.ignition_switch_state = digitalRead(IGNITION_SWITCH_INPUT_3);
      main_data.low_washer_water_level = !digitalRead(LOW_WASHER_WATER_LEVEL_INPUT_4);
 */

   }
//*******************************************************************************

//Outputs Update
   void fnOutputsUpdate(void) {                     // функция обновления состояния выходов

      digitalWrite(WATER_PUMP_OUTPUT_1, main_data.pump_output_state); //
      digitalWrite(LIGHT_OUTPUT_2, main_data.light_output_state); //
      digitalWrite(CONVERTER_OUTPUT_3, main_data.converter_output_state);
      digitalWrite(SENSORS_SUPPLY_5v, main_data.sensors_supply_output_state);
      digitalWrite(MAIN_SUPPLY_OUT, main_data.main_supply_output_state);
   
   }
//*******************************************************************************

//EEPROM Init
   bool fnEEpromInit(void){
      
      EEPROM.readBlock(EEPROM_SETPOINTS_ADDRESS,setpoints_data); // считываем уставки из eeprom
      if(setpoints_data.magic_key == MAGIC_KEY) {                // если ключ совпадает значит не первый запуск
            digitalWrite(BUILTIN_LED, HIGH);                     // и зажигаем светодиод для индикации
            return true;                                         // возвращаем один
      }
      else                                                       // если ключ  не совпадает значит первый запуск
      {     
            myNex.writeStr("p12t0.txt", "Writing defaults...");
            for(uint8_t i=0;i<3;i++ ){                            // мигнём три раза 
                  digitalWrite(BUILTIN_LED, HIGH);
                  delay(1000);
                  digitalWrite(BUILTIN_LED, LOW);
                  delay(1000);
            }

            fnDefaultSetpointsInit();                       // присвеиваем уставки по умолчанию
            EEPROM.writeBlock(EEPROM_SETPOINTS_ADDRESS, setpoints_data);  // и записываем 
            EEPROM.readBlock(EEPROM_SETPOINTS_ADDRESS,setpoints_data); // считываем уставки из eeprom

            if(setpoints_data.magic_key == MAGIC_KEY) {       // проверяем ключ ещё раз
                  digitalWrite(BUILTIN_LED, HIGH);
                  return true;
            }
            else                                              // если не совпадает значит проблемы с EEPROM
            {
                  digitalWrite(BUILTIN_LED, LOW);
                  return false;                              // возвращаем ноль
            }            
      }
      
   }
//*******************************************************************************

//convreter control
bool fnConverterControl(float voltage, uint8_t mode){
    voltage = voltage * 10;
     static bool state = HIGH; // изначально (после старта) включен

     switch (mode)
     {
            case OFF_MODE:
                        state = LOW; 
                        timerLowUConverterOffDelay.stop(); // останавливаем таймер выключения
                        timerConverterShutdownDelay.stop(); //
                        break;
                        
            case ON_MODE:
                        state = HIGH;
                        timerLowUConverterOffDelay.stop(); // останавливаем таймер выключения
                        timerConverterShutdownDelay.stop();
                        break;

            case AUTO_MODE:
                        if(voltage >= setpoints_data.converter_voltage_on ){  // 
                             if(! flag_convOff_due_ign_switch) state = HIGH; // если напряжение в пределах нормы включаем преобразователь
                              flag_convOff_due_voltage = LOW; // флаг что было отключение по низкому напряжению
                              timerLowUConverterOffDelay.stop(); // останавливваем таймер выключения 

                        }

                         
                        if( voltage > setpoints_data.converter_voltage_off ) { //               
                              timerLowUConverterOffDelay.setInterval(((uint32_t)setpoints_data.lowUconverter_off_delay) * MINUTE); // заряжаем таймер на выключение

                        }

                        else {

                              if( timerLowUConverterOffDelay.isReady()){
                                    state = LOW; 
                                    flag_convOff_due_voltage = HIGH; // флаг что было отключение по низкому напряжению
                                    timerLowUConverterOffDelay.stop(); // останавливаем таймер выключения
                                    
                              }
                        }
                          

                        // отключение по таймеру после выключения зажигания
                        if(main_data.ignition_switch_state) {
                              flag_convOff_due_ign_switch = LOW; //сброс флага что было отключение по ignition switch
                              if(! flag_convOff_due_voltage ) state = HIGH;
                              timerConverterShutdownDelay.setInterval(((uint32_t)setpoints_data.converter_shutdown_delay) * MINUTE);           
                        }
                        else
                        {
                              if (timerConverterShutdownDelay.isReady()) {
                                    state = LOW;
                                    flag_convOff_due_ign_switch = HIGH; //установка флага что было отключение по ignition switch
                              }                                    
                        }
                        

                  break;

            default:
                  break;
     }
      
      return state;
}
//*****************************************************************************************

// Analog read 
float fnVoltageRead(void){

      float voltage;
     // voltage =  (analogRead(SUPPLY_VOLTAGE_INPUT) * DIVISION_RATIO_VOLTAGE_INPUT); // 
     voltage = ( voltage_filter.filtered(analogRead(SUPPLY_VOLTAGE_INPUT) - 127 + setpoints_data.voltage_correction) * DIVISION_RATIO_VOLTAGE_INPUT); //
      return voltage;
}
//******************************************************************************************

// fnPjonReceiver 

   void pj_receiver_function(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {

  receive_from_ID = packet_info.sender_id; // от кого пришли данные 
  
  if(receive_from_ID == PJON_WATER_LEVEL_SENSOR_ID){  // если данные пришли от датчика уровня ...
      memcpy(&pjon_wls_percent_receive, payload, sizeof(pjon_wls_percent_receive)); //... копируем данные в соответствующую структуру
      pjon_float_sensor_fault_cnt = 0;  // 
   }

   if(receive_from_ID == PJON_WATER_FLOW_SENSOR_ID){  // если данные пришли от датчика протечки ...
      memcpy(&pjon_wfs_liter_receive, payload, sizeof(pjon_wfs_liter_receive)); //... копируем данные в соответствующую структуру
      pjon_flow_sensor_fault_cnt = 0;  // 
   }

 };

//************************************************************************************************

// fnWaterLevelControl

 void fnWaterLevelControl(void){

   if(pjon_wls_percent_receive.data != pjon_wls_percent_old){  // если изменилось состояние уровня воды...

      switch (pjon_wls_percent_receive.data){     
         

            case WATER_LEVEL_LESS_THEN_25 :   // если меньше четверти ...
                                 main_data.water_level_percent = 1;
                                 break;

            case WATER_LEVEL_25 :
                                 main_data.water_level_percent = 25;
                                // tone(BUZZER,1000, 100);
                                 break;

            case WATER_LEVEL_50 :
                                 main_data.water_level_percent = 50;
                                 
                                 //tone(BUZZER,1000, 100);
                                 break;

            case WATER_LEVEL_75 :
                                 main_data.water_level_percent = 75;
                                 
                                 //tone(BUZZER,1000, 100);
                                 break;
               
            case WATER_LEVEL_100 :              //если максимум...
                                 main_data.water_level_percent = 100;
                                 
                                 //tone(BUZZER,1000, 100);
                                 break;

            case WATER_LEVEL_SENSOR_DEFECTIVE :  // если сенсор неисправен...
                                 main_data.water_level_percent = 0;
                                 break;

            default:
                  main_data.water_level_percent = 0;
                  break;
      }

      pjon_wls_percent_old = pjon_wls_percent_receive.data; // обновляем предыдущее состояние

   }

   //main_data.water_level_liter = pjon_wfs_liter_receive.data;
   main_data.water_level_liter = (setpoints_data.water_tank_capacity * main_data.water_level_percent * 0.01) ;

 }
//*******************************************************************************************

// fnPjonSender
 void fnPjonSender(void){
     
      if(!bus.update()) {

            switch (pjon_sender_cnt)
            {
            case 0:
                  pjon_TX__float_sensor_response = bus.send_packet(PJON_WATER_LEVEL_SENSOR_ID, "R", 1);  //отправляем запрос к датчику уровня воды
                  if(pjon_float_sensor_fault_cnt < setpoints_data.pjon_float_fault_timer) pjon_float_sensor_fault_cnt++;
                  pjon_sender_cnt++;
                  break;

            case 1:
                  pjon_TX__flow_sensor_response = bus.send_packet(PJON_WATER_FLOW_SENSOR_ID, "R", 1);  //отправляем запрос к датчику протечки воды
                  if(pjon_flow_sensor_fault_cnt < setpoints_data.pjon_float_fault_timer) pjon_flow_sensor_fault_cnt++;
                  pjon_sender_cnt++;
                  break;      
            
            default:
                  break;
            }

            if(pjon_sender_cnt > (PJON_MAX_NODES - 1)) pjon_sender_cnt = 0;
                  
      }  
 }   
//*******************************************************************************************

// TaskPilikalka
 void TaskPilikalka( void *pvParameters __attribute__((unused)) )  // This is a Task.
 {
      for (;;) // A Task shall never return or exit.
      {
          if(setpoints_data.buzzer_out_mode)  rtttl :: play ();  // обновление функции мелодии
          else
          {
                rtttl::stop();
          }
          
            vTaskDelay(4);
      }
 }
//*************************************************************************************

//
 void TaskLoop( void *pvParameters __attribute__((unused)) )  // This is a Task.
 {
      for (;;) // A Task shall never return or exit.
      {
            myNex. NextionListen ();

            // ********* ModBus registers update ******************************
            ModbusRTUServer.coilWrite(0x00, main_data.ignition_switch_state);
            ModbusRTUServer.coilWrite(0x01, main_data.door_switch_state);
            ModbusRTUServer.coilWrite(0x02, main_data.proximity_sensor_state);
            ModbusRTUServer.coilWrite(0x03, main_data.pump_output_state);
            ModbusRTUServer.coilWrite(0x04, main_data.converter_output_state);
            ModbusRTUServer.coilWrite(0x05, main_data.light_output_state);
            ModbusRTUServer.coilWrite(0x06, flag_pjon_float_sensor_connected);
            ModbusRTUServer.coilWrite(0x07, flag_pjon_flow_sensor_connected);
            ModbusRTUServer.coilWrite(0x08, main_data.sensors_supply_output_state);
            ModbusRTUServer.coilWrite(0x09, main_data.low_washer_water_level); //
             
            ModbusRTUServer.holdingRegisterWrite(0x00, main_data.battery_voltage * 10);
            ModbusRTUServer.holdingRegisterWrite(0x01, main_data.inside_temperature * 10);
            ModbusRTUServer.holdingRegisterWrite(0x02, main_data.outside_temperature * 10);
            ModbusRTUServer.holdingRegisterWrite(0x03, main_data.water_level_percent);
            ModbusRTUServer.holdingRegisterWrite(0x04, main_data.water_level_liter);

            ModbusRTUServer.holdingRegisterWrite(0x05, pj_float_sensor_fault_counter);
            ModbusRTUServer.holdingRegisterWrite(0x06, pj_flow_sensor_fault_counter);


            if(timerInputsUpdate.isReady() && timerStartDelay.isReady()) fnInputsUpdate();

            if(myNex.currentPageId != ONEWIRESCANNER_PAGE)flag_ow_scanned = LOW;

            main_data.converter_output_state = fnConverterControl(main_data.battery_voltage, setpoints_data.convertet_out_mode);


            //******* отслеживание изменения состояния двери для звуковой индикации
            if(main_data.door_switch_state != flag_door_switch_old_state ){
                  flag_door_switch_old_state = main_data.door_switch_state;
                  if(main_data.door_switch_state){
                        main_data.screen_sleep_mode = false;   //myNex.writeStr("sleep=0");

                        if(main_data.water_level_liter < 10 || main_data.water_level_percent < 25){
                              if ( !rtttl::isPlaying() ) rtttl :: begin (BUZZER, melody_4);
                        }  

                        if ( !rtttl::isPlaying() ) rtttl :: begin (BUZZER, bip_1);     
                  }
                  else
                  {
                        if ( !rtttl::isPlaying() ) rtttl :: begin (BUZZER, bip_2);    
                  }                        
            }


            //****** таймер на отключение экрана
            if(main_data.door_switch_state){
                  timerScreenOffDelay.setInterval((uint32_t)setpoints_data.scrreen_off_delay * SECOND);
            }
            else
            {
                  if(timerScreenOffDelay.isReady()) main_data.screen_sleep_mode = true; //myNex.writeStr("sleep=1");
            }
            
            if(main_data.screen_sleep_mode)myNex.writeStr("sleep=1");
            else myNex.writeStr("sleep=0");
            
            


            //******* Pjon float sensor fault detection 
            if(pjon_float_sensor_fault_cnt >= setpoints_data.pjon_float_fault_timer){    // если больше n запросов сенсору без ответа
                  pjon_wls_percent_receive.data = 0;   // обнуляем значение уровня воды
                  flag_pjon_float_sensor_connected = false;
            }

            if(pjon_float_sensor_fault_cnt == 0)flag_pjon_float_sensor_connected = true;


            if(flag_pjon_float_sensor_connected != flag_pjon_float_sensor_connected_old_state){
                  
                  if(!flag_pjon_float_sensor_connected){
                        if ( !rtttl::isPlaying() ) rtttl :: begin (BUZZER, melody_3);
                        pj_float_sensor_fault_counter++; 
                  }
                  else
                  {
                        if ( !rtttl::isPlaying() ) rtttl :: begin (BUZZER, melody_2);
                  }

                flag_pjon_float_sensor_connected_old_state = flag_pjon_float_sensor_connected;  
            }



            //******* Pjon flow sensor fault detection 
            if(pjon_flow_sensor_fault_cnt >= setpoints_data.pjon_float_fault_timer){    // если больше n запросов сенсору без ответа
                  pjon_wfs_liter_receive.data = 0;   // обнуляем значение уровня воды
                  flag_pjon_flow_sensor_connected = false;
            }

            if(pjon_flow_sensor_fault_cnt == 0)flag_pjon_flow_sensor_connected = true;


            if(flag_pjon_flow_sensor_connected != flag_pjon_flow_sensor_connected_old_state){
                  
                  if(!flag_pjon_flow_sensor_connected){
                        if ( !rtttl::isPlaying() ) rtttl :: begin (BUZZER, melody_3);
                        pj_flow_sensor_fault_counter++; 
                  }
                  else
                  {
                        if ( !rtttl::isPlaying() ) rtttl :: begin (BUZZER, melody_2);
                  }

                flag_pjon_flow_sensor_connected_old_state = flag_pjon_flow_sensor_connected; 

            }



            //*******
            fnWaterLevelControl();
            main_data.main_supply_output_state = fnMainPowerControl();
            main_data.sensors_supply_output_state = fnSensorsPowerControl();

            fnPumpControl();

            fnOutputsUpdate();

            vTaskDelay(1);  // * 15 ms
      }

 }
 //***************************************************************************

 //
 void TaskMenuUpdate( void *pvParameters __attribute__((unused)) )  // This is a Task.
 {
      while (1)
      {
            fnMenuDynamicDataUpdate(); 

            if(myNex.currentPageId != myNex.lastCurrentPageId){
                        fnMenuStaticDataUpdate();
                        myNex.lastCurrentPageId = myNex.currentPageId;
            }

            vTaskDelay(30); // 30 * 15 = 450 ms
      }
 }
 //****************************************************************************

 //
 void TaskTempSensorsUpdate( void *pvParameters __attribute__((unused)) )  
 {
      while (1)
      {
            flag_ds18b20_update = 1-flag_ds18b20_update;
            if(!flag_ds18b20_update)temp_sensors.requestTemperatures();  //команда начала преобразования 
            else{
                  main_data.inside_temperature = temp_sensors.getTempC(thermometerID_1);  // считывание температуры
                  main_data.outside_temperature = temp_sensors.getTempC(thermometerID_2);
                  main_data.spare_temperature = temp_sensors.getTempC(thermometerID_3);
            }   

            vTaskDelay(35); // * 15 ms 
      }
 }
 //***************************************************************

 //
 void TaskPjonTransmitter( void *pvParameters __attribute__((unused)) )  
 {
      while (1)
      {
            if(timerPjonTransmittPeriod.isReady()){
                  fnPjonSender();
            }

      
            pjon_RX_response = bus.receive(1000); // прием данных PJON и возврат результата приёма
           
            vTaskDelay(1); //  * 15 ms 
      }
 }
//********************************************************************

//
void TaskVoltageMeasurement( void *pvParameters __attribute__((unused)) )  
 {
      while (1)
      {
           main_data.battery_voltage =  fnVoltageRead();
            
            vTaskDelay(5); // *15 ms 
      }
 }
//********************************************************************

// Sensors power control + (sleep mode)
bool fnSensorsPowerControl(void){

      return true;
}
//**********************************************************************

//
void TaskModBusPool( void *pvParameters __attribute__((unused)) )  
 {
      while (1)
      {
            //taskENTER_CRITICAL();
            //vTaskSuspendAll();
            //vTaskSuspend(TaskMenuUpdate_Handler);
            ModbusRTUServer.poll();
            //vTaskResume(TaskMenuUpdate_Handler);
           // taskEXIT_CRITICAL();
           // xTaskResumeAll();
            vTaskDelay(20); // *15 ms
            //vTaskDelay( 300 / portTICK_PERIOD_MS );
      }
 }
//********************************************************************


void TaskOwScanner( void *pvParameters __attribute__((unused)) )  
{
      while (1)
      {
            if(flag_ow_scan_to_start){

                  if(!flag_ow_scanned){

                        myNex.writeStr("p9t0.txt", "SCANNING...");    
                        delay(1000);

                        uint8_t address[8];
                        uint8_t count_sensors = 0;
                        String tempString = "";
                        String tempString2;

                        if (oneWire.search(address)){
                        
                              do {
                                    count_sensors++;
                                          
                                    switch (count_sensors)
                                    {
                                          case 1:
                                                for (uint8_t j = 0; j < 8; j++)
                                                { 
                                                setpoints_data.sensors_ID_array[INSIDE_SENSOR-1][j] = address[j];  thermometerID_1[j] = address[j];
                                                tempString2 = String(address[j],HEX);     
                                                tempString += tempString2;
                                                if (j < 7)tempString += ". ";
                                                }
                                                myNex.writeStr("p9t2.txt", tempString);
                                                tempString = ""; tempString2 = "";
                                                break;

                                          case 2:
                                                for (uint8_t j = 0; j < 8; j++)
                                                { 
                                                setpoints_data.sensors_ID_array[OUTSIDE_SENSOR-1][j] = address[j];  thermometerID_2[j] = address[j];
                                                tempString2 = String(address[j],HEX);     
                                                tempString += tempString2;
                                                if (j < 7)tempString += ". ";
                                                }
                                                myNex.writeStr("p9t3.txt", tempString);
                                                tempString = ""; tempString2 = "";
                                                break;
                                          case 3:
                                                for (uint8_t j = 0; j < 8; j++)
                                                { 
                                                setpoints_data.sensors_ID_array[SPARE_SENSOR-1][j] = address[j];  thermometerID_3[j] = address[j];
                                                tempString2 = String(address[j],HEX);     
                                                tempString += tempString2;
                                                if (j < 7)tempString += ". ";
                                                }
                                                myNex.writeStr("p9t4.txt", tempString);
                                                tempString = ""; tempString2 = "";
                                                break;        
                                          
                                          default:
                                          break;
                                    }

                                    if(count_sensors>3)break;  // если найдено больше трёх датчиков - выходим из цикла

                              } while (oneWire.search(address));
                              
                        }


                        if(count_sensors) 
                        {
                              if(count_sensors<3) {
                                    
                                    switch (count_sensors){

                                          case 0:
                                                for (uint8_t j = 0; j < 8; j++)
                                                { 
                                                setpoints_data.sensors_ID_array[INSIDE_SENSOR-1][j] = 0;  thermometerID_1[j] = 0;
                                                tempString2 = String(0,HEX);     
                                                tempString += tempString2;
                                                if (j < 7)tempString += ". ";
                                                }
                                                myNex.writeStr("p9t2.txt", tempString);
                                                tempString = ""; tempString2 = "";

                                                for (uint8_t j = 0; j < 8; j++)
                                                { 
                                                setpoints_data.sensors_ID_array[OUTSIDE_SENSOR-1][j] = 0;  thermometerID_2[j] = 0;
                                                tempString2 = String(0,HEX);     
                                                tempString += tempString2;
                                                if (j < 7)tempString += ". ";
                                                }
                                                myNex.writeStr("p9t3.txt", tempString);
                                                tempString = ""; tempString2 = "";

                                                for (uint8_t j = 0; j < 8; j++)
                                                { 
                                                setpoints_data.sensors_ID_array[SPARE_SENSOR-1][j] = 0;  thermometerID_3[j] = 0;
                                                tempString2 = String(0,HEX);     
                                                tempString += tempString2;
                                                if (j < 7)tempString += ". ";
                                                }
                                                myNex.writeStr("p9t4.txt", tempString);
                                                tempString = ""; tempString2 = "";

                                                break;

                                          case 1:
                                                for (uint8_t j = 0; j < 8; j++)
                                                { 
                                                setpoints_data.sensors_ID_array[OUTSIDE_SENSOR-1][j] = 0;  thermometerID_2[j] = 0;
                                                tempString2 = String(0,HEX);     
                                                tempString += tempString2;
                                                if (j < 7)tempString += ". ";
                                                }
                                                myNex.writeStr("p9t3.txt", tempString);
                                                tempString = ""; tempString2 = "";

                                                for (uint8_t j = 0; j < 8; j++)
                                                { 
                                                setpoints_data.sensors_ID_array[SPARE_SENSOR-1][j] = 0;  thermometerID_3[j] = 0;
                                                tempString2 = String(0,HEX);     
                                                tempString += tempString2;
                                                if (j < 7)tempString += ". ";
                                                }
                                                myNex.writeStr("p9t4.txt", tempString);
                                                tempString = ""; tempString2 = "";

                                                break;

                                          case 2:
                                                for (uint8_t j = 0; j < 8; j++)
                                                { 
                                                setpoints_data.sensors_ID_array[SPARE_SENSOR-1][j] = 0;  thermometerID_3[j] = 0;
                                                tempString2 = String(0,HEX);     
                                                tempString += tempString2;
                                                if (j < 7)tempString += ". ";
                                                }
                                                myNex.writeStr("p9t4.txt", tempString);
                                                tempString = ""; tempString2 = "";
                                                break;

                                         

                                          default :
                                                break;


                                    }

                              }

                              tempString = "FOUND ";
                              tempString2 = String(count_sensors, HEX);
                              tempString += tempString2;
                              tempString2 = " SENSORS";
                              tempString += tempString2;
                              myNex.writeStr("p9t0.txt", tempString); 
                              tempString = "";
                              tempString2 = "";   
                              

                              temp_sensors.requestTemperatures();
                             // delay(1000);
                              main_data.inside_temperature = temp_sensors.getTempC(thermometerID_1);
                              main_data.outside_temperature = temp_sensors.getTempC(thermometerID_2);
                              main_data.spare_temperature = temp_sensors.getTempC(thermometerID_3);
                              
                              myNex.writeStr("p9b0.txt", "Save");
                                    
                        }
                        else
                        {
                              myNex.writeStr("p9t0.txt", "NO SENSOR FOUND");
                              myNex.writeStr("p9b0.txt", "Exit");
                        }

              
                        flag_ow_scanned = HIGH;

                  }
                  else
                  {
                        myNex.writeStr ("page 4");
                        EEPROM.updateBlock(EEPROM_SETPOINTS_ADDRESS, setpoints_data);
                        
                  }

            }
            
            flag_ow_scan_to_start = FALSE;

            vTaskDelay(1); //  *15 ms 
      }
}
//********************************************************************

// Main power control + (sleep mode)
bool fnMainPowerControl(void){
     bool _state;
     if(main_data.ignition_switch_state) {
            timerShutdownDelay.setInterval((uint32_t)setpoints_data.shutdown_delay * HOUR);
            _state = true;
      }  
      else {
            if( timerShutdownDelay.isReady() ) _state = false; 
      }   

      return _state;     
}
//**********************************************************************

// DEBUG 
#if(DEBUG_GENERAL)
  void TaskDebugSerial(void *pvParameters)
      {
            (void) pvParameters;

            for (;;)
            {
                  Serial.println F("======== Tasks status ========");
                  Serial.print F("Tick count: ");
                  Serial.print(xTaskGetTickCount());
                  Serial.print F(", Task count: ");
                  Serial.print(uxTaskGetNumberOfTasks());

                  Serial.println();
                  Serial.println();

                  // Serial task status
                  Serial.print F("- TASK ");
                  Serial.print(pcTaskGetName(NULL));              //
                  Serial.print F(", High Watermark: ");
                  Serial.print(uxTaskGetStackHighWaterMark(NULL)); //  
                  //TaskHandle_t taskDebugSerialHandler = xTaskGetCurrentTaskHandle(); // 
                  Serial.println();

                  //Pilikalka
                  Serial.print F("- TASK ");
                  Serial.print(pcTaskGetName(TaskPilikalka_Handler)); // Get task name with handler
                  Serial.print F(", High Watermark: ");
                  Serial.print(uxTaskGetStackHighWaterMark(TaskPilikalka_Handler));
                  Serial.println(); 

                  //loop
                  Serial.print F("- TASK ");
                  Serial.print(pcTaskGetName(TaskLoop_Handler)); // Get task name with handler
                  Serial.print F(", High Watermark: ");
                  Serial.print(uxTaskGetStackHighWaterMark(TaskLoop_Handler));
                  Serial.println(); 

                  //menuUpdate
                  Serial.print F("- TASK ");
                  Serial.print(pcTaskGetName(TaskMenuUpdate_Handler)); // Get task name with handler
                  Serial.print F(", High Watermark: ");
                  Serial.print(uxTaskGetStackHighWaterMark(TaskMenuUpdate_Handler));
                  Serial.println(); 

                  //TempSensorUpdate
                  Serial.print F("- TASK ");
                  Serial.print(pcTaskGetName(TaskTempSensorsUpdate_Handler)); // Get task name with handler
                  Serial.print F(", High Watermark: ");
                  Serial.print(uxTaskGetStackHighWaterMark(TaskTempSensorsUpdate_Handler));
                  Serial.println(); 

                  //PjonTransmitter
                  Serial.print F("- TASK ");
                  Serial.print(pcTaskGetName(TaskPjonTransmitt_Handler)); // Get task name with handler
                  Serial.print F(", High Watermark: ");
                  Serial.print(uxTaskGetStackHighWaterMark(TaskPjonTransmitt_Handler));
                  Serial.println(); 

                  //VoltageMeasurement
                  Serial.print F("- TASK ");
                  Serial.print(pcTaskGetName(TaskVoltageMeasurement_Handler)); // Get task name with handler
                  Serial.print F(", High Watermark: ");
                  Serial.print(uxTaskGetStackHighWaterMark(TaskVoltageMeasurement_Handler));
                  Serial.println(); 

                  //ModBusPool
                  Serial.print F("- TASK ");
                  Serial.print(pcTaskGetName(TaskModBusPool_Handler)); // Get task name with handler
                  Serial.print F(", High Watermark: ");
                  Serial.print(uxTaskGetStackHighWaterMark(TaskModBusPool_Handler));
                  Serial.println(); 
            
                  //OwScanner
                  Serial.print F("- TASK ");
                  Serial.print(pcTaskGetName(TaskOwScanner_Handler)); // Get task name with handler
                  Serial.print F(", High Watermark: ");
                  Serial.print(uxTaskGetStackHighWaterMark(TaskOwScanner_Handler));
                  Serial.println(); 
                  
                  Serial.println();
                  Serial.println();
                  
                  Serial.print F("timerPumpOffDelay:  ");
                  Serial.println(timerPumpOffDelay.currentTime() / 1000);
                  Serial.print F("timerLowUConverterOffDelay:  ");
                  Serial.println(timerLowUConverterOffDelay.currentTime() / 1000);
                  Serial.print F("timerConverterShutdownDelay:  ");
                  Serial.println(timerConverterShutdownDelay.currentTime() /1000);
                  Serial.print F("timerShutdownDelay:  ");
                  Serial.println(timerShutdownDelay.currentTime() / 1000);
                  Serial.print F("timerScreenOffDelay:  ");
                  Serial.println(timerScreenOffDelay.currentTime() / 1000);

                  Serial.println();

                  Serial.print F("door_switch_state:  ");
                  Serial.println (main_data.door_switch_state);
                  Serial.print F("ignition_switch_state:  ");
                  Serial.println (main_data.ignition_switch_state);
                  Serial.print F("proximity_sensor_state:  ");
                  Serial.println (main_data.proximity_sensor_state);

                  Serial.println();

                  Serial.print F("converter_output_state:  ");
                  Serial.println (main_data.converter_output_state);
                  Serial.print F("light_output_state:  ");
                  Serial.println (main_data.light_output_state);
                  Serial.print F("pump_output_state:  ");
                  Serial.println (main_data.pump_output_state);

                  Serial.println();

                  Serial.print F("pj_float_sensor_fault_counter:  ");
                  Serial.println (pj_float_sensor_fault_counter);
                  Serial.print F("pj_flow_sensor_fault_counter:  ");
                  Serial.println (pj_flow_sensor_fault_counter);

                  Serial.println();
                  Serial.println();
            
                  vTaskDelay( 5000 / portTICK_PERIOD_MS );
            }
      }
#endif    
//*************************************************************************

// Debounce 
      uint8_t fnDebounce(uint8_t sample){        // антидребезг на основе вертикального счетчика

            static uint8_t state, cnt0, cnt1;
            uint8_t delta, toggle;

            delta = sample ^ state;
            cnt1 = cnt1 ^ cnt0;
            cnt0 = ~cnt0;

            cnt0 &= delta;
            cnt1 &= delta;

            toggle = cnt0 & cnt1;
            state ^= toggle;
            return state;

      }
//*************************************************************************