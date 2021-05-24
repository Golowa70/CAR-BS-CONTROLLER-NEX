#ifndef VARIABLES_H
#define VARIABLES_H

//*********** Pjon variables ********************************************************
uint16_t receive_from_ID;                //   идентификатор устройства от которого пришли данные
uint16_t pjon_TX_water_sensor_response; //  результат передачи PJON
uint16_t pjon_RX_response;               //  результат приёма PJON
uint8_t pjon_sensor_fault_cnt;     // счетчик ошибки связи с датчиком уровня, добавляем при каждой передаче и сбрасываем при удачном приёме

bool flag_pjon_water_sensor_connected;
bool flag_pjon_water_sensor_connected_old_state;
bool flag_pjon_flow_sensor_connected;
bool flag_pjon_flow_sensor_connected_old_state;

//pjon receive from water level sensor & water flow sensor
struct PjonReceive
{ // структура принятых данных от датчиков уровня воды по протоколу PJON
  uint8_t value;
} pjon_sensor_receive_data;

//pjon transmitt
/*
  struct PjonSend {                     // структура для отправки данных по протоколу PJON
    
    } data_send_to_main_controller;
  */

//*****************************************************************************************************

//*********** LIGHT VARIABLES *************************************************************************
bool flag_timer_light_delay_off_started; // флаг состояния таймера выключения света

//*****************************************************************************************************

//*********** Setpoints variables *********************************************************************

struct Setpoints
{ // структура для уставок
  uint32_t magic_key;
  uint8_t pump_off_delay;
  uint8_t resistive_sensor_correction; // 0- нолевая коррекция
  uint8_t water_tank_capacity;
  uint8_t water_level_liter;
  uint8_t lowUconverter_off_delay;
  uint8_t converter_shutdown_delay;
  uint8_t converter_voltage_off; // дробное со смещённой вправо точкой 12.7в = 127,  13.2в =132 и т.д.
  uint8_t converter_voltage_on;
  uint8_t convertet_out_mode;
  uint8_t light_off_delay;
  uint8_t light_out_mode;
  uint8_t pjon_ID;
  uint8_t pjon_sensor_fault_timer;
  uint8_t pjon_transmitt_period;
  uint8_t mb_slave_ID;
  uint8_t mb_baud_rate;
  uint8_t buzzer_out_mode;
  uint8_t scrreen_off_delay;  //
  uint8_t resistive_sensor_nominal;    // 
  uint8_t buzzer_melody_3;    // NOT USED
  uint8_t buzzer_melody_4;    // NOT USED
  uint8_t voltage_correction; // 127 - нолевая коррекция
  uint8_t shutdown_delay;
  uint8_t lcd_brightness;
  uint8_t logo_selection;
  uint8_t water_sensor_type_selection;
  uint8_t sensors_ID_array[MAX_TEMP_SENSORS][8];
  uint8_t sensors_select_array[MAX_TEMP_SENSORS]; // inside, outside, spare
} setpoints_data, default_setpoints_data, old_setpoints_data;
// 57 byte
//********** end setpoints variables ******************************************************************

//*********** Main data *******************************************************************************
struct MyData
{
  float battery_voltage;          // напряжени бортсети ( например 124 это 12.4в)
  float outside_temperature;      //  наружная температура
  float inside_temperature;       // температура внутри
  float spare_temperature;        // температура третьего датчика(пока не используется)
  float sensors_supply_voltage;   // напряжение питания датчиков 5в
  float res_sensor_resistance;    // сопротивление резистивного датчика
  uint8_t water_level_percent;    // уровень воды в процентах
  uint8_t water_level_liter;      // уровень воды в литрах
  bool door_switch_state;         // состояние концевика задней двери
  bool proximity_sensor_state;    // состояние датчика приближения
  bool ignition_switch_state;     // состояние входа зажигания
  bool converter_output_state;    // состояние выхода управления инвертором 12/220в
  bool light_output_state;        //состояние выхода освещения
  bool pump_output_state;         //состояние выхода насоса
  bool sensors_supply_output_state; // состояние выхода управления питанием сенсоров 5в
  bool main_supply_output_state;  // состояние выхода управления общим питанием 7.5в
  bool wdt_reset_output_state;    //состояние выхода сброса внешнего WDT
  bool screen_sleep_mode;         // флаг спящего режима экрана Nextion
  bool low_washer_water_level;    // низкий уровень воды в бачке омывателя
  bool common_alarm;
  uint16_t mb_rates[6] = {4800, 7200, 9600, 19200, 38400, 57600};   // скорость связи по протоколу ModBus
} main_data;

//****** end main data **************************************************************

//********** MENU VARIABLES *********************************************************
uint8_t current_page = MAX_PAGES;
uint8_t current_item;
uint8_t *variable_value = NULL;
bool flag_value_changed;
uint8_t var_min_value;
uint8_t var_max_value;
String tempString = "";
String tempString2;
//***********************************************************************************

//********** ONE WIRE VARIABLES *****************************************************
bool flag_ow_scanned;
bool flag_ow_scan_to_start;
bool flag_ds18b20_update; // флаг обновлени чтения температуры с ds18b20
//***********************************************************************************

//********* OTHER VARIABLES *********************************************************
bool flag_door_switch_old_state; // предыдущее состояние двери
bool proximity_sensor_old_state; // предыдущее состояние датчика приближения

bool flag_convOff_due_voltage;    // флаг что конветер был выключен по напряжению
bool flag_convOff_due_ign_switch; // флаг что конветер был выключен по таймеру после выключения зажигания

uint8_t inputs_undebounced_sample = 0;
uint8_t inputs_debounced_state = 0;

bool flag_error_present = false;
//***********************************************************************************

//********* ERROR LOG ***************************************************************
struct ErrLog
{
  uint16_t sens_supply_error_cnt;      // счетчик ошбок питания сенсоров
  uint16_t pj_water_sensor_error_cnt;  // счетчик запросов без ответа датчика уровня
  uint16_t temp_sensors_error_cnt;     // счетчик ошибок шины 1Wire
  uint16_t resist_sensor_error_cnt;    // счетчик ошибок 
} ErrorLog;

struct Alarms
{
  bool sens_supply;
  bool pj_water_sensor;
  bool temp_sensors;
  bool resist_sensor;
  bool common;
} present_alarms,old_alarms;

#endif
