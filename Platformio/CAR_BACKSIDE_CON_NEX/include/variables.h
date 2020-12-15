#ifndef VARIABLES_H
#define VARIABLES_H


//uint16_t analog_input_5_source;                 // значение АЦП на пине А6
//uint16_t analog_input_6_source;                 // значение АЦП на пине А7

uint16_t pj_fault_counter_1;
uint16_t pj_fault_counter_2;

bool flag_door_switch_old_state;                     // предыдущее состояние двери
bool proximity_sensor_old_state;              // предыдущее состояние датчика приближения

bool flag_ds18b20_update;                     // флаг обновлени чтения температуры с ds18b20 (пока не используется)


//Pjon variables 
/*
struct PjonSend {                     // структура для отправки данных по протоколу PJON
  
  } data_send_to_main_controller;
*/

uint16_t send_to_ID ;               //   идентификатор устройства которому передаются данные
uint16_t  receive_from_ID;              //   идентификатор устройства от которого пришли данные
uint16_t  pjon_TX__float_sensor_response;             //  результат передачи PJON 
uint16_t  pjon_TX__flow_sensor_response;             //  результат передачи PJON 
uint16_t  pjon_RX_response;            //  результат приёма PJON 

uint8_t pjon_sender_cnt; // счетчик для поочередной передачи нескольким адресатам, инкрементируется при передаче
uint8_t pjon_float_sensor_fault_cnt;   // счетчик ошибки связи с датчиком уровня, добавляем при каждой передаче и сбрасываем при удачном приёме 
uint8_t pjon_flow_sensor_fault_cnt;   // счетчик ошибки связи с датчиком протечки, добавляем при каждой передаче и сбрасываем при удачном приёме 
bool flag_pjon_float_sensor_connected;
bool flag_pjon_float_sensor_connected_old_state;
bool flag_pjon_flow_sensor_connected;
bool flag_pjon_flow_sensor_connected_old_state;

//pjon receive from water level sensor & water flow sensor
struct PjonReceive {                  // структура принятых данных от датчиков уровня воды по протоколу PJON
uint8_t data;
} pjon_wls_percent_receive,pjon_wfs_liter_receive;



//water level
uint8_t pjon_wls_percent_old;   // предыдущее значение уровня воды принятое от датчика уровня
uint8_t pjon_wls_liter_old;     // предыдущее значение количества воды принятое от датчика уровня

//buzzer

// light
bool flag_timer_light_delay_off_started;    // флаг состояния таймера выключения света


// menu
//------------Setpoints variables --------------------------------------

  struct Setpoints {                 // структура для уставок         
    uint32_t magic_key;        
    uint8_t pump_off_delay;
    uint8_t flow_sensor_correction; // 127 - нолевая коррекция
    uint8_t water_tank_capacity;
    uint8_t water_level_liter;
    uint8_t converter_off_delay;
    uint8_t converter_shutdown_delay;
    uint8_t converter_voltage_off; // дробное со смещённой вправо точкой 12.7в = 127,  13.2в =132 и т.д.
    uint8_t converter_voltage_on;
    uint8_t convertet_out_mode;
    uint8_t light_off_delay;
    uint8_t light_out_mode;
    uint8_t pjon_ID;
    uint8_t pjon_float_fault_timer;
    uint8_t pjon_transmitt_period; 
    uint8_t mb_slave_ID;
    uint8_t mb_baud_rate;
    uint8_t buzzer_out_mode;
    uint8_t scrreen_off_delay; // 
    uint8_t buzzer_melody_2; // NOT USED
    uint8_t buzzer_melody_3; // NOT USED
    uint8_t buzzer_melody_4; // NOT USED
    uint8_t voltage_correction; // 127 - нолевая коррекция
    uint8_t shutdown_delay;
    uint8_t lcd_brightness;   
    uint8_t spare_byte_1;
    uint8_t spare_byte_2;
    uint8_t sensors_ID_array [MAX_TEMP_SENSORS][8];
    uint8_t sensors_select_array[MAX_TEMP_SENSORS ]; // inside, outside, spare
  } setpoints_data, default_setpoints_data, old_setpoints_data;


//--------- end setpoints variables ---------------------------------------

//-- Main data --------------------------------------------------------
  struct MyData {
    float battery_voltage;        // напряжени бортсети ( например 124 это 12.4в)
    float outside_temperature;     //  наружная температура
    float inside_temperature;       // температура внутри
    float spare_temperature;       // температура третьего датчика
    uint8_t water_level_percent;
    uint8_t water_level_liter;
    bool door_switch_state;        // состояние концевика задней двери
    bool proximity_sensor_state;   // состояние датчика приближения
    bool ignition_switch_state;
    bool converter_output_state;        // состояние выхода управления инвертором 12/220в
    bool light_output_state ;    //состояние выхода освещения 
    bool pump_output_state;     //состояние выхода насоса
    bool sensors_supply_output_state;
    bool main_supply_output_state;
    bool wdt_reset_output_state;
    bool screen_sleep_mode;
    bool low_washer_water_level;
    uint16_t mb_rates [6]={4800, 7200, 9600, 19200, 38400, 57600};
  } main_data;

//----- end main data --------------------------------

//********** MENU VARIABLES ***********************************


//*************************************************************


#endif
 