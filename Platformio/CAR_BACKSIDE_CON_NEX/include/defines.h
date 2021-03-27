#ifndef DEFINES_H
#define DEFINES_H

#include <Arduino.h>

#define DEBUG_GENERAL 0

//colors
#define WHITE       65535
#define YELLOW      65504
#define BLUE        20510
#define GREEN       33792
#define GRAY        27469

//timers
#define TEMP_SENSORS_UPDATE_PERIOD   1000
#define MENU_UPDATE_PERIOD           500

#define NEXTION_BAUD_RATE            115200

//menu pages
#define MAIN_PAGE              0
#define WATER_PAGE             1
#define IOSTATUS_PAGE          2
#define SETTINGS_PAGE          3
#define ONEWIRESET_PAGE        4
#define WATERSET_PAGE          5
#define CONVSET_PAGE           6
#define LIGHTSET_PAGE          7
#define PJONSET_PAGE           8
#define ONEWIRESCANNER_PAGE    9
#define MODBUSSET_PAGE        10
#define BASESET_PAGE          11
#define MAX_PAGES             20 

//modes
#define OFF_MODE               0
#define ON_MODE                1
#define AUTO_MODE              2


//eeprom
#define EEPROM_SETPOINTS_ADDRESS  0
#define MAGIC_KEY                 0x123456//7   //ключь для определения записаны ли уставки в память EEPROM
//#define MAGIC_KEY_PART_1          0x1234567   // вариант с двумя частями один в начале второй в конце
//#define MAGIC_KEY_PART_2          0x7654321

//communications
#define PJON_BUS_PIN         3   // new 
#define ONE_WIRE_PIN         4
#define DE_RS485_PIN         6   // in RS485.h

// water level sensor
#define WATER_LEVEL_LESS_THEN_25      1      
#define WATER_LEVEL_25                2
#define WATER_LEVEL_50                3
#define WATER_LEVEL_75                4
#define WATER_LEVEL_100               5
#define WATER_LEVEL_SENSOR_DEFECTIVE  6

// onewire
#define MAX_TEMP_SENSORS        3
#define INSIDE_SENSOR           1
#define OUTSIDE_SENSOR          2
#define SPARE_SENSOR            3

//inputs
#define DOOR_SWITCH_INPUT_1                 54       //A0
#define PROXIMITY_SENSOR_INPUT_2            55       //A1
#define IGNITION_SWITCH_INPUT_3             56       //A2 
#define LOW_WASHER_WATER_LEVEL_INPUT_4      57       //A3
#define SUPPLY_VOLTAGE_INPUT                A4       //A4
#define SENSORS_VOLTAGE_INPUT               A5       //A5
#define RESISTIVE_SENSOR                    A6       //A6
#define WATER_FLOW_SENSOR                    5       //YF-S201 (пока не используется)
#define BUTTON_ON_BOARD                     10       // кнопка на плате (пока не используется)
#define POWER_OK_FROM_ADM705                41       //
//#define ENCODER_SW							35
//#define ENCODER_DT							36
//#define ENCODER_CLK							37


//outputs
#define WATER_PUMP_OUTPUT_1      22  
#define LIGHT_OUTPUT_2           23  
#define CONVERTER_OUTPUT_3       24  
#define SENSORS_SUPPLY_5v         7  
#define MAIN_SUPPLY_OUT			  9 
#define BUZZER                    2  
#define BUILTIN_LED              13
#define WDT_RESET_OUT             8  

//pjon swbb
#define PJON_MY_ID                    2
#define PJON_MAIN_CONTROLLER_ID       1
#define PJON_WATER_LEVEL_SENSOR_ID    3
#define PJON_WATER_FLOW_SENSOR_ID     4
#define PJON_MAX_NODES                2 // пока два


#define VERSION "ver 1.0"
#define DIVISION_RATIO_VOLTAGE_INPUT  0.0208    // разрешение 0.0025  уможить на коэфициент резистивного делителя 8.2


#define SECOND       1000           //ms  секунда
#define MINUTE       60000          //ms  минута
#define HOUR         3600000        //ms  час


#define INPUTS_UPDATE_PERIOD   50        //ms период обновления опроса входов
#define START_DELAY            3000       //ms задержка опроса входов после подачи питания (пока датчики запустятся)
#define PRX_SENSOR_FEEDBACK_DELAY   500   //ms задержка реакции датчика приближения (для стабильности)
#define WDT_RESET_PERIOD        500000  // us период сброса ADM705 < 1.6 sec

#endif