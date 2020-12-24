#ifndef INIT_FUNCTIONS_H
#define INIT_FUNCTIONS_H

#include <Arduino.h>
#include "defines.h"
#include "variables.h"


void fnIOInit(void) {
    
//inputs declaration
pinMode(DOOR_SWITCH_INPUT_1, INPUT_PULLUP);
pinMode(PROXIMITY_SENSOR_INPUT_2, INPUT_PULLUP);
pinMode(IGNITION_SWITCH_INPUT_3, INPUT);
pinMode(LOW_WASHER_WATER_LEVEL_INPUT_4, INPUT_PULLUP);//
pinMode(WATER_FLOW_SENSOR, INPUT_PULLUP);//
pinMode(BUTTON_ON_BOARD, INPUT_PULLUP);//


//outputs declaration
pinMode(WATER_PUMP_OUTPUT_1, OUTPUT);
pinMode(LIGHT_OUTPUT_2, OUTPUT);
pinMode(CONVERTER_OUTPUT_3, OUTPUT);
pinMode(SENSORS_SUPPLY_5v, OUTPUT);
pinMode(MAIN_SUPPLY_OUT, OUTPUT);
pinMode(BUZZER , OUTPUT);
pinMode(BUILTIN_LED, OUTPUT); 
pinMode(WDT_RESET_OUT, OUTPUT); 

digitalWrite(WATER_PUMP_OUTPUT_1, LOW);
digitalWrite(LIGHT_OUTPUT_2, LOW);
digitalWrite(CONVERTER_OUTPUT_3, LOW);
digitalWrite(SENSORS_SUPPLY_5v, LOW);
digitalWrite(MAIN_SUPPLY_OUT, LOW);
digitalWrite(BUZZER, LOW);
digitalWrite(BUILTIN_LED, LOW);
digitalWrite(WDT_RESET_OUT, LOW);
}


void fnDefaultSetpointsInit(void){

    setpoints_data.magic_key = MAGIC_KEY ;        
    setpoints_data.pump_off_delay = 5; // сек
    setpoints_data.flow_sensor_correction = 0;
    setpoints_data.water_tank_capacity = 35; // литр
    setpoints_data.water_level_liter = 0;
    setpoints_data.converter_off_delay = 3; // min
    setpoints_data.converter_shutdown_delay = 5; // min
    setpoints_data.converter_voltage_off = 115; // 11.5 вольт
    setpoints_data.converter_voltage_on = 130; //13 вольт
    setpoints_data.convertet_out_mode = AUTO_MODE;
    setpoints_data.light_off_delay = 10; // мин
    setpoints_data.pjon_ID = 2;
    setpoints_data.pjon_float_fault_timer = 5; // сек
    setpoints_data.pjon_transmitt_period = 2;  // сек
    setpoints_data.mb_slave_ID = 2;
    setpoints_data.mb_baud_rate = 1; //1-9600, 2-19200, 3-38400 ...
    setpoints_data.buzzer_out_mode = ON_MODE;
    setpoints_data.scrreen_off_delay = 5; // 5 секунд
    setpoints_data.buzzer_melody_2 = 2;
    setpoints_data.buzzer_melody_3 = 3;
    setpoints_data.buzzer_melody_4 = 4;
    setpoints_data.voltage_correction = 127; // 
    setpoints_data.shutdown_delay = 30; //30 минут
    setpoints_data.lcd_brightness = 100;   
    setpoints_data.spare_byte_1 = 0;
    setpoints_data.spare_byte_2 = 0;
    
}

#endif