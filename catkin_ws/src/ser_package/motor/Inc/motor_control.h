#ifndef CONTROL_H_INCLUDED
#define CONTROL_H_INCLUDED

#include "unitreeMotor.h"
#include "motor_msg.h"  // 电机通信协议
#include "SerialPort.h"
MOTOR_recv position_torque_control(SerialPort& serial, int motor_id,float Postion,float Torque);
MOTOR_recv position_get(SerialPort& port, int Motor_id);
MOTOR_recv postion_control(SerialPort& serial,int Motor_id, float Position);
MOTOR_recv position_control_custom(SerialPort& serial,int Motor_id, float Position,float k_p,float k_w);

MOTOR_recv postion_control_high(SerialPort& serial,int Motor_id, float Position);

MOTOR_recv velocity_control(SerialPort& serial,int Motor_id,float Velicity);
MOTOR_recv torque_control(SerialPort& serial,int Motor_id,float Torque);
//MOTOR_recv Zero_control(SerialPort& serial,int Motor_id);
//void motorstop(SerialPort& serial,int Motor_id);
//void postion_control_new(int leg_num,int Motor_id, float Position);


#endif // CONTROL_H_INCLUDED
