#include "motor_control.h"
#include "Debug.h"

/*MOTOR_recv Zero_control(int leg_num,int Motor_id){
    MOTOR_send motor_run;
    // receive message struct
    MOTOR_recv motor_r;

    // set the id of motor
    motor_run.id = Motor_id;
    // set the motor type, A1Go1 or B1
    motor_run.motorType = MotorType::A1Go1;
    motor_run.mode = 10;
    motor_run.T = 0.0;
    motor_run.W = 0.0;
    motor_run.Pos = 0.0;
    motor_run.K_P = 0.0;
    motor_run.K_W = 0.0;

    motor_r.motorType = motor_run.motorType;

    // encode data into motor commands
    modify_data(&motor_run);
    if (leg_num==1){
     ONE_LEG.sendRecv(&motor_run, &motor_r);
    }
    else if(leg_num==2){
     TWO_LEG.sendRecv(&motor_run, &motor_r);
    }
    else if(leg_num==3){
     THREE_LEG.sendRecv(&motor_run, &motor_r);
    }
    else if(leg_num==4){
     FOUR_LEG.sendRecv(&motor_run, &motor_r);
    }
    else {
    std::cout << "s输入错误腿号 "  << std::endl;

    }
    extract_data(&motor_r);

    return motor_r;
}*/



using namespace std;
MOTOR_recv position_torque_control(SerialPort& serial, int motor_id,float Postion,float Torque)
{
    
}
MOTOR_recv position_get(SerialPort& serial, int Motor_id)
{
    MOTOR_send motor_run;
    // receive message struct
    MOTOR_recv motor_r;
    // set the id of motor
    motor_run.id = Motor_id;
    // set the motor type, A1Go1 or B1
    motor_run.motorType = MotorType::A1Go1;
    motor_run.mode = 0;
    motor_run.T = 0.0;
    motor_run.W = 0.0;
    motor_run.Pos = 0;
    motor_run.K_W = 0;//12     //28.5     //4.5
    motor_run.K_P = 0;//1       //3     //0.05
    motor_r.motorType = motor_run.motorType;
    modify_data(&motor_run);
    serial.sendRecv(&motor_run, &motor_r);
    extract_data(&motor_r);
    return motor_r;
}

MOTOR_recv position_control_custom(SerialPort& serial,int motor_id,float Position,float k_p,float k_w)
{
MOTOR_send motor_run;
    // receive message struct
    MOTOR_recv motor_r;
    // set the id of motor
    motor_run.id = motor_id;
    // set the motor type, A1Go1 or B1
    motor_run.motorType = MotorType::A1Go1;
    motor_run.mode = 10;
    motor_run.T = 0.0;
    motor_run.W = 0.0;
    motor_run.Pos = Position;
    motor_run.K_W = k_w;//12     //28.5     //4.5
    motor_run.K_P = k_p;//1       //3     //0.05

    motor_r.motorType = motor_run.motorType;

    modify_data(&motor_run);
    serial.sendRecv(&motor_run, &motor_r);
    extract_data(&motor_r);

    /////////////////////////////
    //motor_r.Pos = Position;
    /////////////////////////////

    return motor_r;
}

MOTOR_recv postion_control(SerialPort& serial, int Motor_id, float Position)
{
    MOTOR_send motor_run;
    // receive message struct
    MOTOR_recv motor_r;
    // set the id of motor
    motor_run.id = Motor_id;
    // set the motor type, A1Go1 or B1
    motor_run.motorType = MotorType::A1Go1;
    motor_run.mode = 10;
    motor_run.T = 0.0;
    motor_run.W = 0.0;
    motor_run.Pos = Position;
    motor_run.K_W = 4;//12     //28.5     //4.5
    motor_run.K_P = 0.1;//1       //3     //0.05

    motor_r.motorType = motor_run.motorType;
    //cout<<"position"<<endl;
    // encode data into motor commands
   // modify_data(&motor_run);
   // serial.sendRecv(&motor_run, &motor_r);
  //  extract_data(&motor_r);
    //if(Motor_id == 2)
    //Debug::Output("leg:%d,position:%f\n",Motor_id,Position);
    //cout<<"leg:"<<Motor_id<<",position:"<<Position<<endl;
    //usleep(100000);
    return motor_r;
}


MOTOR_recv postion_control_high(SerialPort& serial, int Motor_id, float Position)
{
    MOTOR_send motor_run;
    // receive message struct
    MOTOR_recv motor_r;

    // set the id of motor
    motor_run.id = Motor_id;
    // set the motor type, A1Go1 or B1
    motor_run.motorType = MotorType::A1Go1;
    motor_run.mode = 10;
    motor_run.T = 0.0;
    motor_run.W = 0.0;
    motor_run.Pos = Position;
    motor_run.K_W = 12;//12     //28.5     //4.5
    motor_run.K_P = 1;//1       //3     //0.05

    motor_r.motorType = motor_run.motorType;

    // encode data into motor commands
    //modify_data(&motor_run);
    //serial.sendRecv(&motor_run, &motor_r);

    //extract_data(&motor_r);
    return motor_r;
}



MOTOR_recv velocity_control(SerialPort& serial, int Motor_id, float Velocity) {
    MOTOR_send motor_run;
    // receive message struct
    MOTOR_recv motor_r;

    // set the id of motor
    motor_run.id = Motor_id;
    // set the motor type, A1Go1 or B1
    motor_run.motorType = MotorType::A1Go1;
    motor_run.mode = 10;
    motor_run.T = 0.0;
    motor_run.W = Velocity * 9.1;
    motor_run.Pos = 0.0;
    motor_run.K_P = 0.0;
    motor_run.K_W = 10;

    motor_r.motorType = motor_run.motorType;

    // encode data into motor commands
    //modify_data(&motor_run);

    //serial.sendRecv(&motor_run, &motor_r);
    //extract_data(&motor_r);
    //usleep(10000); ???
    return motor_r;
}

MOTOR_recv torque_control(SerialPort& serial, int Motor_id, float Torque) {
    MOTOR_send motor_run;
    // receive message struct
    MOTOR_recv motor_r;

    // set the id of motor
    motor_run.id = Motor_id;
    // set the motor type, A1Go1 or B1
    motor_run.motorType = MotorType::A1Go1;
    motor_run.mode = 10;
    motor_run.T = Torque;//max==33.5
    motor_run.W = 0.0;

    motor_run.Pos = 0.0;
    motor_run.K_P = 0;
    motor_run.K_W = 10;



    motor_r.motorType = motor_run.motorType;

    // encode data into motor commands
    //modify_data(&motor_run);

    //serial.sendRecv(&motor_run, &motor_r);
    //extract_data(&motor_r);
    //usleep(10000);//?

    return motor_r;
}

void postion_control_new(SerialPort& serial, int Motor_id, float Position) {
    MOTOR_send motor_run;
    // receive message struct
    MOTOR_recv motor_r;

    // set the id of motor
    motor_run.id = Motor_id;
    // set the motor type, A1Go1 or B1
    motor_run.motorType = MotorType::A1Go1;
    motor_run.mode = 10;
    motor_run.T = 0.0;
    motor_run.W = 0.0;
    motor_run.Pos = Position;
    motor_run.K_P = 0.02;
    motor_run.K_W = 1.5;

    motor_r.motorType = motor_run.motorType;

    // encode data into motor commands
    modify_data(&motor_run);
    serial.sendRecv(&motor_run, &motor_r);
}

/*void motor_control( position_angel angel1,position_angel angel2,position_angel angel3,position_angel angel4)
{
//    printf("1腿%f %f %f\n",angel1.alfa,angel1.beta,angel1.gama);
//    printf("2腿%f %f %f\n",angel2.alfa,angel2.beta,angel2.gama);
//    printf("3腿%f %f %f\n",angel3.alfa,angel3.beta,angel3.gama);
//    printf("4腿%f %f %f\n\n",angel4.alfa,angel4.beta,angel4.gama);
    float K_pos,D_pos,X_pos;

    K_pos=K1_origin_pos+(-1)*angel1.gama*9.1;
    D_pos=D1_origin_pos+angel1.alfa*9.1;
    X_pos=X1_origin_pos+angel1.beta*9.1;
//    printf("1号腿髋!!!!!!!!!!!!!!%f\n",K_pos);
//    printf("1号腿大!!!!!!!!!!!!!!%f\n",D_pos);
//    printf("1号腿小!!!!!!!!!!!!!!%f\n\n",X_pos);
    if((K_pos<K1_origin_pos+9.1*pi/4)&& (K_pos>K1_origin_pos-9.1*pi/4))
    {
    MOTOR_recv r11=postion_control(angel1.leg,K_Joint,K_pos);
 //       printf("K1完成\n");
    }

    else
    {
        printf("K1_PROTECT_ERROR!!!!!!!!!\n");
        printf("1腿%f %f %f\n",angel1.alfa,angel1.beta,angel1.gama);
    }
    if((D_pos<D1_origin_pos+9.1*pi) && (D_pos>D1_origin_pos-9.1*pi))
    {
       MOTOR_recv r12=postion_control(angel1.leg,D_Joint,D_pos);
 //       printf("D1完成\n");
    }
    else
    {
        printf("D1_PROTECT_ERROR!!!!!!!!!\n");
        printf("1腿%f %f %f\n",angel1.alfa,angel1.beta,angel1.gama);
    }
    if((X_pos>X1_origin_pos+9.1*pi/9) && (X_pos<X1_origin_pos+9.1*pi))
    {
       MOTOR_recv r13=postion_control(angel1.leg,X_Joint,X_pos);
 //      printf("X1完成\n");
    }
    else
    {
        printf("X1_PROTECT_ERROR!!!!!!!!!\n");
        printf("1腿%f %f %f\n",angel1.alfa,angel1.beta,angel1.gama);
    }



    K_pos=K2_origin_pos+angel2.gama*9.1;
    D_pos=D2_origin_pos+angel2.alfa*9.1;
    X_pos=X2_origin_pos+angel2.beta*9.1;
//    printf("2号腿髋!!!!!!!!!!!!!!%f\n",K_pos);
//    printf("2号腿大!!!!!!!!!!!!!!%f\n",D_pos);
//    printf("2号腿小!!!!!!!!!!!!!!%f\n\n",X_pos);
    if((K_pos<K2_origin_pos+9.1*pi/4)&& (K_pos>K2_origin_pos-9.1*pi/4))
    {
        MOTOR_recv r21=postion_control(angel2.leg,K_Joint,K_pos);
  //      printf("K2完成\n");
    }

    else
    {
        printf("K2_PROTECT_ERROR!!!!!!!!!\n");
        printf("2腿%f %f %f\n",angel2.alfa,angel2.beta,angel2.gama);
    }
    if((D_pos<D2_origin_pos+9.1*pi) && (D_pos>D2_origin_pos-9.1*pi))
    {
        MOTOR_recv r22=postion_control(angel2.leg,D_Joint,D_pos);
 //       printf("D2完成\n");
    }
    else
    {
        printf("D2_PROTECT_ERROR!!!!!!!!!\n");
        printf("2腿%f %f %f\n",angel2.alfa,angel2.beta,angel2.gama);
    }
    if((X_pos>X2_origin_pos-9.1*pi) && (X_pos<X2_origin_pos-9.1*pi/9))
    {
        MOTOR_recv r23=postion_control(angel2.leg,X_Joint,X_pos);
 //       printf("X2完成\n");
    }
    else
    {
        printf("X2_PROTECT_ERROR!!!!!!!!!\n");
        printf("2腿%f %f %f\n",angel2.alfa,angel2.beta,angel2.gama);
    }


    K_pos=K3_origin_pos+(-1)*angel3.gama*9.1;
    D_pos=D3_origin_pos+angel3.alfa*9.1;
    X_pos=X3_origin_pos+angel3.beta*9.1;
//    printf("3号腿髋!!!!!!!!!!!!!!%f\n",K_pos);

//    printf("3号腿小!!!!!!!!!!!!!!%f\n\n",X_pos);

    if((K_pos<K3_origin_pos+9.1*pi/4)&& (K_pos>K3_origin_pos-9.1*pi/4))
    {
       MOTOR_recv r31=postion_control(angel3.leg,K_Joint,K_pos);
  //     printf("K3完成\n");

    }

    else
    {
        printf("K3_PROTECT_ERROR!!!!!!!!!\n");
        printf("3腿%f %f %f\n",angel3.alfa,angel3.beta,angel3.gama);
    }
    if((D_pos<D3_origin_pos+9.1*pi) && (D_pos>D3_origin_pos-9.1*pi))
    {
         MOTOR_recv r32=postion_control(angel3.leg,D_Joint,D_pos);
  //       printf("D3完成\n");
    }
    else
    {
        printf("D3_PROTECT_ERROR!!!!!!!!!\n");
        printf("3腿%f %f %f\n",angel3.alfa,angel3.beta,angel3.gama);
    }
    if((X_pos>X3_origin_pos-9.1*pi) && (X_pos<X3_origin_pos-9.1*pi/9))
    {
        MOTOR_recv r33=postion_control(angel3.leg,X_Joint,X_pos);
  //      printf("X3完成\n");
    }
    else
    {
        printf("X3_PROTECT_ERROR!!!!!!!!!\n");
        printf("3腿%f %f %f\n",angel3.alfa,angel3.beta,angel3.gama);
    }
//printf("3号腿大!!!!!!!!!!!!!!%f",D_pos);

    K_pos=K4_origin_pos+angel4.gama*9.1;
    D_pos=D4_origin_pos+angel4.alfa*9.1;
    X_pos=X4_origin_pos+angel4.beta*9.1;
//    printf("4号腿髋!!!!!!!!!!!!!!%f\n",K_pos);

    //printf("4号腿大!!!!!!!!!!!!!!%f\n",D_pos);
//    printf("4号腿小!!!!!!!!!!!!!!%f\n\n",X_pos);


    if((K_pos<K4_origin_pos+9.1*pi/4)&& (K_pos>K4_origin_pos-9.1*pi/4))
    {
        MOTOR_recv r41=postion_control(angel4.leg,K_Joint,K_pos);
 //       printf("K4完成\n");

    }

    else
    {
        printf("4腿%f %f %f\n",angel4.alfa,angel4.beta,angel4.gama);
        printf("K4_PROTECT_ERROR!!!!!!!!!\n");
    }
    if((D_pos<D4_origin_pos+9.1*pi) && (D_pos>D4_origin_pos-9.1*pi))
    {
       MOTOR_recv r42=postion_control(angel4.leg,D_Joint,D_pos);
   //    printf("D4完成\n");
    }
    else
    {
        printf("4腿%f %f %f\n",angel4.alfa,angel4.beta,angel4.gama);
        printf("D4_PROTECT_ERROR!!!!!!!!!\n");
    }
    if((X_pos>X4_origin_pos+9.1*pi/9) && (X_pos<X4_origin_pos+9.1*pi))
    {
        MOTOR_recv r43=postion_control(angel4.leg,X_Joint,X_pos);
  //      printf("X4完成\n");
    }
    else
    {
        printf("4腿%f %f %f\n",angel4.alfa,angel4.beta,angel4.gama);
        printf("X4_PROTECT_ERROR!!!!!!!!!\n");
    }
}


void motor_control_high( position_angel angel1,position_angel angel2,position_angel angel3,position_angel angel4)
{
//    printf("1腿%f %f %f\n",angel1.alfa,angel1.beta,angel1.gama);
//    printf("2腿%f %f %f\n",angel2.alfa,angel2.beta,angel2.gama);
//    printf("3腿%f %f %f\n",angel3.alfa,angel3.beta,angel3.gama);
//    printf("4腿%f %f %f\n\n",angel4.alfa,angel4.beta,angel4.gama);
    float K_pos,D_pos,X_pos;

    K_pos=K1_origin_pos+(-1)*angel1.gama*9.1;
    D_pos=D1_origin_pos+angel1.alfa*9.1;
    X_pos=X1_origin_pos+angel1.beta*9.1;
//    printf("1号腿髋!!!!!!!!!!!!!!%f\n",K_pos);
//    printf("1号腿大!!!!!!!!!!!!!!%f\n",D_pos);
//    printf("1号腿小!!!!!!!!!!!!!!%f\n\n",X_pos);
    if((K_pos<K1_origin_pos+9.1*pi/4)&& (K_pos>K1_origin_pos-9.1*pi/4))
    {
 //       MOTOR_recv r11=postion_control_high(angel1.leg,K_Joint,K_pos);
  //      printf("K1完成\n");
    }

    else
    {
        printf("K1_PROTECT_ERROR!!!!!!!!!\n");
        printf("1腿%f %f %f\n",angel1.alfa,angel1.beta,angel1.gama);
    }
    if((D_pos<D1_origin_pos+9.1*pi) && (D_pos>D1_origin_pos-9.1*pi))
    {
//        MOTOR_recv r12=postion_control_high(angel1.leg,D_Joint,D_pos);
//       printf("D1完成\n");
    }
    else
    {
        printf("D1_PROTECT_ERROR!!!!!!!!!\n");
        printf("1腿%f %f %f\n",angel1.alfa,angel1.beta,angel1.gama);
    }
    if((X_pos>X1_origin_pos+9.1*pi/9) && (X_pos<X1_origin_pos+9.1*pi))
    {
 //      MOTOR_recv r13=postion_control_high(angel1.leg,X_Joint,X_pos);
//       printf("X1完成\n");
    }
    else
    {
        printf("X1_PROTECT_ERROR!!!!!!!!!\n");
        printf("1腿%f %f %f\n",angel1.alfa,angel1.beta,angel1.gama);
    }



    K_pos=K2_origin_pos+angel2.gama*9.1;
    D_pos=D2_origin_pos+angel2.alfa*9.1;
    X_pos=X2_origin_pos+angel2.beta*9.1;
//    printf("2号腿髋!!!!!!!!!!!!!!%f\n",K_pos);
//    printf("2号腿大!!!!!!!!!!!!!!%f\n",D_pos);
//    printf("2号腿小!!!!!!!!!!!!!!%f\n\n",X_pos);
    if((K_pos<K2_origin_pos+9.1*pi/4)&& (K_pos>K2_origin_pos-9.1*pi/4))
    {
//        MOTOR_recv r21=postion_control_high(angel2.leg,K_Joint,K_pos);
//        printf("K2完成\n");
    }

    else
    {
        printf("K2_PROTECT_ERROR!!!!!!!!!\n");
        printf("2腿%f %f %f\n",angel2.alfa,angel2.beta,angel2.gama);
    }
    if((D_pos<D2_origin_pos+9.1*pi) && (D_pos>D2_origin_pos-9.1*pi))
    {
//        MOTOR_recv r22=postion_control_high(angel2.leg,D_Joint,D_pos);
//        printf("D2完成\n");
    }
    else
    {
        printf("D2_PROTECT_ERROR!!!!!!!!!\n");
        printf("2腿%f %f %f\n",angel2.alfa,angel2.beta,angel2.gama);
    }
    if((X_pos>X2_origin_pos-9.1*pi) && (X_pos<X2_origin_pos-9.1*pi/9))
    {
//        MOTOR_recv r23=postion_control_high(angel2.leg,X_Joint,X_pos);
//        printf("X2完成\n");
    }
    else
    {
        printf("X2_PROTECT_ERROR!!!!!!!!!\n");
        printf("2腿%f %f %f\n",angel2.alfa,angel2.beta,angel2.gama);
    }


    K_pos=K3_origin_pos+(-1)*angel3.gama*9.1;
    D_pos=D3_origin_pos+angel3.alfa*9.1;
    X_pos=X3_origin_pos+angel3.beta*9.1;
//    printf("3号腿髋!!!!!!!!!!!!!!%f\n",K_pos);

//    printf("3号腿小!!!!!!!!!!!!!!%f\n\n",X_pos);

    if((K_pos<K3_origin_pos+9.1*pi/4)&& (K_pos>K3_origin_pos-9.1*pi/4))
    {
//       MOTOR_recv r31=postion_control_high(angel3.leg,K_Joint,K_pos);
//       printf("K3完成\n");

    }

    else
    {
        printf("K3_PROTECT_ERROR!!!!!!!!!\n");
        printf("3腿%f %f %f\n",angel3.alfa,angel3.beta,angel3.gama);
    }
    if((D_pos<D3_origin_pos+9.1*pi) && (D_pos>D3_origin_pos-9.1*pi))
    {
//         MOTOR_recv r32=postion_control_high(angel3.leg,D_Joint,D_pos);
//         printf("D3完成\n");
    }
    else
    {
        printf("D3_PROTECT_ERROR!!!!!!!!!\n");
        printf("3腿%f %f %f\n",angel3.alfa,angel3.beta,angel3.gama);
    }
    if((X_pos>X3_origin_pos-9.1*pi) && (X_pos<X3_origin_pos-9.1*pi/9))
    {
 //       MOTOR_recv r33=postion_control_high(angel3.leg,X_Joint,X_pos);
  //      printf("X3完成\n");
    }
    else
    {
        printf("X3_PROTECT_ERROR!!!!!!!!!\n");
        printf("3腿%f %f %f\n",angel3.alfa,angel3.beta,angel3.gama);
    }
//printf("3号腿大!!!!!!!!!!!!!!%f",D_pos);

    K_pos=K4_origin_pos+angel4.gama*9.1;
    D_pos=D4_origin_pos+angel4.alfa*9.1;
    X_pos=X4_origin_pos+angel4.beta*9.1;
//    printf("4号腿髋!!!!!!!!!!!!!!%f\n",K_pos);

    //printf("4号腿大!!!!!!!!!!!!!!%f\n",D_pos);
//    printf("4号腿小!!!!!!!!!!!!!!%f\n\n",X_pos);


    if((K_pos<K4_origin_pos+9.1*pi/4)&& (K_pos>K4_origin_pos-9.1*pi/4))
    {
  //      MOTOR_recv r41=postion_control_high(angel4.leg,K_Joint,K_pos);
    //    printf("K4完成\n");

    }

    else
    {
        printf("4腿%f %f %f\n",angel4.alfa,angel4.beta,angel4.gama);
        printf("K4_PROTECT_ERROR!!!!!!!!!\n");
    }
    if((D_pos<D4_origin_pos+9.1*pi) && (D_pos>D4_origin_pos-9.1*pi))
    {
 //      MOTOR_recv r42=postion_control_high(angel4.leg,D_Joint,D_pos);
 //      printf("D4完成\n");
    }
    else
    {
        printf("4腿%f %f %f\n",angel4.alfa,angel4.beta,angel4.gama);
        printf("D4_PROTECT_ERROR!!!!!!!!!\n");
    }
    if((X_pos>X4_origin_pos+9.1*pi/9) && (X_pos<X4_origin_pos+9.1*pi))
    {
  //      MOTOR_recv r43=postion_control_high(angel4.leg,X_Joint,X_pos);
  //      printf("X4完成\n");
    }
    else
    {
        printf("4腿%f %f %f\n",angel4.alfa,angel4.beta,angel4.gama);
        printf("X4_PROTECT_ERROR!!!!!!!!!\n");
    }
}
*/
