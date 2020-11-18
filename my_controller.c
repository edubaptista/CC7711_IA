/*
 * File:        my_controller.c
 * Date:		06/11/2020
 * Description:	Controlador Robô - Webots - CC7711 - Inteligencia Artif. e Robotica - Prof. Ricardo Destro
 * Author:		Eduardo Baptista dos Santos - RA: 22.217.017-7
 * Modifications: 	Alterações no módulo de controle do robô para a detecção de blocos móveis no mundo
 */
 
#include <stdio.h> 
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>
#include <webots/accelerometer.h>

#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10
#define MAX_VELOC 5 

WbDeviceTag Leds[QtddLeds];

void controlaLeds(int x, int y)
{
    int i = 0;
    while(i < x){
       wb_led_set(Leds[i], y);
       i++;
    }
}


    
int main(int argc, char** argv)
{
    int i = 0;
    int qtdBlocosMoveis = 0;
    double LeituraSensorProx[QtddSensoresProx];
    double moveParaDireita = 1.0, moveParaEsquerda = 1.0;
                                                                                                         
    wb_robot_init();

    WbDeviceTag MotorEsquerdo, MotorDireito;

    MotorEsquerdo = wb_robot_get_device("left wheel motor");
    MotorDireito = wb_robot_get_device("right wheel motor");

    WbNodeRef robot_node = wb_supervisor_node_get_from_def("ePuck"); //captura o supervisor

    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation"); //identifica o campo de posição

    const double *posicaoRobo; //variáel que vai receber a posição do robo
    
    wb_motor_set_position(MotorEsquerdo, INFINITY);
    wb_motor_set_position(MotorDireito, INFINITY);

    wb_motor_set_velocity(MotorEsquerdo, 0);
    wb_motor_set_velocity(MotorDireito, 0);

    WbDeviceTag SensorProx[QtddSensoresProx];

    Leds[0] = wb_robot_get_device("led0");
    wb_led_set(Leds[0], 0);
    Leds[1] = wb_robot_get_device("led1");
    wb_led_set(Leds[1], 0);
    Leds[2] = wb_robot_get_device("led2");
    wb_led_set(Leds[2], 0);
    Leds[3] = wb_robot_get_device("led3");
    wb_led_set(Leds[3], 0);
    Leds[4] = wb_robot_get_device("led4");
    wb_led_set(Leds[4], 0);
    Leds[5] = wb_robot_get_device("led5");
    wb_led_set(Leds[5], 0);
    Leds[6] = wb_robot_get_device("led6");
    wb_led_set(Leds[6], 0);
    Leds[7] = wb_robot_get_device("led7");
    wb_led_set(Leds[7], 0);
    Leds[8] = wb_robot_get_device("led8");
    wb_led_set(Leds[8], 0);
    Leds[9] = wb_robot_get_device("led9");
    wb_led_set(Leds[9], 0);

    SensorProx[0] = wb_robot_get_device("ps0");
    SensorProx[1] = wb_robot_get_device("ps1");
    SensorProx[2] = wb_robot_get_device("ps2");
    SensorProx[3] = wb_robot_get_device("ps3");
    SensorProx[4] = wb_robot_get_device("ps4");
    SensorProx[5] = wb_robot_get_device("ps5");
    SensorProx[6] = wb_robot_get_device("ps6");
    SensorProx[7] = wb_robot_get_device("ps7");

    wb_distance_sensor_enable(SensorProx[0], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[1], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[2], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[3], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[4], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[5], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[6], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[7], TIME_STEP);
    
  
    while (wb_robot_step(TIME_STEP) != -1)
    {
        for (i = 0; i < QtddSensoresProx; i++){
            LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]);
        }
        
        posicaoRobo = wb_supervisor_field_get_sf_vec3f(trans_field);
        //printf("Quantidade posicaoRobo[0]: %f posicaoRobo[1] %f posicaoRobo[2]: %f posicaoRobo[3] %f;\n", posicaoRobo[0], posicaoRobo[1], posicaoRobo[2], posicaoRobo[3]);
        if(posicaoRobo[0] > -0.7 && posicaoRobo[0] < -0.3 && posicaoRobo[2] > -0.7 && posicaoRobo[2] < -0.3){
            moveParaDireita = 1;
            moveParaEsquerda = 1;
            
            if(LeituraSensorProx[0] > 500 || LeituraSensorProx[1] > 500 || LeituraSensorProx[6] > 500 || LeituraSensorProx[7] > 500){                                                                 
                int x = 0;
        	     while(x < 3){
                      controlaLeds(10, 1);
                      wb_robot_step(100);
                      controlaLeds(10, 0);
                      wb_robot_step(100);
                      x++;
                  }  
                printf("Blocos móveis encontrados: %d;\n", ++qtdBlocosMoveis);}
                controlaLeds(qtdBlocosMoveis, 1);
                printf("quant blocos %d",qtdBlocosMoveis );
  
          }
          else if (LeituraSensorProx[0] > 400){
                printf("Sensor 0 ativado\n"); //primeiro sensor da direita
                moveParaDireita = 1;
                moveParaEsquerda = -0.15;
            }
            else if (LeituraSensorProx[1] > 400){
                printf("Sensor 1 ativado\n"); //segundo sensor da direita
                moveParaDireita = 1;
                moveParaEsquerda = -0.30;
            }
            else if (LeituraSensorProx[2] > 400){
                printf("Sensor 2 ativado\n"); //terceiro sensor da direita
                moveParaDireita = 1;
                moveParaEsquerda = -0.45;
            }
            else if (LeituraSensorProx[5] > 400){
                printf("Sensor 3 ativado\n"); //primeiro sensor da esquerda
                moveParaDireita = -0.45;
                moveParaEsquerda = 1;
            }
            else if (LeituraSensorProx[6] > 400){
                printf("Sensor 4 ativado\n"); //segundo sensor da esquerda
                moveParaDireita = -0.3;
                moveParaEsquerda = 1;
            }
            else if (LeituraSensorProx[7] > 400){
                printf("Sensor 5 ativado\n"); //terceiro sensor da esquerda
                moveParaDireita = -0.15;
                moveParaEsquerda = 1;
            }
            else{
            //nenhum sensor foi ativado, direcao nao é alterada
                moveParaDireita = 1;
                moveParaEsquerda = 1;
            }
        

        wb_motor_set_velocity(MotorEsquerdo, MAX_VELOC * moveParaEsquerda);
        wb_motor_set_velocity(MotorDireito, MAX_VELOC * moveParaDireita);

     };

    wb_robot_cleanup();

    return 0; 
}

