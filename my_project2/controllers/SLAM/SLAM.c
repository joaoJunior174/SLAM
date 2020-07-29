#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>
#include <webots/gps.h>
#include <math.h>
#include <webots/supervisor.h>
#include <webots/inertial_unit.h>

#define TIME_STEP 64
//defini aqui uma função para pegar o max entre dois números
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define M_PI 3.14159265358979323846

void delay(int time_milisec)
{
double currentTime, initTime, Timeleft;
double timeValue = (double)time_milisec/1000;
initTime = wb_robot_get_time();
Timeleft =0.00;
while (Timeleft < timeValue)
 {
  currentTime = wb_robot_get_time();
  Timeleft=currentTime-initTime;
  wb_robot_step(TIME_STEP);
  }
}


void converterCoordenadasGrid(double i,double j,double x,double y,double gridMap[16][16]){

     int indicei = round((8+i)+x);
     int indicej = round((8-j)+y);

     printf("i: %i j: %i\n",indicei,indicej);

}

double calculaAngulo(double ang){

   return (M_PI*ang)/180;

}

void mostraAngulo(double rad){

   printf("Angulo: %f %f\n",(180*rad)/M_PI,rad);

}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  //instanciando motores
 // WbDeviceTag left_motor_1 = wb_robot_get_device("front left wheel");
 // WbDeviceTag right_motor_1 = wb_robot_get_device("front right wheel");
 // WbDeviceTag left_motor_2 = wb_robot_get_device("back left wheel");
  //WbDeviceTag right_motor_2 = wb_robot_get_device("back right wheel");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag yaw = wb_robot_get_device("inertial unit");
  
  //instanciando todos  sensores da frente
  WbDeviceTag sensf4 = wb_robot_get_device("so4");
  WbDeviceTag sensf6 = wb_robot_get_device("so6");
  WbDeviceTag sensf0 = wb_robot_get_device("so0");
  WbDeviceTag sensf7 = wb_robot_get_device("so7");
  WbDeviceTag sensf5 = wb_robot_get_device("so5");
  
  //aplicando o tempo em passos que os sensores vão caputar sinais
  wb_distance_sensor_enable(sensf4,TIME_STEP);
  wb_distance_sensor_enable(sensf6,TIME_STEP);
  wb_distance_sensor_enable(sensf0,TIME_STEP);
  wb_distance_sensor_enable(sensf7,TIME_STEP);
  wb_distance_sensor_enable(sensf5,TIME_STEP);
  wb_gps_enable(gps, TIME_STEP);
  wb_inertial_unit_enable(yaw, TIME_STEP);
  //contador para a janela da integral
  int cont=0;
  //setando posição do motor
   //wb_motor_set_position(left_motor_1,INFINITY);
   //wb_motor_set_position(right_motor_1,INFINITY);
   //wb_motor_set_position(left_motor_2,INFINITY);
   //wb_motor_set_position(right_motor_2,INFINITY);
  
  //velocidade dos motores nas 4 rodas
  double left_speed_1 = 0;
  double left_speed_2 = 0;
  double right_speed_1=0;
  double right_speed_2=0;
  
  //variaveis que vão pegar os valores dos sensores
  //double sensf2_value;
  double sensf4_value;
  double sensf6_value;
  double sensf0_value;
  double sensf7_value;
  double sensf5_value;
  //variáveis de controle 
  double position=190;
  double old_error=0;
  double integral=0;
  //constantes
  double p_gain = 0.6; 
  double i_gain = 0.00; 
  double d_gain = 0.8;

  double gridMap[16][16];
  
  

  while (wb_robot_step(TIME_STEP) != -1) {
  
     
     double *valores = wb_inertial_unit_get_roll_pitch_yaw(yaw);
     //printf("valores: %f %f %f\n",valores[0],valores[1],valores[2]);
     
     double anguloDiff = -valores[2]-calculaAngulo(10);
     //printf("Angulo em graus: %f\n",anguloDiff);
     mostraAngulo(-valores[2]);
     
     
     double *coord=wb_gps_get_values(gps);
     printf("Coordenadas gps: %f %f %f\n",coord[0],coord[1],coord[2]);
     //converterCoordenadasGrid(coord[0],coord[2],gridMap);
     //pegando os valores dos sensores
     sensf4_value = wb_distance_sensor_get_value(sensf4);
     sensf6_value = wb_distance_sensor_get_value(sensf6);
     sensf0_value = wb_distance_sensor_get_value(sensf0);
     sensf7_value = wb_distance_sensor_get_value(sensf7);
     sensf5_value = wb_distance_sensor_get_value(sensf5);
     
     double distanciaMetros=5-(5*sensf4_value)/1024;
    
     printf("Valor do sensor %f\n",distanciaMetros);
    
     double andarY=cos(anguloDiff)*distanciaMetros;
     double andarX=sin(anguloDiff)*distanciaMetros;
     
     printf("Valor de andar %f %f\n",andarY,andarX);
     
     converterCoordenadasGrid(coord[0],coord[2],andarX,andarY,gridMap);
    
     //verifica se há algum objeto na frente de acordo com o valor retornado pelo sensor
     if(sensf4_value > 850){
     
         //seta velocidades contrárias para rotacionar o robô
         left_speed_1 = -3.0;
         left_speed_2 = -3.0;
         right_speed_1= 3.0;
         right_speed_2= 3.0;
     
         //seta as velocidades
        // wb_motor_set_velocity(left_motor_1,left_speed_1);
        // wb_motor_set_velocity(left_motor_2,left_speed_2);
        // wb_motor_set_velocity(right_motor_1,right_speed_1);
        // wb_motor_set_velocity(right_motor_2,right_speed_2);
     
         //aplica um delay de 0,2s
         delay(200);
         //volta a velocidade ao normal
         left_speed_1 = 3.0;
         left_speed_2 = 3.0;
         right_speed_1= 3.0;
         right_speed_2= 3.0;
     
     }
     //verifica se tem uma parede à esquerda, se houver, ele diminui a distância mínima entre o robô e a parede à direita, assim o robô não raspa em nada 
     if(sensf0_value > 850)
         position = 100;
     else
         position = 170;
          
     //aqui é feita a verificação entre os sensores da frente e o do lado direito, para então ser pego o maior valor. O maior valor entre eles é o que importa, pois isso significa que tem alguma 
     //chance de colisão     
     double dist = (1024.0-MAX(sensf5_value,MAX(sensf7_value,sensf6_value)));
     double error = position - dist;
     //aqui é feito uma janela para o cálculo da integral, sendo essa janela de 50 iterações, ou seja, uma iteração de 50 time step faz o calculo com a integral, depois outros 50 não considera
     //isso tira os altos erros acumulados   
     cont=cont+1;
     if(cont < 50){
        integral=0;
     }
     else{  
        integral = integral + error;
        if(cont>100)
           cont=0;
          
     }    
     //aqui é o código do slide, não mudei nada
     double dif_erro= error - old_error; 
     old_error = error;
     double power = p_gain*error + i_gain*integral + d_gain*dif_erro;
          
     right_speed_1 = 3.0+power;   
     right_speed_2 = 3.0+power;   
         
     if(right_speed_1 > 5.0){
         
        right_speed_1 = 5.0;
        right_speed_2 = 5.0;
         
      }
      if(right_speed_1 < 1.0){
         
        right_speed_1 = 1.0;
        right_speed_2 = 1.0;
         
       }
     
       //aqui seta a velocidade dos motores
       //wb_motor_set_velocity(left_motor_1,left_speed_1);
       //wb_motor_set_velocity(left_motor_2,left_speed_2);
       //wb_motor_set_velocity(right_motor_1,right_speed_1);
       //wb_motor_set_velocity(right_motor_2,right_speed_2);     
     

  };

  wb_robot_cleanup();

  return 0;
}