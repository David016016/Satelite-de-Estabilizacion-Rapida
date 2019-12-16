#include <Servo.h> 
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
MPU6050 sensor;
// Valores RAW (sin procesar) del acelerometro  en los ejes x,y,z
int ax, ay, az;
float Kp = 1.4;
float Kd = 0.3;
float Ki = 0.3;
float Kp1 = 0.7;
float Kd1 = 0.2;
float Ki1 = 0.15;
int Rint = -40;
int pos1;
float pos;
int posfinal1;
int posfinal;
int Rext = 40;
unsigned long time = 0; // tiempo de ejecucion del ultimo ciclo
int periodo = 50; // Periodo de muestreo en ms
int sensorPin=4; //Pin Analogico donde esta conectada la señal del Sensor de distancia
int sensorPin1=5;
int medida; // Lo que mide el sensor. Son ADCs.
int dcal [] = {-190, -160, -110, -60, 0, 40, 60, 90, 120};// Calibracion de ADC a Distancia
//int ADCcal [] = {20, 40, 60, 80, 100, 120, 140, 160, 180};
int ADCcal [] = {-80, -60, -40, -20, 0, 20, 40, 60, 80};
int lastDist; // Valor anterior de Distancia para calcular la Velocidad
int lastDist1;
int dist;
int dist1;// distancia en mm con el 0 en el centro de la barra
int nvel=5; //  numero de valores de velocidad sobre los que calculamos la media
int v[5];
int v1[5];
int vel;
int vel1;// valor medio de las nvel velocidades ultimas
float I; // Valor Integral
float I1;
Servo myservoy;
Servo myservox;// create servo object to control a servo
float reposo = 100; // valor que mantiene la barra horizontal
float reposo1 = 100;
void setup(){
  Serial.begin(57600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();
  analogReference(EXTERNAL);
  myservoy.attach(11);
  myservox.attach(10);
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
  
  myservoy.write(reposo);
  myservox.write(reposo1);
  delay(1000);
}

void loop(){ 
  if (millis()>time+periodo){ // ¿Ha transcurrido el periodo?
    time = millis();
    sensor.getAcceleration(&ax, &ay, &az);
    // Medimos DISTANCIA
    float accel_ang_x=atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
    float accel_ang_y=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
    lastDist = dist;
    lastDist1 = dist1;
    // Guardamos el valor anterior de dist para calcular la velocidad
    dist=accel_ang_y-8;//-8
    dist1=accel_ang_x+5;//+5
    if(accel_ang_y>=70){
      dist=70;
    }
    if(accel_ang_y<=-70){
      dist=-70;
    }
    if(accel_ang_x>=70){
      dist1=70;
    }
    if(accel_ang_x<=-70){
      dist1=-70;
    }
    // Calculo de la media de la VELOCIDAD
    for (int i=0; i<nvel-1; i++){ // Movemos todas hacia la izq para dejar libre la ultima.
      v[i] =v[i+1];
    }
    v[nvel-1]= (dist - lastDist); // Ponemos un dato nuevo
    vel=0;
    for (int i=0; i<nvel; i++){ // Calculamos la media
      vel = vel+ v[i];
    }
    vel = vel/nvel;
    
    for (int j=0; j<nvel-1; j++){ // Movemos todas hacia la izq para dejar libre la ultima.
      v1[j] =v1[j+1];
    }
    v1[nvel-1]= (dist1 - lastDist1); // Ponemos un dato nuevo
    vel1=0;
    for (int j=0; j<nvel; j++){ // Calculamos la media
      vel1 = vel1+ v1[j];
    }
    vel1 = vel1/nvel;
    // Integral de (-Rint,Rint)
      
    if(abs(dist)>Rint && abs(dist)<Rext){ // Solo si esta dentro de (-Rext,Rext) y fuera
      I=I+dist*Ki;
    } 
    else{
      I=0;
    }

    if(abs(dist1)>Rint && abs(dist1)<Rext){ // Solo si esta dentro de (-Rext,Rext) y fuera
      I1=I1+dist1*Ki1;
    }
    else{
      I1=0;
    }
    // Calculamos posicion del servo
   
    pos=-(Kp*dist+Kd*vel+I);
    posfinal=reposo+pos;
    pos1=-(Kp1*dist1+Kd1*vel1+I1);
    posfinal1=reposo1+pos1;
    if(posfinal>=180){
      posfinal=180;
    }
    if(posfinal<=0){
      posfinal=0;
    }
    if(posfinal1>=180){
      posfinal1=180;
    }
    if(posfinal1<=0){
      posfinal1=0;
    }
    myservox.write(posfinal1);
    myservoy.write(posfinal);
    Serial.print("Salida, Salida1, Entrada, Entrada1 ");
    Serial.print(posfinal);
    Serial.print(", ");
    Serial.print(posfinal1);
    Serial.print(", ");
    Serial.print(dist);
    Serial.print(", ");
    Serial.println(dist1);
  }
}
