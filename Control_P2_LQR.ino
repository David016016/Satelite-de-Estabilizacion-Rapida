#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
MPU6050 sensor;
// Valores RAW (sin procesar) del acelerometro  en los ejes x,y,z
int ax, ay, az;
double a;
double k[] = {1.1, 0.3334};
double k1[] = {0.7, 0.2164};
int Rint = -40;
float pos;
float pos1;
int posfinal;
int posfinal1;
int Rext = 40;
unsigned long time = 0; // tiempo de ejecucion del ultimo ciclo
int periodo = 50; // Periodo de muestreo en ms
int sensorPin = 4; //Pin Analogico donde esta conectada la señal del Sensor de distancia
int sensorPin1 = 5;
int medida; // Lo que mide el sensor. Son ADCs.
int lastDist; // Valor anterior de Distancia para calcular la Velocidad
int lastDist1;
int dist;
int dist1;// distancia en mm con el 0 en el centro de la barra
int nvel = 5; //  numero de valores de velocidad sobre los que calculamos la media
int nvel1 = 5;
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
void setup() {
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

void loop() {
  if (millis() > time + periodo) { // ¿Ha transcurrido el periodo?
    time = millis();
    sensor.getAcceleration(&ax, &ay, &az);
    // Medimos DISTANCIA
    float accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
    float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
    lastDist = dist;
    lastDist1 = dist1;
    // Guardamos el valor anterior de dist para calcular la velocidad
    dist = accel_ang_y - 8;
    dist1 = accel_ang_x + 8;
    if (accel_ang_y >= 90) {
      dist = 90;
    }
    if (accel_ang_y <= -90) {
      dist = -90;
    }
     if (accel_ang_x >= 90) {
      dist1 = 90;
    }
    if (accel_ang_x <= -90) {
      dist1 = -90;
    }
    // Calculo de la media de la VELOCIDAD
    for (int i = 0; i < nvel - 1; i++) { // Movemos todas hacia la izq para dejar libre la ultima.
      v[i] = v[i + 1];
    }
    v[nvel - 1] = (dist - lastDist); // Ponemos un dato nuevo
    vel = 0;
    for (int i = 0; i < nvel; i++) { // Calculamos la media
      vel = vel + v[i];
    }
    vel = vel / nvel;

    for (int j = 0; j < nvel1 - 1; j++) { // Movemos todas hacia la izq para dejar libre la ultima.
      v1[j] = v1[j + 1];
    }
    v1[nvel1 - 1] = (dist1 - lastDist1); // Ponemos un dato nuevo
    vel1 = 0;
    for (int j = 0; j < nvel1; j++) { // Calculamos la media
      vel1 = vel1 + v1[j];
    }
    vel1 = vel1 / nvel1;

    double pos = -k[0] * dist - k[1] * vel - dist*1.2;
    posfinal = reposo + pos;
    /*if(abs(dist1)>Rint && abs(dist1)<Rext){ // Solo si esta dentro de (-Rext,Rext) y fuera
      a=a+dist1*k1[1];
    }else{
      a=0;
    }*/
    double pos1 = -k1[0] * dist1 - k1[1] * vel1 - dist1 ;
    posfinal1 = reposo1 + pos1;
    
    /*if(posfinal<70){
      posfinal=posfinal-20;
    }else{ 
      if(posfinal>130){
        posfinal=posfinal+20;
      }else{
        posfinal=posfinal;
      }
    }*/
    if (posfinal >= 180) {
      posfinal = 180;
    }
    if (posfinal <= 0) {
      posfinal = 0;
    }
    if (posfinal1 >= 180) {
      posfinal1 = 180;
    }
    if (posfinal1 <= 0) {
      posfinal1 = 0;
    }
    myservoy.write(posfinal);
    myservox.write(posfinal1);
    Serial.print("  dist1  ");
    Serial.print(dist1);
    Serial.print("  pos1  ");
    Serial.print(pos1);
    Serial.print("   posfinal1   ");
    Serial.println(posfinal1);
  }
}
