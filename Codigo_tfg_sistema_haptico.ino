#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel tira = Adafruit_NeoPixel(2, 2, NEO_RGB + NEO_KHZ800);

//Debido a los problemas de comunicación entre la mano robot y el sistema háptico
//este código utiliza los pines 10 y 11 para la comunicación por Bluetooth
//mayores utiliza el puerto Serial para la comprobación de errores durante el funcionamiento.
//Afecta al esquema eléctrico, donde se configuran los pines 0 y 1 para la comunicación
//por bluetooth
SoftwareSerial mibt(10, 11);

uint8_t potenciometros = 9;
unsigned long tiempo_modonormalon;
unsigned long tiempo_modonormaloff;
uint16_t Ton = 150;
uint16_t Toff = 1500;

int8_t leds;
int8_t color1;
int8_t color2;
int8_t color3;
int8_t ccolor1;
int8_t ccolor2;
int8_t ccolor3;
int32_t tiempoluz;
int32_t duracion;


bool led = true;
bool mantener;
bool encendido;
bool modoguardar;
bool modoplay;
bool pararluz;
bool pararcalibrarluz;
bool modocalibrarluz;

volatile bool modonormal = true;
bool modonormalreal = true;

volatile bool fuera = 0;
volatile unsigned int cuenta = 0;
volatile bool boton = 0;
volatile bool guardar, guardarluz, playing, calibrar, calibrarluz, parar, play, playluz;
volatile uint8_t entradaboton = 7;
volatile bool primercalibra;
volatile bool calibrando;
volatile bool stopcalibra;
volatile bool contguardar;



uint8_t pininterrupcion = 4;
uint8_t valorRx;
uint8_t vvalor;
uint8_t dedo;

bool primero = true;
bool segundo;

unsigned long tiempo_boton2;
unsigned long tiempo_prudencia = 200;
unsigned long tiempo_prudencia2 = 2001;
unsigned long tiempo_prudencia3 = 5000;
unsigned long tiempo_guardar;
unsigned long tiempo_ejecutar;
unsigned long tiempo_calibrando;
unsigned long tiempo_mantener;



//print manda el correspondiente numero ascii de lo escrito entre parentesis : print(1) manda un 49
//write manda numero escrito literal (hasta 8 bits, de 0 a 255);

bool fase0 = 1;
bool fase2 = 1;
bool fase3 = 0;

uint16_t a, b, c, d, e, f, g, h, i, j;
uint32_t aa, bb, cc, dd, ee, ff, gg, hh, ii, jj;

unsigned long t;
uint16_t MMin[9];
uint16_t MMax[9];
int32_t mmin[9] = {15, 15, 15, 15, 15, 15, 15, 15, 60};
int32_t mmax[9] = {165, 165, 165, 165, 165, 165, 165, 165, 130};

int8_t led1 = 20;
int8_t led2 = 20;
int8_t led3 = 20;

void setup() {
  leerEEPROM();
  tira.begin();
  tira.show();
  tira.setBrightness(30);
  /*Funciona mal al configurar aquí la interrupción. En su lugar
    //se configurará en la interrupción por hardware, al pulsar el botón
    SREG = (SREG & 0b01111111); //Desabilitar interrupciones
    TIMSK4 = TIMSK4 | 0b00000000; //Deshabilita la interrupcion por
    desbordamiento
    TCCR4B |= (1 << CS40); //Configura preescala para que FT2 sea de 7812.5Hz
  */ Serial.begin(38400);
  mibt.begin(38400);
  pinMode(entradaboton, INPUT_PULLUP);
  attachInterrupt( pininterrupcion, ServicioBoton, FALLING);
  SREG = (SREG & 0b01111111) | 0b10000000; //Habilitar interrupciones
}

void ServicioBoton() {
  Serial.println(" boton"); //compruebo en el serial el angulo leido
  tiempo_boton2 = millis();
  fuera = false;
  detachInterrupt(pininterrupcion);
  TIMSK4 |= (1 << TOIE4); //Habilita la interrupcion por desbordamiento
}

ISR(TIMER4_OVF_vect) {//atención por interrupción por rebosamiento timer 4
  cuenta++;
  if (cuenta > 13) { // // comprobamos enstado del boton cada 53 milisegundos     (2 ^ 16·[(16 - 1) x10 - 6] / MHz = 0, 004 s. 0, 004x 13 veces el rebosamiento = 0, 053s
    Serial.print(millis() - tiempo_boton2); //

    cuenta = 0;
    boton = digitalRead(entradaboton);
    if (!boton) {
      if (calibrando) {
        stopcalibra = true;
        pararcalibrarluz = true;
        pararluz = true;
        modonormal = false;
      }
      else if (playing) {
        playing = false;
        parar = true;
        contguardar = false;
        modonormal = false;
      }
      else {
        guardar = true;
        modonormal = false;
      }
      if (millis() - tiempo_boton2 > tiempo_prudencia2) {
        play = true;
        modonormal = false;
        attachInterrupt( pininterrupcion, ServicioBoton, FALLING);
      }
      if (millis() - tiempo_boton2 > tiempo_prudencia3) {
        calibrar = true;
        primercalibra = true;
        modonormal = false;
        attachInterrupt( pininterrupcion, ServicioBoton, FALLING);
      }
    }
    else {
      Serial.print("fuera "); //
      TIMSK4 = (TIMSK4 & 0b00000000); //Desabilitar interrupciones
      TCCR4B |= (1 << CS40);
      primero = true;
      fuera = true;
      attachInterrupt( pininterrupcion, ServicioBoton, FALLING);
    }
  }
}

void loop() {
  if (led)
    Led();
  if (fuera) {
    if (calibrar)
      Calibrar();
    else if (play or parar)
      Ejecutar();
    else if (guardar)
      Guardar();
  }
  if (fase0)
    Fase0();
  if (fase2)
    Fase2();
  if (fase3)
    Fase3(valorRx);
}

void Calibrar() {
  play = false;
  guardar = false;
  if (stopcalibra) {
    calibrando = false;
    stopcalibra = false;
    calibrar = false;
    fase0 = true;
    //escribir en eeproom los vectores de 0 a 8 Min y de 9 a 17 Max
    uint8_t j = 0;
    for (int i = 0; i < potenciometros * 2; i = i + 2) {
      EEPROM.put(i, MMin[j]);
      EEPROM.put(i + 18, MMax[j]);
      j++;
    }
  }
  else {
    calibrando = true;
    fase0 = false;
    if (primercalibra) {
      calibrarluz = true;
      for (int i = 0; i < potenciometros; i++) {
        MMin[i] = 1023;
        MMax[i] = 0;
        primercalibra = false;
      }
    }
    else {
      primercalibra = false;
      Serial.print(" Comparando "); //
      for (int i = 0; i < potenciometros; i++) {
        uint16_t z = analogRead(i);
        MMin[i] = min(z, MMin[i]);
        MMax[i] = max(z, MMax[i]);
      }
    }
  }
}

void Ejecutar() {
  guardar = false;
  playluz = true;
  play = false;
  if (parar or not(contguardar)) {
    play = false;
    parar = false;
    pararluz = true;
    playing = false;
    mibt.write(12);
    fase0 = 1;
    Serial.print(" PARADO "); //compruebo en el serial el angulo leid
  }
  else {
    playing = true;
    playluz = true;
    mibt.write(11);
    fase0 = 0;
    Serial.print(" EJECUTANDO"); //compruebo en el serial el angulo leid
  }
}
void Guardar() {
  contguardar = true;
  guardar = false;
  play = false;
  guardarluz = true;
  mibt.write(10);
  Serial.println(" Posición guardada"); 
}

void Fase0() {
  for (int i = 0; i < potenciometros; i++) {
    a = analogRead(i); //leo entrada analogica de 0 a la cantidad de potenciometros
    aa = map(a, MMin[i], MMax[i], mmin[i], mmax[i]); //transformo a angulo
    if (a < MMin[i]) {//comparo con la tensión minima accesible para optimizar el angulo de salida
      aa = mmin[i];
    }
    else if (a > MMax[i]) {//comparo con la tensión maxima accesible para optimizar el angulo de salida
      aa = min(aa, mmax[i]);
    }
    mibt.write(aa);
  }
}

void Fase2 () {
  // Serial.println(" Fase2"); //
  if (mibt.available()) {
    valorRx = mibt.read();
    fase3 = 1;
    fase2 = 0;
  }
  else {
    valorRx = 0;
  }
}

void Fase3(int a) {
  Serial.println(" Fase3 "); //
  if (primero && (a > 0 && a <= 5)) { //número de dedo, entre 1 y 5
    primero = false;
    segundo = true;
    dedo = a;
    goto fin;
  }
  if (segundo) { //(0 a 180 grados) como máximo
    segundo = false;
    primero = true;
    vvalor = a;
    switch (dedo) {
      case 1: //1
        analogWrite(3, vvalor);
        break;
      case 2://2
        analogWrite(5, vvalor);
        break;
      case 3: //3
        analogWrite(9, vvalor);
        break;
      case 4: //4
        analogWrite(10, vvalor);
        break;
      case 5: //5
        analogWrite(11, vvalor);
        break;
    }
  }
fin:
  fase3 = 0;
  fase2 = 1;
}
void leerEEPROM() {
  uint8_t j = 0;
  for (int i = 0; i < potenciometros * 2; i = i + 2) {
    EEPROM.get(i, MMin[j]);
    EEPROM.get(i + 18, MMax[j]);
    j++;
  }
}
void Led() {
  if (guardarluz) {
    modonormalreal = false;
    if (parar or pararcalibrarluz) {
      modoplay = false;
      modocalibrarluz = false;
      parar = false;
      guardarluz = false;
      modoplay = false;
      playluz = false;
      modoguardar = false;
      modoplay = false;
      modoplay = false;
      pararcalibrarluz = false;
      led1 = 0;
      led2 = 0;
      led3 = 0;
      tira.setPixelColor(0, 0, 0, 0);
      tira.setPixelColor(1, 0, 0, 0);
      tira.show();
    }
    else {
      guardarluz = false;
      modonormal = false;
      modonormalreal = false;
      modoplay = false;
      modocalibrarluz = false;
      modoguardar = true;
      led1 = 120;
      led2 = 255;
      led3 = 0;
      duracion = 600;
      tiempoluz = 75;
      tiempo_mantener = millis();
      tiempo_guardar = millis();
      tira.setPixelColor(0, led1, led2, led3);
      tira.setPixelColor(1, led1, led2, led3);
      tira.show();
      encendido = true;
    }
  }
  if (playluz) {
    playluz = false;
    modoplay = true;
    modonormal = false;
    modonormalreal = false;
    led1 = 0;
    led2 = 255;
    led3 = 255;
    tiempo_ejecutar = millis();
    tiempo_ejecutar = millis();
    tira.setPixelColor(0, led1, led2, led3);
    tira.setPixelColor(1, led1, led2, led3);
    tira.show();
    encendido = true;
  }
  if (pararluz) {
    modoplay = false;
    modonormal = false;
    modonormalreal = false;
    pararluz = false;
    modoguardar = true;
    led1 = 0;
    led2 = 255;
    led3 = 0;
    duracion = 400;
    tiempoluz = 50;
    tiempo_mantener = millis();
    tiempo_guardar = millis();
    tira.setPixelColor(0, led1, led2, led3);
    tira.setPixelColor(1, led1, led2, led3);
    tira.show();
  }
  if (pararcalibrarluz) {
    modoplay = false;
    modonormalreal = false;
    modocalibrarluz = false;
    parar = false;
    guardarluz = false;
    modoplay = false;
    playluz = false;
    modoguardar = true;
    modoplay = false;
    modoplay = false;
    pararcalibrarluz = false;
    led1 = 0;
    led2 = 255;
    led3 = 0;
    duracion = 400;
    tiempoluz = 50;
    tiempo_mantener = millis();
    tiempo_guardar = millis();
    tira.setPixelColor(0, led1, led2, led3);
    tira.setPixelColor(1, led1, led2, led3);
    tira.show();
  }
  if (calibrarluz) {
    calibrarluz = false;
    guardarluz = false;
    modonormal = false;
    modonormalreal = false;
    modoplay = false;
    playluz = false;
    modoguardar = false;
    modocalibrarluz = true;
    led1 = 255;
    led2 = 255;
    led3 = 255;
    tiempo_calibrando = millis();
    tira.setPixelColor(0, led1, led2, led3);
    tira.setPixelColor(1, led1, led2, led3);
    tira.show();
    encendido = true;
  }
  if (modocalibrarluz) {
    calibrarluz = false;
    guardarluz = false;
    modoplay = false;
    playluz = false;
    modoguardar = false;
    if (encendido == true) {
      tira.setPixelColor(0, led1, led2, led3);
      tira.setPixelColor(1, led1, led2, led3);
      tira.show();
    }
    else
    {
      tira.setPixelColor(0, 0, 0, 0);
      tira.setPixelColor(1, 0, 0, 0);
      tira.show();
    }
    Serial.print(" t guardar ");
    Serial.print(millis() - tiempo_calibrando);
    Serial.println(" encendido= ");
    if ( millis() - tiempo_calibrando > 50) {
      tiempo_calibrando = millis();
      encendido = !encendido;
    }
  }
  if (modoplay) {
    if (encendido == true) {
      tira.setPixelColor(0, led1, led2, led3);
      tira.setPixelColor(1, led1, led2, led3);
      tira.show();
    }
    else
    {
      tira.setPixelColor(0, 0, 0, 0);
      tira.setPixelColor(1, 0, 0, 0);
      tira.show();
    }
    if ( millis() - tiempo_ejecutar > 200) {
      tiempo_ejecutar = millis();
      encendido = !encendido;
    }
  }
  if (modoguardar) {
    if (encendido == true) {
      tira.setPixelColor(0, led1, led2, led3);
      tira.setPixelColor(1, led1, led2, led3);
      tira.show();
    }
    else
    {
      tira.setPixelColor(0, 0, 0, 0);
      tira.setPixelColor(1, 0, 0, 0);
      tira.show();
    }
    if ( millis() - tiempo_guardar > tiempoluz) {
      tiempo_guardar = millis();
      encendido = !encendido;
    }
    if ( millis() - tiempo_mantener > duracion) {
      modoguardar = false;
      modonormal = true;
    }
  }
  if (modonormal) {
    modonormal = false;
    modonormalreal = true;
    led1 = 255;
    led2 = 0;
    led3 = 255;
    tiempo_modonormalon = millis();
    tiempo_modonormaloff = millis();
    tira.setPixelColor(0, led1, led2, led3);
    tira.setPixelColor(1, led1, led2, led3);
    tira.show();
    encendido = true;
  }
  if (modonormalreal == true) {
    if (encendido == true) {
      tira.setPixelColor(0, led1, led2, led3);
      tira.setPixelColor(1, led1, led2, led3);
      tira.show();
    }
    else
    {
      tira.setPixelColor(0, 0, 0, 0);
      tira.setPixelColor(1, 0, 0, 0);
      tira.show();
    }
    if (encendido) {
      if ( millis() - tiempo_modonormalon > Ton) {
        tiempo_modonormalon = millis();
        encendido = !encendido;
      }
    }
    else {
      if ( millis() - tiempo_modonormalon > Toff) {
        tiempo_modonormalon = millis();
        encendido = !encendido;
      }
    }
  }
}
