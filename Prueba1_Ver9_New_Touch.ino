#include <PinChangeInterruptBoards.h>
#include <YetAnotherPcInt.h>
#include <TinyGsmClientSIM800.h>
#include <TinyGsmCommon.h>
#include <TinyGsmFifo.h>
#include <CRC32.h>

#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <sdios.h>
#include <SysCall.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

/*
 * 18/04/19 Version totalmente operativa de Dispensador Touch V2.0 sin programacion por SW mi por GSM
 * 23/05/19 Se cambian las entradas de pulsos para generar la señal del flujo, se usan PCINT 53 y 52 con adicion de libreria. SE cambia a A14 y A15 para dejar libre pines de HW SPI para memoria SD 16/02/2020
 * 27/07/2019 Se agrega visualizacion de archivo *.bmp guardado en memoria SD como pieza publicitaria. Se arregla el problema de los puntos para los botones y se deja casi todo su sector para habilitar 
 * Queda habilitada la visualizacion de varios archivos en la SD. Ademas de la buena separacion entre la superficie de toque en los dos botones.
 * 31/07/2019 Se empieza a ensamblar la funcionalidad GPRS, se adiciona la posibilidad de elegir los archivos del 0 al 9 para tener ese numero para mostrar.
 * 24/08/2019 Se programa para que las constantes de conexion por GPRS sean enviadas en la orden para asi ser adaptables al contexto que se use, se genera el codigo para que la evolucion se muestre por LCD y se envie SMS de estado.
 * 25/08/2019 Se codifican las pantallas de debug en el TFT, se cambia a SPI HW para la SD y se eliminan problemas de carga de archivos de la RED
 * 26/06/2019 Se implementa el borrado selectivo y total (WIPE) de archivos de la SD. 
 * 27/08/2019 Se corrige e implementa un nuevo codigo de liquidacion de procesos.
 * 30/08/2019 Se codifica la seccion de configuracion de pulsos por cantidad de producto.
 * 16/02/2020 Se retoma para hacer instalacion y completar el segundo dispensador.
 * 17/02/2020 Se implementa salida por tiempo si no hay conteo de pulsos por 30 segundos
 * 16/03/2020 Se corrigen las definiciones de los puertos analogos para el Touch. Se redefinen los vectores touch de los botones
 * 18/03/2020 Se cambia la entrada de pulsos por efectos de la implementacion del conector general
 * 19/06/2020 Se ha estado trabajando en el problema de la impresora 2 "0" en vez de espacio.
 * se trabaja en la conexion entre controladores por usart3. Se implementa el tiempo de vacio. Se habia implementado el registro de orientacion de la LCD.catch_Tiempo
 * 
 */
#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 256

#include <SPI.h>             //incluida si SPI HW
#include <SdFat.h>
#include <sdios.h>
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen_kbv.h> 
#include <HCSR04.h>
#include <Adafruit_Thermal.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <YetAnotherPcInt.h>
#include <TinyGsmClient.h>

#define SerialMon Serial
#define SerialAT  Serial1
#define USE_SDFAT
#define MINPRESSURE 2000
#define MAXPRESSURE 1200
#define buzz    42
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define XP      8
#define XM      A2
#define YP      A3
#define YM      9
#define bomba1  46
#define bomba2  44
#define pulsos_m1 A8          //A8 Pul1 Amarillo conector JST
#define pulsos_m2 A9          //A9 Pul1 Amarillo conector JST
#define rst_m1 43             //23 RST M1 Verde connector JST macho
#define rst_m2 26             //28 RST M2 Verde connector JST macho
#define SD_CS  53
#define NAMEMATCH ""           // "" matches any name
#define PALETTEDEPTH   8       // support 256-colour Palette
#define USE_SDIO 1
#define print_cst A13           //21

const int TS_LEFT = 907, TS_RT = 136, TS_TOP = 942, TS_BOT = 139;

uint32_t knownCRC32    = 0xD135996B;
uint32_t knownFileSize = 1024;   // In case server does not send it
uint32_t tamano;

int pixel_x, pixel_y;     //Touch_getXY() updates global vars
volatile int cont_pul1=0;
volatile int cont_pul1_old=0;
volatile int cont_pul2=0;
volatile int cont_pul2_old=0;
long timeCounter=0;
int color[]={RED,GREEN,YELLOW,BLACK,BLUE,CYAN,MAGENTA};
int cont_cabezote_m1=0;
int cont_cabezote_m2=0;
int cont1_cabezote_m1=0;
int cont1_cabezote_m2=0;
long pulsos;

int cont_proceso_vacio_1=0;
int cont_proceso_vacio_2=0;

String labels[]={"Borrando..."," "," ","Acerque envase","Llenando...","Pagar:","Imprimiendo...","Parametrizando..."};
String code="Avtec1832";
String lbl_m1="";
String lbl_m2="";
String msg="";
String mensaje="";
String mensaje1="";
String mensaje2="";
String D="";
String T="";
String resumen="";
unsigned long catch_Tiempo0;       //Timer del tiempo maximo de bomba 1
unsigned long catch_Tiempo1;       //Timer del tiempo maximo de bomba 2
unsigned long catch_Tiempo2;
unsigned long catch_Tiempo3;
unsigned long catch_Tiempo4;
unsigned long catch_Tiempo5;
unsigned long catch_Tiempo6;
unsigned long catch_Tiempo7;
unsigned long catch_Tiempo8;       //Timer del protector de pantalla o Logo
unsigned long catch_Tiempo9=0;     
unsigned long catch_Tiempo10=0;    
unsigned long catch_Tiempo11=0;    //Timer de proceso vacio1
unsigned long catch_Tiempo12=0;    //Timer de proceso vacio2
unsigned long catch_Tiempo13=0;    //Timer de proceso vacio1
unsigned long catch_Tiempo14=0;    //Timer de proceso vacio2

unsigned long tdata;
unsigned long Tel=EEPROM.get(600,Tel);          //Carga el numero administrador
unsigned int t_max_m1=EEPROM.get(500,t_max_m1); //EEPROM.write(500)
unsigned int t_max_m2=EEPROM.get(504,t_max_m2); //EEPROM.write(504)
unsigned int t_modo_dist_m1=100;                  //EEPROM.write(508)
unsigned int t_modo_dist_m2=100;                  //EEPROM.write(512)
unsigned int t_logo=EEPROM.get(516,t_logo);       //EEPROM.write(516)
unsigned int h_touch=EEPROM.read(520);          //EEPROM.write(520)
unsigned int rt_dsp=EEPROM.get(524,rt_dsp);     //EEPROM.put(524)

boolean acc_bomba1=false;
boolean acc_bomba2=false;
boolean b_distancia_m1=false;
boolean b_distancia_m2=false;
boolean b_completo_m1=false;
boolean b_completo_m2=false;
boolean b_impresion_m1=false;
boolean b_impresion_m2=false;
boolean b_liquida_m1=false;
boolean b_liquida_m2=false;
boolean h_dist_m1=false;
boolean h_dist_m2=false;
boolean b_dist_m1=false;
boolean b_dist_m2=false;
boolean h_cortina=false;
boolean h_cortina_t=false;
boolean e_cortina=true;
boolean b_todo=false;
boolean b_proceso1=false;
boolean b_proceso2=false;
boolean h_touch_1=true;
boolean h_touch_2=true;
boolean f_print=false;
boolean h_archivos=false;
boolean h_sd=false;
boolean b_proceso_vacio_1=false;
boolean b_proceso_vacio_2=false;
boolean b1_e=false;
boolean b2_e=false;
boolean e_rcv=false;
boolean fwd=false;

const char separator=',';
const int dataLength=12;
String data[dataLength];

unsigned long  cant_total_ml;
unsigned long  dinero_operacion_m1;
unsigned long  pulsos_operacion_m1;
unsigned long  acum_dinero_m1;
unsigned long  acum_vertido_m1;
unsigned long  acum_restante_m1;
unsigned long  acum_operaciones_m1_carga;
unsigned long  acum_operaciones_m1_total;
unsigned long  cant_porcion_m1;
int mind1_s1;                                   //EEPROM(300)
int maxd1_s1;                                   //EEPROM(302)
int mind1_s2;                                   //EEPROM(304)
int maxd1_s2;                                   //EEPROM(306)
int Sensd11;
int Sensd12;
int vel_bomba1;                                 //EEPROM(308)
int ram_bomba1;                                 //EEPROM(310)


unsigned long  cant_total_m2;
unsigned long  dinero_operacion_m2;
unsigned long  pulsos_operacion_m2;
unsigned long  acum_dinero_m2;
unsigned long  acum_vertido_m2;
unsigned long  acum_restante_m2;
unsigned long  acum_operaciones_m2_carga;
unsigned long  acum_operaciones_m2_total;
unsigned long  cant_porcion_m2;
int mind2_s1;                                   //EEPROM.write(400)
int maxd2_s1;                                   //EEPROM.write(402)
int mind2_s2;                                   //EEPROM.write(404)
int maxd2_s2;                                   //EEPROM.write(406)
int Sensd21;
int Sensd22;
int vel_bomba2;                                 //EEPROM(408)
int ram_bomba2;                                 //EEPROM(410)

int tempPul1=0;
int tempPul2=0; 
int flujo1=0;
int flujo2=0;

char namebuf[32] = "/";                        //BMP files in root directory
File root;
int pathlen;
int cont_arch=0;
int cont_sd=0;
int intentos_dw=0;
const uint8_t SD_CHIP_SELECT=53;              //53 SPI HW, 10 SPI SW
char lbl_b1[5]="1234mL";
char lbl_b2[5]="9876mL";
int NumAdmon=EEPROM.read(299); 

Adafruit_Thermal printer(&Serial2);                //Este es el USART2 para la impresora
TouchScreen_kbv ts(XP, YP, XM, YM, 300);           
TSPoint_kbv tp;                                    //global point
HCSR04 D_Sensor_m1_1(49,48);     //Trigger,ECHO
HCSR04 D_Sensor_m1_2(47,45);
HCSR04 D_Sensor_m2_1(23,22);
HCSR04 D_Sensor_m2_2(25,24);
SdFatEX SD;                     // USE_SDIO
Adafruit_GFX_Button M1_btn, M2_btn;
MCUFRIEND_kbv tft;
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

void setup(void){
  pinMode(bomba1,OUTPUT);
  pinMode(bomba2,OUTPUT);
  pinMode(print_cst,OUTPUT);
  pinMode(buzz,OUTPUT);
  pinMode(pulsos_m1,INPUT_PULLUP);
  pinMode(pulsos_m2,INPUT_PULLUP);
  pinMode(rst_m1,INPUT_PULLUP);
  pinMode(rst_m2,INPUT_PULLUP);
  digitalWrite(pulsos_m1,HIGH);
  digitalWrite(pulsos_m2,HIGH);
  digitalWrite(buzz,LOW);
  digitalWrite(bomba1,LOW);
  digitalWrite(bomba2,LOW);
  Serial.begin(115200);         //Local puerto
  Serial1.begin(115200);        //GSM puerto
  Serial2.begin(9600);          //Impresora puerto
  Serial3.begin(115200);        //Intercomunicacion
  Beep();
  uint16_t ID = tft.readID();
  if (ID==0xD3D3)ID=0x9481; // write-only shield
  tft.begin(ID);
  if(rt_dsp>4){
    rt_dsp=1;
    EEPROM.put(524,rt_dsp);
  }
  tft.setRotation(rt_dsp); 
  tft.fillScreen(WHITE);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  msg="Dispensador Touch V1.0";
  Presente_datos(true);
  printer.begin(); 
//  printer.setDefault();
  Carga_data_m1();
  Carga_data_m2();
  if(EEPROM.read(8)>250){
    EEPROM.write(8,0);
  }
  NumAdmon=EEPROM.read(299);
  if(NumAdmon>250){
    EEPROM.write(599,1);
    long num=3185160372;
    EEPROM.put(600,num);
  }
  if(Configuracion()){
    String modemInfo = modem.getModemInfo();
    msg=(F("Modem: "));
    Presente_datos(false);
    msg=(modemInfo);
    Presente_datos(true);
    client.stop();
    msg=(F("Servidor desconectado"));
    Presente_datos(true);
    modem.gprsDisconnect();
    msg=(F("GPRS desconectado"));
    Presente_datos(true);
  }
  bool good = SD.begin(SD_CHIP_SELECT);
  if(!good){
    msg=(F("no SD.."));
    Presente_datos(true);
    while(!SD.begin(SD_CHIP_SELECT)){
      cont_sd++;
      msg="Intento: "+String(cont_sd);
      Presente_datos(true);
      SD.begin(SD_CHIP_SELECT);
      if(cont_sd>2){
        msg="Problema con SD, continuando...";
        Presente_datos(true);
        h_sd=false;
        cont_sd=0;
        break;
      }
    }                       //Habilita necesidad de SD
  }else{
    msg=(F("SD encontrada.."));
    Presente_datos(true);
    h_sd=true;
  }
  tft.fillScreen(WHITE);
  tft.setCursor(0,0);
  root=SD.open("/");
  printDirectory(root, 0);
  msg=(F("Dispensador listo...OK"));
  Presente_datos(true); 
  tft.fillScreen(WHITE);
  Show_lbl1_m1();
  Show_lbl1_m2();
  catch_Tiempo2=millis();
  catch_Tiempo8=millis(); 
  catch_Tiempo13=millis();
  catch_Tiempo14=millis(); 
  h_cortina_t=true;
  PcInt::attachInterrupt(pulsos_m1,ConteoP1,CHANGE);   //Entrada de pulsos bomba1
  PcInt::attachInterrupt(pulsos_m2,ConteoP2,CHANGE);   //Entrada de pulsos bomba2
  Beep();
  e_rcv=true;
  interrupts();
}

void loop(void){
  Cortina();
  if(Touch_getXY()){Init_Proceso();}
  if(acc_bomba1){Habilita_M1();}
  if(acc_bomba2){Habilita_M2();}

  if(acc_bomba1&&cont_pul1_old!=cont_pul1){
    Show_data1(cont_pul1,0);
    cont_pul1_old=cont_pul1;
    tdata=EEPROM.get(108,tdata);
    b_proceso1=true;
    if(cont_pul1>=tdata){b_proceso1=false;b_completo_m1=true;Stop_proceso(1);} //Pulsos_operacion_m1
  }

  if(acc_bomba2&&cont_pul2_old!=cont_pul2){
    Show_data2(cont_pul2,0);
    cont_pul2_old=cont_pul2;
    tdata=EEPROM.get(208,tdata);
    b_proceso2=true;
    if(cont_pul2>=tdata){b_proceso2=false;b_completo_m2=true;Stop_proceso(2);}    //Pulsos_operacion_m2
  }
 
  if(acc_bomba1&&b_distancia_m1){//Esto habilita la bomba si se cumple distancia y boton 1
    On_bomba(1);
    delay(2);
    cont1_cabezote_m1++;
    if(cont1_cabezote_m1==1){Cabezote_m1(4);}
  } 
  
  if(b_completo_m1){
    Beep();
    b_completo_m1=false;
    b_impresion_m1=true;
    Barra_estado1(4);
    Cabezote_m1(5);
    tft.print("$");tft.print(dinero_operacion_m1);
    catch_Tiempo4=millis();
    M1_btn.drawButton(false);
  }
 
  if(acc_bomba2&&b_distancia_m2){//Esto habilita la bomba si se cumple distancia y boton 2
    On_bomba(2);
    delay(2);
    cont1_cabezote_m2++;
    if(cont1_cabezote_m2==1){Cabezote_m2(4);}
  }
  
  if(b_completo_m2){
    Beep();
    b_completo_m2=false;
    b_impresion_m2=true;
    Barra_estado2(4);
    Cabezote_m2(5);
    tft.print("$");tft.print(dinero_operacion_m2);
    catch_Tiempo5=millis();
    M2_btn.drawButton(false);
  }
  
  Valide_reset();

  if(millis()-catch_Tiempo4>5000 && b_impresion_m1){
    b_impresion_m1=false;
    b_liquida_m1=true;
    Cabezote_m1(6);                       //Imprimiendo Bomba 1
    Barra_estado1(5);
    Impresion_m1();
    catch_Tiempo6=millis();
    h_cortina_t=false;
    catch_Tiempo8=millis();
  }
  if(millis()-catch_Tiempo5>5000 && b_impresion_m2){
    b_impresion_m2=false;
    b_liquida_m2=true;
    Cabezote_m2(6);                       //Imprimiendo Bomba 2
    Barra_estado2(5);
    Impresion_m2(); 
    catch_Tiempo7=millis();
    h_cortina_t=false;
    catch_Tiempo8=millis();
  } 

  if(millis()-catch_Tiempo6>2000 && b_liquida_m1){
    b_liquida_m1=false;
    Liquida_m1();
    if(EEPROM.read(8)==1){mensaje1=mensaje2;Mensajea();}else{msg="No habilitado\n";Presente_datos(true);}
    Pre_lcd();
    Show_lbl1_m1();
    if(!acc_bomba2){Show_lbl1_m2();}
    h_cortina_t=true;
    catch_Tiempo8=millis();
  }
  if(millis()-catch_Tiempo7>2000 && b_liquida_m2){
    b_liquida_m2=false;
    Liquida_m2();
    if(EEPROM.read(8)==1){mensaje1=mensaje2;Mensajea();}else{msg="No habilitado\n";Presente_datos(true);}
    Pre_lcd();
    Show_lbl1_m2();
    if(!acc_bomba1){Show_lbl1_m1();}
    h_cortina_t=true;
    catch_Tiempo8=millis();
  } 

  if((millis()-catch_Tiempo8>1000*t_logo) && (h_cortina_t==true) && (acc_bomba1==false) && (acc_bomba2==false)){
    h_cortina=true;
    Serial.println("habilitando cortina.."+String(millis()));
    f_print=true;
    h_cortina_t=false;
    catch_Tiempo8=millis();
  } 

  if(millis()-catch_Tiempo11>500 and acc_bomba1){ 
    tempPul1++;
    flujo1=cont_pul1/tempPul1;
    if(tempPul1>t_max_m1 and flujo1<1){
      Stop_proceso(1);
    }
    catch_Tiempo11=millis();
  }
  if(millis()-catch_Tiempo12>500 and acc_bomba2){ 
    tempPul2++;
    flujo2=cont_pul2/tempPul2;
    if(tempPul2>t_max_m2 and flujo2<1){
      Stop_proceso(2);
    }
    catch_Tiempo12=millis();
  }

}

void Habilita_M1(){
  if(h_dist_m1&&b_dist_m1){
    Sensd11=D_Sensor_m1_1.dist();//measureDistanceCm();
    Sensd12=D_Sensor_m1_2.dist();//measureDistanceCm();
    if(Sensd11>mind1_s1&&Sensd11<maxd1_s1&&Sensd12>mind1_s2&&Sensd12<maxd1_s2){ //
      b_distancia_m1=true;
      if(t_modo_dist_m1==100&&b_distancia_m1==1){b_dist_m1=false;}
        cont_cabezote_m1=0;
        catch_Tiempo0=millis();
        catch_Tiempo11=millis();
        tempPul1=0;
    }else{
        b_distancia_m1=false;
        cont_cabezote_m1++;
        if(cont_cabezote_m1==1){Cabezote_m1(3);}
        cont1_cabezote_m1=0;
    }
  }
}

void Habilita_M2(){
  if(h_dist_m2&&b_dist_m2){
    Sensd21=D_Sensor_m2_1.dist();//measureDistanceCm();
    Sensd22=D_Sensor_m2_2.dist();//measureDistanceCm();
    if(Sensd21>mind2_s1&&Sensd21<maxd2_s1&&Sensd22>mind2_s2&&Sensd22<maxd2_s2){  
      b_distancia_m2=true;
      if(t_modo_dist_m2==100&&b_distancia_m2==1){b_dist_m2=false;} 
        cont_cabezote_m2=0;
        catch_Tiempo1=millis();
        catch_Tiempo12=millis();
        tempPul2=0;
    }else{
      b_distancia_m2=false;
      cont_cabezote_m2++;
      if(cont_cabezote_m2==1){Cabezote_m2(3);}
      cont1_cabezote_m2=0;
    }
  }
}

void Cortina(){
  if(h_cortina==true&&h_sd==true){
    while (!f_print){Muestre_logo();}         //Nunca es falsa asi que no se hace
    e_cortina=true;
    f_print=false;
    if(acc_bomba1==false&&acc_bomba2==false){
      h_cortina_t=true;
      catch_Tiempo8=millis();
    }
  }
}

void serialEvent(){
  if(!e_rcv){return;}
  D=Serial.readString();
  Serial1.print(D);
  Asigne_valores(D);
  Discrimine();
  h_cortina=false;
  f_print=false;
  Serial.flush();  
}

void serialEvent1(){
  if(!e_rcv){return;}
  T=Serial1.readString();
  Serial.println(T);
  T.trim();
  if(T.startsWith("+CMT:")){
    T=T.substring(T.indexOf("&"),T.indexOf("*"));
    Serial.println(T);
    Asigne_valores(T);
    Discrimine();
  }
  h_cortina=false;
  f_print=false;
  Serial1.flush();  
}

void serialEvent3(){
  if(!e_rcv){return;}
  String T=Serial3.readString();
  Serial.println(T);
  T.trim();
  Asigne_valores(T);
  if(data[1]=="beep"){Beep1();}
  if(data[1]=="print" and data[2]=="3"){Impresion_m3();}
  if(data[1]=="print" and data[2]=="4"){Impresion_m4();}
  h_cortina=false;
  f_print=false;
  Serial1.flush();  
}

void Valide_reset(){
  boolean reset_m1=digitalRead(rst_m1);
  if(!reset_m1){
    pinMode(rst_m1,OUTPUT);
    digitalWrite(rst_m1,HIGH);
    delay(500);
    pinMode(rst_m1,INPUT_PULLUP);
    Cabezote_m1(0);
    acc_bomba1=false;
    Off_bomba(1);
    Data_screen_m1();
    reset_m1=digitalRead(rst_m1);
    if(!reset_m1){
      Cabezote_m1(0);
      for(int j=0;j<2000;j++){
        delay(1);
      }
      b_todo=false;
      Rst1();
    }
    Show_lbl1_m1();
    Show_lbl1_m2();
  }
 
  boolean reset_m2=digitalRead(rst_m2);
  if(!reset_m2){
    pinMode(rst_m2,OUTPUT);
    digitalWrite(rst_m2,HIGH);
    delay(500);
    pinMode(rst_m2,INPUT_PULLUP);
    Cabezote_m2(0);
    acc_bomba1=false;
    Off_bomba(2);
    Data_screen_m2();
    reset_m2=digitalRead(rst_m2);
    if(!reset_m2){
      Cabezote_m2(0);
      for(int j=0;j<2000;j++){
        delay(1);
      }
      b_todo=false;
      Rst2();
    }
    Show_lbl1_m1();
    Show_lbl1_m2();
  }
}

void Init_Proceso(){
  if(h_touch==1){
    int btn1_x=190;
    int btn1_y=170; 
    int btn2_y=10;
    if(rt_dsp==3){
      btn1_x=150;
      btn1_y=165; 
      btn2_y=1; 
    }
    if(rt_dsp==1){
      btn1_x=150;
      btn1_y=1; 
      btn2_y=165;       
    }
    if(pixel_x>btn1_x&&pixel_x<(btn1_x+140)&&pixel_y>btn1_y&&pixel_y<(btn1_y+140)&&!acc_bomba1){ //x=190,330 y=170,310
      unsigned long a=EEPROM.get(132,a);        //Cantidad parcial del recipiente modulo1
      unsigned long b=EEPROM.get(120,b);        //Acumulado restante contenedor modulo1
      if(b<a){
        Pre_lcd();
        msg="Llenar tanque 1 vacio...";
        mensaje1=msg;
        Presente_datos(true);
        Mensajea();
        return;
      }
      acc_bomba1=true;
      Cabezote_m1(3);
      Start_proceso(1);
      h_cortina=false; 
      h_cortina_t=false;
      h_touch_1=true;            //aqui
      catch_Tiempo0=millis(); 
    }
    if(pixel_x>btn1_x&&pixel_x<(btn1_x+140)&&pixel_y>btn2_y&&pixel_y<(btn2_y+140)&&!acc_bomba2){ //x=190,330 y=10,150
      unsigned long a=EEPROM.get(232,a);        //Cantidad parcial del recipiente modulo2
      unsigned long b=EEPROM.get(220,b);        //Acumulado restante contenedor modulo2
      if(b<a){
        Pre_lcd();
        msg="Llenar tanque 2 vacio...";
        mensaje1=msg;
        Presente_datos(true);
        Mensajea();
        return;
      }
      acc_bomba2=true;
      Cabezote_m2(3);
      Start_proceso(2);
      h_cortina=false; 
      h_cortina_t=false;
      h_touch_2=true;            //aqui
      catch_Tiempo1=millis();  
    }
  }
}

void Asigne_valores(String D){
  for (int i=0;i<dataLength;i++){
     int index=D.indexOf(separator);
     data[i]=D.substring(0,index);   //toInt()
     D=D.substring(index+1);
  } 
}

void Discrimine(){
  Pre_lcd();
  msg="Configurando Dispensador Touch V1.0...";
  mensaje1=msg;
  Presente_datos(true);
  if(data[11]!=code){
    msg=(F("Denegado.."));
    mensaje1=msg;
    Presente_datos(true);
    goto salgaD;
  }
  
  for(int j=0;j<12;j++){
    Serial.println("data_"+String(j)+":"+String(data[j])); 
  }

  if(data[1]=="PnAdm"){     //Programa numero celular para programacion
    mensaje1="#,Grabando numero Adm,"+data[2]+",*"+"\r\n";
    msg=mensaje1;
    unsigned long NumAdm=data[2].toInt();
    int d=600;
    EEPROM.put(d,NumAdm); 
  }
  
  if(data[1]=="Pprod"){     //Programa producto
    if(data[2]=="1"){
      int nprod1=data[3].length();
      char c;
      EEPROM.write(1499,nprod1);
      for(int j=1500;j<1500+nprod1;j++){
        EEPROM.write(j,data[3][j-1500]);
      }
      Cabezote_m1(1);
      Pre_lcd();
    }
    if(data[2]=="2"){
      int nprod2=data[3].length();
      char c;
      EEPROM.write(1549,nprod2);
      for(int j=1550;j<1550+nprod2;j++){
        EEPROM.write(j,data[3][j-1550]);
      }
      Cabezote_m2(1);
      Pre_lcd();
    }
    msg=mensaje1;
  }
  if(data[1]=="Ppuls"){       //Pulsos por cantidad de producto
    if(data[2]=="1"){
      unsigned long t=data[3].toInt();
      EEPROM.put(108,t);
      Data_screen_m1();
      Pre_lcd();
    }
    if(data[2]=="2"){
      unsigned long t=data[3].toInt();
      EEPROM.put(208,t);
      Data_screen_m2();
      Pre_lcd();
    }
    msg=mensaje1;
  }
  if(data[1]=="Pvlor"){       //Valor de cada porcion servida
    if(data[2]=="1"){
      unsigned long t=data[3].toInt();
      EEPROM.put(104,t);
      Data_screen_m1();
      Pre_lcd();
    }
    if(data[2]=="2"){
      unsigned long t=data[3].toInt();
      EEPROM.put(204,t);
      Data_screen_m2();
      Pre_lcd();
    }
    msg=mensaje1;
  }
  if(data[1]=="Pctot"){        //Cantidad total del recipiente contenedor
    if(data[2]=="1"){
      unsigned long t=data[3].toInt();
      EEPROM.put(100,t);
      Data_screen_m1();
      Pre_lcd();
    }
    if(data[2]=="2"){
      unsigned long t=data[3].toInt();
      EEPROM.put(200,t);
      Data_screen_m2();
      Pre_lcd();
    }
    msg=mensaje1;
  }
  if(data[1]=="Pcpar"){       //Cantidad parcial del recipiente contenedor
    if(data[2]=="1"){
      unsigned long t=data[3].toInt();
      EEPROM.put(132,t);
      Data_screen_m1();
      Pre_lcd();
    }
    if(data[2]=="2"){
      unsigned long t=data[3].toInt();
      EEPROM.put(232,t);
      Data_screen_m2();
      Pre_lcd();
    }
    msg=mensaje1;
  }
  if(data[1]=="Rdfab"){      //Restablecer datos de fabrica
    if(data[2]=="1"){
      Carga_fab();
      Data_screen_m1();
      Pre_lcd();
    }
    if(data[2]=="2"){
      Carga_fab();
      Data_screen_m2();
      Pre_lcd();
    }
    msg=mensaje1;
  }
  if(data[1]=="Pmpre"){      //Modo de trabajo sensor presencia
    if(data[2]=="1"){
      unsigned long t=data[3].toInt();
      EEPROM.put(508,t);
      Data_screen_m1();
      Pre_lcd();
    }
    if(data[2]=="2"){
      unsigned long t=data[3].toInt();
      EEPROM.put(512,t);
      Data_screen_m2();
      Pre_lcd();
    }
    msg=mensaje1;
  }
  if(data[1]=="RstD"){       //Reseteo de informacion por SW
    if(data[3]=="1"){b_todo=true;}else{b_todo=false;}
    if(data[2]=="1"){
      Rst1();
      Data_screen_m1();
      Pre_lcd();
    }
    if(data[2]=="2"){
      Rst2();
      Data_screen_m2();
      Pre_lcd();
    }
    msg=mensaje1;
  }
  if(data[1]=="PmTlo"){      //Programa tiempo de visualizacion de imagenes y es el tiempo vacio
    unsigned long t=data[2].toInt();
    EEPROM.put(516,t);
    t=EEPROM.get(516,t);
    mensaje1="#,Grabando tiempo logo,"+String(t)+",*"+"\r\n";
    msg=mensaje1;
  }

  if(data[1]=="PrPreFill"){      //Programa tiempo de vacio despues de iniciar proceso
    unsigned long t=data[2].toInt();
    EEPROM.put(500,t);
    EEPROM.put(504,t);
    t_max_m1=EEPROM.get(500,t_max_m1);
    t_max_m2=EEPROM.get(504,t_max_m2);
    t=EEPROM.get(500,t);
    mensaje1="#,Grabando tiempo vacio,"+String(t_max_m1)+",*"+"\r\n";
    msg=mensaje1;
  }

  if(data[1]=="VelB1"){      //Programa la velocidad de la bomba1
    unsigned long t=data[2].toInt();
    EEPROM.put(308,t);
    vel_bomba1=EEPROM.get(308,vel_bomba1);
    t=EEPROM.get(308,t);
    mensaje1="#,Grabando velocidad bomba1,"+String(vel_bomba1)+",*"+"\r\n";
    msg=mensaje1;
  }

  if(data[1]=="VelB2"){      //Programa la velocidad de la bomba2
    unsigned long t=data[2].toInt();
    EEPROM.put(408,t);
    vel_bomba2=EEPROM.get(408,vel_bomba2);
    t=EEPROM.get(408,t);
    mensaje1="#,Grabando velocidad bomba2,"+String(vel_bomba2)+",*"+"\r\n";
    msg=mensaje1;
  }

  if(data[1]=="RampB1"){      //Programa la rampa de inicio de la bomba1
    unsigned long t=data[2].toInt();
    EEPROM.put(310,t);
    ram_bomba1=EEPROM.get(310,ram_bomba1);
    t=EEPROM.get(310,t);
    mensaje1="#,Grabando rampa acc bomba1,"+String(ram_bomba1)+",*"+"\r\n";
    msg=mensaje1;
  }

  if(data[1]=="RampB2"){      //Programa la rampa de inicio de la bomba2
    unsigned long t=data[2].toInt();
    EEPROM.put(410,t);
    ram_bomba2=EEPROM.get(410,ram_bomba2);
    t=EEPROM.get(410,t);
    mensaje1="#,Grabando ramp acc bomba2,"+String(ram_bomba2)+",*"+"\r\n";
    msg=mensaje1;
  }

  if(data[1]=="ChArc"){            //Cargue archivo de la RED via GPRS
    while(!Cargue_archivo()==1){   //Hagalo hasta que se complete
      Pre_lcd();
      Cargue_archivo();
      if(intentos_dw>3){
        intentos_dw=0;
        mensaje1="#,Unload,";
        goto s1;
      }
    }
    mensaje1="#,load,";
s1:
    mensaje1+=data[3]+','+data[4]+','+String(tamano)+",*"+"\r\n";
    mensaje1+=resumen;
    msg=mensaje1;
  }
  if(data[1]=="ShArc"){             //Inicia muestra de archivos graficos
    unsigned long t=EEPROM.get(516,t);
    h_cortina=true;
    h_cortina_t=true;
    mensaje1="#,Lanzando logo(s),"+String(t)+",*"+"\r\n";
    msg=mensaje1;
  }
  if(data[1]=="ShAcc"){            //Muestre los acumulados por bloque
    if(data[2]=="1"){
      Data_screen_m1();
      msg=mensaje1;
      Pre_lcd();
    }
    if(data[2]=="2"){
      Data_screen_m2();
      msg=mensaje1;
      Pre_lcd();
    }
  }
  if(data[1]=="WpArc"){            //Haga un Wipe a la SD
    msg="Haciendo un WIPE a SD...";
    mensaje1=msg;
    Presente_datos(true);
    Wipe_arch();
  }                                //ARREGLAR
  if(data[1]=="ErArc"){            //Borre un archivo *.* de la SD
    int a=data[2].length();
    a=data[2].length();
    int dir=1300;
    for(int j=0;j<a;j++){          //Guarda nombre de archivo a borrar 1300 de EEPROM
      EEPROM.write(dir+j,data[2][j]);
    }
    char arch[a+1]={};
    for(int j=0;j<a;j++){
      arch[j]=EEPROM.read(dir+j);
    }  
    msg="Borrando archivo:"+String(arch)+'('+String(a)+')'+'\n';
    if(!SD.remove(arch)){msg+="Operacion abortada...\n";}else{msg+="Borrado exitoso...\n";}
    Presente_datos(true);
    root=SD.open("/");
    printDirectory(root,0);
    msg+=mensaje1;
    Presente_datos(true);
  } 
  if(data[1]=="LsArc"){          //Liste los archivos *.* de la SD
    root=SD.open("/");
    printDirectory(root,0);
    msg=mensaje1;
    Presente_datos(true);
  } 
  if(data[1]=="EnTou"){          //Habilita touch sirve para habilitar funcionamiento de maquina
    msg="Touch habilitado:";
    msg+=String(data[2]);
    mensaje1=msg;
    Presente_datos(true);
    h_touch=data[2].toInt();
    EEPROM.write(520,h_touch);
  } 
  if(data[1]=="PrRdsp"){          //Programa la rotacion del display
    msg="Rotacion del display:";
    msg+=String(data[2]);
    mensaje1=msg;
    Presente_datos(true);
    rt_dsp=data[2].toInt();
    EEPROM.write(524,rt_dsp);
    tft.setRotation(rt_dsp);
  } 
  
  if(data[1]=="PrLbLBt"){        //Programa el titulo para productos
    int a=data[2].length();
    unsigned int dir=140;
    for(int j=0;j<a;j++){        //Guarda label boton 1 desde 140 de EEPROM
      EEPROM.write(dir+j,data[2][j]);
      lbl_m1[j]=char(EEPROM.read(140+j));
    }
    a=data[3].length();
    dir=240;
    for(int j=0;j<a;j++){        //Guarda label boton 2 desde 240 de EEPROM
      EEPROM.write(dir+j,data[3][j]);
      lbl_m2[j]=char(EEPROM.read(240+j));
    }
    msg="Programando nombres de botones:";
    msg+=String(data[2]);
    msg+="-";
    msg+=String(data[3]);
    mensaje1=msg;
    Presente_datos(true);
  }
  if(data[1]=="EnSMS"){          //Habilita envio de SMS's periodicos 
    msg="Envio SMS's periodicos habilitado:";
    msg+=String(data[2]);
    mensaje1=msg;
    Presente_datos(true);
    byte t=data[2].toInt();
    EEPROM.write(8,t);          //Habilitacion en byte 8 de la EEPROM
  } 
  if(data[1]=="PrNumAdm"){      //Adhiere un nuevo numero de administrador
    NumAdmon=EEPROM.read(599);
    unsigned long num;
    num=data[2].toInt();
    int d=NumAdmon*4+600;      //600 es el vector de numeros de administrador x 4 bytes cada uno
    EEPROM.put(d,num);
    msg="Numero Adm:";
    msg+=EEPROM.get(d,num);
    msg+="->(";msg+=d;msg+=")\n";
    NumAdmon++;
    EEPROM.write(599,NumAdmon);
    mensaje1=msg;
  }
  if(data[1]=="RdNumAdm"){     //Lee los numeros grabados de administrador
    NumAdmon=EEPROM.read(599);
    unsigned long num;
    unsigned int d=600;
    msg="Numeros programados:";
    msg+=NumAdmon;
    msg+='\n';
    for(int j=0;j<NumAdmon;j++){
      msg+=EEPROM.get(d,num);
      msg+="->(";msg+=d;msg+=")\n";
      d+=4;
    }
    mensaje1=msg;
  }
  if(data[1]=="BrNumAdm"){     //Borra los numeros grabados de administrador
    NumAdmon=254;
    EEPROM.write(599,NumAdmon);
    msg="Borrados numeros administradores\n";
    mensaje1=msg;
  }
  if(data[1]=="CfPuls"){  //Envia a proceso de configuracion pulsos por cantidad de porducto
    unsigned long t;
    if(data[2]=="1"){
      Show_config1(); 
    }
    if(data[2]=="2"){
      Show_config2(); 
    }
    msg="Configuracion de pulsos "+data[2]+"="+String(pulsos)+'\n';
    mensaje1=msg;
  }
  if(data[1]=="T_logo"){     //Programa el tiempo de cada transicion publicitaria
    t_logo=data[2].toInt();
    EEPROM.put(516,t_logo);
    msg="Tiempo Logos "+data[2]+"="+String(t_logo)+'\n';
    mensaje1=msg;
  }
  if(data[1]=="Print"){     //Imprime recibo
    if(data[2]=="1"){Impresion_m1();}
    if(data[2]=="2"){Impresion_m2();}
    if(data[2]=="3"){Impresion_m3();}   
    if(data[2]=="4"){Impresion_m4();}  
    msg="Imprimiendo "+data[2];
    mensaje1=msg;
  }
salgaD:  
  Stop_proceso(1);
  Stop_proceso(2);
  if(fwd){Mensajea();}
  Carga_data_m1();
  Carga_data_m2();
  delay(1000L);
  tft.fillScreen(WHITE);
  Show_lbl1_m1();
  Show_lbl1_m2();
}

void Pre_lcd(){
  tft.fillScreen(WHITE);
  tft.setCursor(0,0);
  tft.setTextSize(2);  
}

void Show_config1(){
  Pre_lcd();
  Cabezote_m1(7);
  tft.println("M1");
  M1_btn.initButton(&tft,120,250,200,100,WHITE,CYAN,BLACK,"PARAR",5); //Este es la declaracion del boton 1
  M1_btn.drawButton(false);
  On_bomba(1);
  tft.setCursor(150,120);
  tft.setTextColor(BLUE);
  long int t=0;
  cont_pul1=t;
  while(!Touch_getXY()){
    if(cont_pul1!=t){
      tft.fillRect(140,120,150,60,WHITE);
      tft.setCursor(145,120);
      tft.print(t); 
      t=cont_pul1;
    }
  }
  pulsos=t;
  pulsos_operacion_m1=t;
  EEPROM.put(108,pulsos_operacion_m1);
  Off_bomba(1);
}

void Show_config2(){
  Pre_lcd();
  Cabezote_m1(7);
  tft.println("M2");
  M1_btn.initButton(&tft,360,250,200,100,WHITE,CYAN,BLACK,"PARAR",5); //Este es la declaracion del boton 2
  M1_btn.drawButton(false);
  On_bomba(2);
  tft.setCursor(150,120);
  tft.setTextColor(BLUE);
  long int t=0;
  cont_pul1=t;
  while(!Touch_getXY()){
    if(cont_pul2!=t){
      tft.fillRect(140,120,150,60,WHITE);
      tft.setCursor(145,120);
      tft.print(t);           //cont_pul2
      t=cont_pul2;
    }
  }
  pulsos=t;
  pulsos_operacion_m2=t;
  EEPROM.put(208,pulsos_operacion_m2);
  Off_bomba(2);
}

void Show_lbl1_m1(){
  Cabezote_m1(1);
  Barra_estado1(0);
  M1_btn.initButton(&tft,120,145,200,100,WHITE,CYAN,BLACK,"1000mL",5); //Este es la declaracion del boton 1
  M1_btn.drawButton(false);
  tft.fillRect(0,200,240,40,WHITE);
}

void Show_lbl1_m2(){
  Cabezote_m2(2);
  Barra_estado2(0);
  M2_btn.initButton(&tft,360,145,200,100,WHITE,CYAN,BLACK,"1000mL",5); //Este es la declaracion del boton 1
  M2_btn.drawButton(false);
  tft.fillRect(240,200,240,40,WHITE);
}

void Cabezote_m1(int j){ //0-Borrando... 3-"Acerque el recipiente" 4-"Llenando" 5-"Valor a Pagar" 6-"Imprimiendo Tickete" 7-"Gracias..."
  tft.fillRect(0,0,480,40,WHITE);
  tft.setTextColor(BLACK);
  tft.setCursor(10,10);
  tft.setTextSize(4);
  if(j==1){
    if(EEPROM.read(1499)>49){EEPROM.write(1499,1);}
    for(int p=1500;p<1500+EEPROM.read(1499);p++){
      char c=EEPROM.read(p);
      tft.print(c);
    }
  }else{
    tft.print(labels[j]);
    }
}

void Cabezote_m2(int j){
  tft.fillRect(0,280,480,40,WHITE); //0-Borrando... 3-"Acerque el recipiente" 4-"Llenando" 5-"Valor a Pagar" 6-"Imprimiendo Tickete" 7-"Gracias..."
  tft.setTextColor(BLACK);
  tft.setCursor(120,280);
  tft.setTextSize(4);
  if(j==2){
    if(EEPROM.read(1549)>49){EEPROM.write(1549,1);}
    for(int p=1550;p<1550+EEPROM.read(1549);p++){
      char c=EEPROM.read(p);
      tft.print(c);
    }
  }else{
    tft.print(labels[j]);
  }
}

void Beep(){
  Serial3.println("&,beep,1,,,,,,,,,Avtec1832,1,*");
  Serial3.flush();
  for(int t=0;t<50;t++){
    digitalWrite(buzz,HIGH);
    delay(2);
    digitalWrite(buzz,LOW);
    delay(2);
  }
  catch_Tiempo8=millis();
  tempPul1=0;
  tempPul2=0;
}

void Beep1(){
  for(int t=0;t<50;t++){
    digitalWrite(buzz,HIGH);
    delay(2);
    digitalWrite(buzz,LOW);
    delay(2);
  }
  catch_Tiempo8=millis();
  tempPul1=0;
  tempPul2=0;
}

void On_bomba(int j){
  if(b1_e||b2_e){return;}
  Serial.println("rutina_On");
  int pwm1=EEPROM.get(308,pwm1);
  pwm1=map(pwm1,0,100,40,255);
  int t_acc1=EEPROM.get(310,t_acc1);
  int pwm2=EEPROM.get(408,pwm2);
  pwm2=map(pwm2,0,100,40,255);
  int t_acc2=EEPROM.get(410,t_acc2);  
  if(j==1){
    for(int j=40;j<pwm1;j++){
      analogWrite(bomba1,j);
      delay(t_acc1/(pwm1/40)*10); 
    }
    analogWrite(bomba1,pwm1);
    b1_e=true;
  }    
  if(j==2){
    for(int j=40;j<pwm2;j++){
      analogWrite(bomba2,j);
      delay(t_acc2/(pwm2/40)*10); 
    }
    analogWrite(bomba2,pwm2);
    b2_e=true;
  }
}

void Off_bomba(int j){
  if(j==1){tempPul1=0;b1_e=false;digitalWrite(bomba1,LOW);}
  if(j==2){tempPul2=0;b2_e=false;digitalWrite(bomba2,LOW);}
}

void Start_proceso(int j){
  if(j==1){
    Beep();
    Serial.println("Ini_bomba 1");
    Barra_estado1(1);
    M1_btn.drawButton(true);
    cont_pul1=0;
    cont_pul1_old=0;
    h_dist_m1=true;
    b_dist_m1=true;
    catch_Tiempo9=millis();
    catch_Tiempo13=millis();
  }
  if(j==2){
    Beep();
    Serial.println("Ini_bomba 2");
    Barra_estado2(1);
    M2_btn.drawButton(true); 
    cont_pul2=0;
    cont_pul2_old=0;
    h_dist_m2=true;
    b_dist_m2=true;
    catch_Tiempo10=millis();
    catch_Tiempo14=millis();
  }
}

void Stop_proceso(int j){
  if(j==1){
    Beep();
    Serial.println("Fin_ bomba 1");
    tft.fillRect(0,200,240,40,WHITE);
    acc_bomba1=false;
    Off_bomba(j);
    cont_pul1=0;
    cont_pul1_old=0;
    Barra_estado1(0);
    Cabezote_m1(1);
    M1_btn.drawButton(false);
    h_cortina_t=true;
    h_touch_1=false;
    catch_Tiempo8=millis();
  }
  if(j==2){
    Beep();
    Serial.println("Fin_bomba 2");
    tft.fillRect(240,200,480,40,WHITE);
    acc_bomba2=false;
    Off_bomba(j);
    cont_pul2=0;
    cont_pul2_old=0;
    Barra_estado2(0);
    Cabezote_m2(2);
    M2_btn.drawButton(false);
    h_cortina_t=true; 
    h_touch_2=false;
    catch_Tiempo8=millis();
  }
}

void ConteoP1(){
  cont_pul1++;
}

void ConteoP2(){
  cont_pul2++;
}

void Rst1(){
  Carga_data_m1();
  for(byte j=0;j<5;j++){
    Beep();
  }
  M1_btn.drawButton(true);
  Barra_estado1(3);
  cont_pul1=0;
  acum_vertido_m1=0;                                    //EEPROM(116)unsigned long
  EEPROM.put(116,acum_vertido_m1);
  acum_restante_m1=cant_total_ml;                       //EEPROM(120)unsigned long
  EEPROM.put(120,acum_restante_m1);
  acum_operaciones_m1_carga=0;                          //EEPROM(124)unsigned long
  EEPROM.put(124,acum_operaciones_m1_carga);
  if(b_todo==true){
    acum_operaciones_m1_total=0;                        //EEPROM(128)unsigned long
    EEPROM.put(128,acum_operaciones_m1_total);
    acum_dinero_m1=0;                                   //EEPROM(112)unsigned long 
    EEPROM.put(112,acum_dinero_m1);
    b_todo=false;
  }
}

void Rst2(){
  Carga_data_m2();
  for(byte j=0;j<5;j++){
    Beep();
  }
  M2_btn.drawButton(true);
  Barra_estado2(3);
  cont_pul2=0;
  acum_vertido_m2=0;                                    //EEPROM(216)unsigned long
  EEPROM.put(216,acum_vertido_m2);
  acum_restante_m2=cant_total_m2;                       //EEPROM(220)unsigned long
  EEPROM.put(220,acum_restante_m2);
  acum_operaciones_m2_carga=0;                          //EEPROM(224)unsigned long
  EEPROM.put(224,acum_operaciones_m2_carga);
  if(b_todo==true){
    acum_operaciones_m2_total=0;                        //EEPROM(228)unsigned long
    EEPROM.put(228,acum_operaciones_m2_total);
    acum_dinero_m2=0;                                   //EEPROM(212)unsigned long 
    EEPROM.put(212,acum_dinero_m2);
    b_todo=false;
  }
}

void Barra_estado1(int j){
  int px=0,py=40,ax=360,ay=10;
  tft.fillRect(px,py,ax,ay,color[j]);
}

void Barra_estado2(int j){
  int px=120,py=265,ax=480,ay=10;
  tft.fillRect(px,py,ax,ay,color[j]);
}

void Impresion_m1(){
  Serial.println("Imprimiendo recibo de m1...");
  delay(500);
  printer.wake();   
  printer.justify('C');
  printer.println(lbl_m1);
  printer.justify('L');
  unsigned long t=cant_porcion_m1;
  printer.println("Cantidad vendida: "+String(t)+" mL");    //ok  Cantidad vendida mL
  t=dinero_operacion_m1;
  printer.println("Valor vendido: $ "+String(t));           //ok  Valor Vendido $
  // printer.justify('C');
  // printer.printBarcode("1234567890123", EAN13);
  // printer.feed(3); 
  printer.setDefault();
  printer.sleep();
  delay(500);
}

void Impresion_m2(){
  Serial.println("Imprimiendo recibo de m2...");
  delay(500);
  printer.wake();  
  printer.justify('C');
  printer.println(lbl_m2);
  printer.justify('L');
  unsigned long t=cant_porcion_m1;
  printer.println("Cantidad vendida: "+String(t)+" mL");    //ok  Cantidad vendida mL
  t=dinero_operacion_m1;
  printer.println("Valor vendido: $ "+String(t));           //ok  Valor Vendido $
  // printer.justify('C');
  // printer.printBarcode("1234567890123", EAN13);
  // printer.feed(1);  
  printer.setDefault();
  printer.sleep();
  delay(500);  
}

void Impresion_m3(){
  Serial.println("Imprimiendo recibo de m3...");
  printer.setDefault();
  delay(500);
  printer.wake();  
  printer.justify('C');
  printer.print(data[5]+"\t");
  printer.print(data[3]);
  printer.write(0x20);
  printer.println("mL");  //ok  Cantidad vendida mL
  printer.println("$ "+String(data[4]));  //ok  Valor Vendido $
  printer.println();
  printer.println();
  printer.println();
  // printer.justify('C');
  // printer.setBarcodeHeight(50);
  // printer.printBarcode("1234567890123", UPC_A);
  // printer.feed(1);  
//  printer.setDefault();

  printer.sleep();
  delay(500);  
}

void Impresion_m4(){
  Serial.println("Imprimiendo recibo de m4...");
  digitalWrite(print_cst,LOW);
  delay(500);
  printer.wake();  
  printer.justify('C');
  printer.println(data[5]);
  printer.justify('L');
  printer.println("Cantidad vendida: "+String(data[3])+" mL");    //ok  Cantidad vendida mL
  printer.println("Valor vendido: $ "+String(data[4]));           //ok  Valor Vendido $
  // printer.justify('C');
  // printer.printBarcode("1234567890123", EAN13);
  // printer.feed(1);  
  printer.setDefault();
  printer.sleep();
  delay(500);  
}

void Liquida_m1(){
  Serial.println("Liquidando m1...");
  Carga_data_m1();
  unsigned long t=cant_porcion_m1;
  mensaje2=("Cantidad vendida: "+String(t)+" mL\n");    //ok  Cantidad vendida mL
  t=dinero_operacion_m1;
  mensaje2+=("Valor vendido: $ "+String(t)+'\n');       //ok  Valor Vendido $
  t=acum_vertido_m1+cant_porcion_m1;EEPROM.put(116,t);
  mensaje2+=("Cantidad vertida:"+String(t)+" mL\n");    //ok  Cantidad Vertida mL Lo que le he sacado al tanque
  t=dinero_operacion_m1+acum_dinero_m1;EEPROM.put(112,t);                                         
  mensaje2+=("Valor acumulado: $ "+String(t)+'\n');     //ok  Dinero acumulado $
  t=EEPROM.get(116,t);
  unsigned long c=cant_total_ml-t;EEPROM.put(120,c);
  mensaje2+=("Cantidad restante:"+String(c)+" mL\n");   //ok  Cantidad restante mL 
  t=acum_operaciones_m1_carga+1;EEPROM.put(124,t);
  mensaje2+=("Operaciones carga: "+String(t)+'\n');     //ok  Cantidad operaciones por carga
  t=acum_operaciones_m1_total+1;EEPROM.put(128,t);
  mensaje2+=("Operaciones total: "+String(t)+'\n');     //ok  Cantidad operaciones total  
  SerialMon.println(mensaje2);  
}

void Liquida_m2(){
  Serial.println("Liquidando m2...");
  Carga_data_m2();
  unsigned long t=cant_porcion_m2;
  mensaje2=("Cantidad vendida: $ "+String(t)+" mL\n");    //ok  Cantidad vendida mL
  t=dinero_operacion_m2;
  mensaje2+=("Valor vendido:"+String(t)+'\n');            //ok  Valor Vendido $
  t=acum_vertido_m2+cant_porcion_m2;EEPROM.put(216,t);
  mensaje2+=("Cantidad vertida:"+String(t)+" mL\n");      //ok  Cantidad Vertida mL Lo que le he sacado al tanque
  t=dinero_operacion_m2+acum_dinero_m2;EEPROM.put(212,t);                                         
  mensaje2+=("Valor acumulado: $ "+String(t)+'\n');       //ok  Dinero acumulado $
  t=EEPROM.get(216,t);
  unsigned long c=cant_total_m2-t;EEPROM.put(220,c);
  mensaje2+=("Cantidad restante:"+String(c)+" mL\n");     //ok  Cantidad restante mL 
  t=acum_operaciones_m2_carga+1;EEPROM.put(224,t);
  mensaje2+=("Operaciones carga: "+String(t)+'\n');       //ok  Cantidad operaciones por carga
  t=acum_operaciones_m2_total+1;EEPROM.put(228,t);
  mensaje2+=("Operaciones total: "+String(t)+'\n');       //ok  Cantidad operaciones total  
  SerialMon.println(mensaje2);  
}

void Show_data1(unsigned long j, int h){
  tft.drawRect(20,200,200,38,BLACK);
  unsigned long tporc=EEPROM.get(108,tporc);
  unsigned int porcentaje=(j*100)/tporc;
  tft.fillRect(20,202,porcentaje*2,34,GREEN);
  if(porcentaje>=100){tft.drawRect(220,200,40,38,WHITE);}
}

void Show_data2(unsigned long j, int h){
  tft.drawRect(260,200,200,38,BLACK);
  unsigned long tporc=EEPROM.get(208,tporc);
  unsigned int porcentaje=(j*100)/tporc;
  tft.fillRect(260,202,porcentaje*2,34,GREEN);
}

void Carga_data_m1(){
  lbl_m1="";
  if(EEPROM.read(1499)>49){EEPROM.write(1499,1);}
  for(int p=1500;p<1500+EEPROM.read(1499);p++){
    char c=EEPROM.read(p);
    lbl_m1+=c;
  }
  unsigned long t;
  cant_total_ml=EEPROM.get(100,t);                   //EEPROM(100)unsigned long
  dinero_operacion_m1=EEPROM.get(104,t);             //EEPROM(104)unsigned long
  pulsos_operacion_m1=EEPROM.get(108,t);             //EEPROM(108)unsigned long
  acum_dinero_m1=EEPROM.get(112,t);                  //EEPROM(112)unsigned long
  acum_vertido_m1=EEPROM.get(116,t);                 //EEPROM(116)unsigned long
  acum_restante_m1=EEPROM.get(120,t);                //EEPROM(120)unsigned long
  acum_operaciones_m1_carga=EEPROM.get(124,t);       //EEPROM(124)unsigned long
  acum_operaciones_m1_total=EEPROM.get(128,t);       //EEPROM(128)unsigned long
  cant_porcion_m1=EEPROM.get(132,t);                 //EEPROM(132)unsigned long
  t_max_m1=EEPROM.get(500,t);                        //EEPROM(500)int Maximo tiempo de accionamiento bomba
  t_modo_dist_m1=EEPROM.read(508);                   //EEPROM(508)int Intervalos de medicion distancia
  h_touch=EEPROM.read(520);                          //EEPROM(520)int Habilita Touch
  rt_dsp=EEPROM.read(524);                           //EEPROM(524)int Rotacion Display
  t_logo=EEPROM.get(516,t);                          //EEPROM(516)int Tiempo de visualizacion de imagenes en SD
  mind1_s1=EEPROM.read(300);                         //EEPROM(300)Minima distancia sensor 1
  maxd1_s1=EEPROM.read(302);                         //EEPROM(302)Maxima distancia sensor 2
  mind1_s2=EEPROM.read(304);                         //EEPROM(304)Minima distancia sensor 1
  maxd1_s2=EEPROM.read(306);                         //EEPROM(306)Maxima distancia sensor 2
  vel_bomba1=EEPROM.read(308);                       //EEPROM(308)velocidad de la bomba 1 en porcentaje
  ram_bomba1=EEPROM.read(310);                       //EEPROM(310)rampa de aceleracion de bomba 1 en segundos
}


void Carga_data_m2(){
  lbl_m2="";
  if(EEPROM.read(1549)>49){EEPROM.write(1549,1);}
  for(int p=1550;p<1550+EEPROM.read(1549);p++){
    char c=EEPROM.read(p);
    lbl_m2+=c;
  }
  unsigned long t;
  cant_total_m2=EEPROM.get(200,t);                //EEPROM(200)unsigned long
  dinero_operacion_m2=EEPROM.get(204,t);          //EEPROM(204)unsigned long
  pulsos_operacion_m2=EEPROM.get(208,t);          //EEPROM(208)unsigned long
  acum_dinero_m2=EEPROM.get(212,t);               //EEPROM(212)unsigned long
  acum_vertido_m2=EEPROM.get(216,t);              //EEPROM(216)unsigned long
  acum_restante_m2=EEPROM.get(220,t);             //EEPROM(220)unsigned long
  acum_operaciones_m2_carga=EEPROM.get(224,t);    //EEPROM(224)unsigned long
  acum_operaciones_m2_total=EEPROM.get(228,t);    //EEPROM(228)unsigned long
  cant_porcion_m2=EEPROM.get(232,t);              //EEPROM(232)unsigned long
  t_max_m2=EEPROM.get(504,t);                     //EEPROM(504)int Maximo tiempo de accionamiento bomba
  t_modo_dist_m2=EEPROM.read(512);                //EEPROM(512)int Intervalos de medicion distancia
  h_touch=EEPROM.read(520);                       //EEPROM(520)int Habilita Touch
  rt_dsp=EEPROM.read(524);                        //EEPROM(524)int Rotacion Display
  t_logo=EEPROM.get(516,t);                       //EEPROM(516)int Tiempo de visualizacion de imagenes en SD
  mind2_s1=EEPROM.read(400);                      //EEPROM(400) Minima distancia sensor 1
  maxd2_s1=EEPROM.read(402);                      //EEPROM(402) Maxima distancia sensor 2
  mind2_s2=EEPROM.read(404);                      //EEPROM(404) Minima distancia sensor 2
  maxd2_s2=EEPROM.read(406);                      //EEPROM(406) Maxima distancia sensor 2
  vel_bomba2=EEPROM.read(408);                    //EEPROM(408)velocidad de la bomba 2 en porcentaje
  ram_bomba2=EEPROM.read(410);                    //EEPROM(410)rampa de aceleracion de bomba 2 en segundos
}

void Carga_fab(){
  unsigned long tdata=100;
  EEPROM.put(108,tdata);                                    //Cantidad de pulsos m1
  tdata=200;
  EEPROM.put(208,tdata);                                    //Cantidad de pulsos m2
  tdata=20000;
  EEPROM.put(100,tdata);                                    //Capacidad de liquido m1
  EEPROM.put(200,tdata);                                    //Capacidad de liquido m2
  EEPROM.put(120,tdata);                                    //Capacidad restante de liquido m1
  EEPROM.put(220,tdata);                                    //Capacidad restante de liquido m2
  tdata=1000;
  EEPROM.put(132,tdata);                                    //Cantidad de liquido m1 por porcion
  EEPROM.put(232,tdata);                                    //Cantidad de liquido m2 por porcion  
  unsigned int t=3;
  EEPROM.write(300,t);                                    //distancias trip de los sensores de presencia
  EEPROM.write(304,t);
  EEPROM.write(400,t);
  EEPROM.write(404,t);
  t=40;
  EEPROM.write(302,t);                                    //distancias trip de los sensores de presencia
  EEPROM.write(306,t);
  EEPROM.write(402,t);
  EEPROM.write(406,t);
  t=10;                               
  EEPROM.write(500,t);                                    //Valor maximo para poner recipiente
  EEPROM.write(504,t);
  unsigned long p=1200;                                   //Valor porcion vertida
  EEPROM.put(104,p);
  p=2000;
  EEPROM.put(204,p);
  p=100;                                                  //Modo sensor presencia
  EEPROM.put(508,p);
  p=100;
  EEPROM.put(512,p);
  p=30;                           
  EEPROM.put(516,p);                                      //Valor tiempo de mostrar imagenes e intervalo  
  p=1;                           
  EEPROM.put(520,p);                                      //Valor para habilitar touch 
  p=1;                           
  EEPROM.put(524,p);                                      //Valor de rotacion display rt_dsp  
  p=0;
  EEPROM.put(124,p);                                      //operaciones por carga m1
  EEPROM.put(224,p);                                      //operaciones por carga m2
  EEPROM.put(128,p);                                      //operaciones totales m1
  EEPROM.put(228,p);                                      //operaciones totales m2
  EEPROM.put(112,p);                                      //dinero acumulado carga m1
  EEPROM.put(212,p);                                      //dinero acumulado carga m2  
  EEPROM.put(116,p);                                      //cantidad vertida por carga m1
  EEPROM.put(216,p);                                      //cantidad vertida por carga m2
  p=100;
  EEPROM.put(308,p);                                      //Velocidad de bomba1
  EEPROM.put(408,p);                                      //Velocidad de bomba2
  p=10;
  EEPROM.put(310,p);                                      //Rampa inicial de bomba1
  EEPROM.put(410,p);                                      //Rampa inicial de bomba2
}

void Data_screen_m1(){
  tft.fillScreen(WHITE);
  tft.setTextSize(5);
  tft.setTextColor(BLACK);
  tft.setCursor(10,0);
  tft.println(lbl_m1);
  tft.setTextSize(3);
  tft.setTextColor(BLUE);
  tft.setCursor(0,50);
  unsigned long t;
  mensaje1="#,Resumen_1,";
  mensaje1+=lbl_m1+',';
  tft.println("Capacidad mL: "+(String)EEPROM.get(100,t));              //EEPROM(100)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Valor: $ "+(String)EEPROM.get(104,t));                   //EEPROM(104)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Senal op: "+(String)EEPROM.get(108,t));                  //EEPROM(108)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Dinero Acum: "+(String)EEPROM.get(112,t));               //EEPROM(112)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Modo presc: "+(String)EEPROM.get(508,t));                //EEPROM(508)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Cantidad Total: "+(String)EEPROM.get(100,t));            //EEPROM(100)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Cantidad Vertida: "+(String)EEPROM.get(116,t));          //EEPROM(116)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Cantidad Restante: "+(String)EEPROM.get(120,t));         //EEPROM(120)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Num. oper/carga: "+(String)EEPROM.get(124,t));           //EEPROM(124)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Num. oper/total: "+(String)EEPROM.get(128,t));           //EEPROM(128)unsigned long
  mensaje1+=String(t);
  mensaje1+=",*\r\n";
  for(int j=0;j<10000;j++){
    delay(1);
  }
  tft.fillScreen(WHITE);
  Serial.println(mensaje1);
}

void Data_screen_m2(){
  tft.fillScreen(WHITE);
  tft.setTextSize(5);
  tft.setTextColor(BLACK);
  tft.setCursor(10,0);
  tft.println(lbl_m2);
  tft.setTextSize(3);
  tft.setTextColor(BLUE);
  tft.setCursor(0,50);
  unsigned long t;
  mensaje1="#,Resumen_2,";
  mensaje1+=lbl_m2+',';
  tft.println("Capacidad mL: "+(String)EEPROM.get(200,t));             //EEPROM(200)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Valor: $ "+(String)EEPROM.get(204,t));                  //EEPROM(204)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Senal op: "+(String)EEPROM.get(208,t));                 //EEPROM(208)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Dinero Acum: "+(String)EEPROM.get(212,t));              //EEPROM(212)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Modo presc: "+(String)EEPROM.get(512,t));               //EEPROM(512)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Cantidad Total: "+(String)EEPROM.get(200,t));           //EEPROM(200)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Cantidad Vertida: "+(String)EEPROM.get(216,t));         //EEPROM(216)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Cantidad Restante: "+(String)EEPROM.get(220,t));        //EEPROM(220)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Num. oper/carga: "+(String)EEPROM.get(224,t));          //EEPROM(224)unsigned long
  mensaje1+=String(t)+',';
  tft.println("Num. oper/total: "+(String)EEPROM.get(228,t));          //EEPROM(228)unsigned long
  mensaje1+=String(t);
  mensaje1+=",*\r\n";
  for(int j=0;j<10000;j++){
    delay(1);
  }
  tft.fillScreen(WHITE);
  Serial.println(mensaje1);
}

void Muestre_logo(){
  char *nm=namebuf+pathlen;
  File f=root.openNextFile();
  uint8_t ret;
  uint32_t start;
  if (f!=NULL) {
  #ifdef USE_SDFAT
    f.getName(nm, 32-pathlen);
  #else
    strcpy(nm,(char *)f.name());
  #endif
  f.close();
  strlwr(nm);
  if (strstr(nm,".bmp")!=NULL&&strstr(nm, NAMEMATCH)!=NULL){
      Serial.print(namebuf);
      Serial.print(F(" - "));
      tft.fillScreen(0);
      start=millis();
      ret=showBMP(namebuf,1,1);
      switch (ret) {
          case 0:
              Serial.print(millis()-start);
              Serial.println(F("ms"));
              f_print=true;
              h_cortina=true;
              catch_Tiempo8=millis();
              break;
          case 1:
              Serial.println(F("mala posicion"));
              h_cortina=false;
              h_cortina_t=false;
              break;
          case 2:
              Serial.println(F("no BMP ID"));
              h_cortina=false;
              h_cortina_t=false;
              break;
          case 3:
              Serial.println(F("numero erroneo de planos"));
              h_cortina=false;
              h_cortina_t=false;
              break;
          case 4:
              Serial.println(F("Formato BMP no soportado"));
              h_cortina=false;
              h_cortina_t=false;
              break;
          case 5:
              Serial.println(F("paleta no soportada"));
              h_cortina=false;
              h_cortina_t=false;
              break;
          default:
              Serial.println(F("desconocido"));
              h_cortina=false;
              h_cortina_t=false;
              break;
      }
  }
  }
  else root.rewindDirectory();
  h_cortina=false;                      //mirar aqui para lo de la cortina
}

#define BMPIMAGEOFFSET 54
#define BUFFPIXEL      20

uint16_t read16(File& f) {
    uint16_t result;         // read little-endian
    f.read(&result, sizeof(result));
    return result;
}

uint32_t read32(File& f) {
    uint32_t result;
    f.read(&result, sizeof(result));
    return result;
}

uint8_t showBMP(char *nm, int x, int y){
  File bmpFile;
  int bmpWidth, bmpHeight;    // W+H in pixels
  uint8_t bmpDepth;           // Bit depth (currently must be 24, 16, 8, 4, 1)
  uint32_t bmpImageoffset;    // Start of image data in file
  uint32_t rowSize;           // Not always = bmpWidth; may have padding
  uint8_t sdbuffer[3 * BUFFPIXEL];    // pixel in buffer (R+G+B per pixel)
  uint16_t lcdbuffer[(1 << PALETTEDEPTH) + BUFFPIXEL], *palette = NULL;
  uint8_t bitmask, bitshift;
  boolean flip = true;        // BMP is stored bottom-to-top
  int w, h, row, col, lcdbufsiz = (1 << PALETTEDEPTH) + BUFFPIXEL, buffidx;
  uint32_t pos;               // seek position
  boolean is565 = false;      //

  uint16_t bmpID;
  uint16_t n;                 // blocks read
  uint8_t ret;

  if ((x >= tft.width()) || (y >= tft.height()))
      return 1;               // off screen

  bmpFile = SD.open(nm);      // Parse BMP header
  bmpID = read16(bmpFile);    // BMP signature
  (void) read32(bmpFile);     // Read & ignore file size
  (void) read32(bmpFile);     // Read & ignore creator bytes
  bmpImageoffset = read32(bmpFile);       // Start of image data
  (void) read32(bmpFile);     // Read & ignore DIB header size
  bmpWidth = read32(bmpFile);
  bmpHeight = read32(bmpFile);
  n = read16(bmpFile);        // # planes -- must be '1'
  bmpDepth = read16(bmpFile); // bits per pixel
  pos = read32(bmpFile);      // format
  if (bmpID != 0x4D42) ret = 2; // bad ID
  else if (n != 1) ret = 3;   // too many planes
  else if (pos != 0 && pos != 3) ret = 4; // format: 0 = uncompressed, 3 = 565
  else if (bmpDepth < 16 && bmpDepth > PALETTEDEPTH) ret = 5; // palette 
  else {
      bool first = true;
      is565 = (pos == 3);               // ?already in 16-bit format
      // BMP rows are padded (if needed) to 4-byte boundary
      rowSize = (bmpWidth * bmpDepth / 8 + 3) & ~3;
      if (bmpHeight < 0) {              // If negative, image is in top-down order.
          bmpHeight = -bmpHeight;
          flip = false;
      }

      w = bmpWidth;
      h = bmpHeight;
      if ((x + w) >= tft.width())       // Crop area to be loaded
          w = tft.width() - x;
      if ((y + h) >= tft.height())      //
          h = tft.height() - y;

      if (bmpDepth <= PALETTEDEPTH) {   // these modes have separate palette
          bmpFile.seek(BMPIMAGEOFFSET); //palette is always @ 54
          bitmask = 0xFF;
          if (bmpDepth < 8)
              bitmask >>= bmpDepth;
          bitshift = 8 - bmpDepth;
          n = 1 << bmpDepth;
          lcdbufsiz -= n;
          palette = lcdbuffer + lcdbufsiz;
          for (col = 0; col < n; col++) {
              pos = read32(bmpFile);    //map palette to 5-6-5
              palette[col] = ((pos & 0x0000F8) >> 3) | ((pos & 0x00FC00) >> 5) | ((pos & 0xF80000) >> 8);
          }
      }

      // Set TFT address window to clipped image bounds
      tft.setAddrWindow(x, y, x + w - 1, y + h - 1);
      for (row = 0; row < h; row++) { // For each scanline...
          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          uint8_t r, g, b, *sdptr;
          int lcdidx, lcdleft;
          if (flip)   // Bitmap is stored bottom-to-top order (normal BMP)
              pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else        // Bitmap is stored top-to-bottom
              pos = bmpImageoffset + row * rowSize;
          if (bmpFile.position() != pos) { // Need seek?
              bmpFile.seek(pos);
              buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col = 0; col < w; ) {  //pixels in row
              lcdleft = w - col;
              if (lcdleft > lcdbufsiz) lcdleft = lcdbufsiz;
              for (lcdidx = 0; lcdidx < lcdleft; lcdidx++) { // buffer at a time
                  uint16_t color;
                  // Time to read more pixel data?
                  if (buffidx >= sizeof(sdbuffer)) { // Indeed
                      bmpFile.read(sdbuffer, sizeof(sdbuffer));
                      buffidx = 0; // Set index to beginning
                      r = 0;
                  }
                  switch (bmpDepth) {          // Convert pixel from BMP to TFT format
                      case 24:
                          b = sdbuffer[buffidx++];
                          g = sdbuffer[buffidx++];
                          r = sdbuffer[buffidx++];
                          color = tft.color565(r, g, b);
                          break;
                      case 16:
                          b = sdbuffer[buffidx++];
                          r = sdbuffer[buffidx++];
                          if (is565)
                              color = (r << 8) | (b);
                          else
                              color = (r << 9) | ((b & 0xE0) << 1) | (b & 0x1F);
                          break;
                      case 1:
                      case 4:
                      case 8:
                          if (r == 0)
                              b = sdbuffer[buffidx++], r = 8;
                          color = palette[(b >> bitshift) & bitmask];
                          r -= bmpDepth;
                          b <<= bmpDepth;
                          break;
                  }
                  lcdbuffer[lcdidx] = color;

              }
              tft.pushColors(lcdbuffer, lcdidx, first);
              first = false;
              col += lcdidx;
          }           // end cols
      }               // end rows
      tft.setAddrWindow(0, 0, tft.width() - 1, tft.height() - 1); //restore full screen
      ret = 0;        // good render
  }
  bmpFile.close();
  return (ret);
}

bool Touch_getXY(void){
  TSPoint_kbv p=ts.getPoint();
  pinMode(YP,OUTPUT);                               //Necesario para no interferencia
  pinMode(XM,OUTPUT);
  digitalWrite(YP,HIGH);                            //because TFT control pins
  digitalWrite(XM,HIGH);
  bool pressed=(p.z>MINPRESSURE&&p.z<MAXPRESSURE);
  if(p.z>300){       //>570
    pressed=true;
    Beep();
  }
  if(pressed){
    pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width()); //.kbv makes sense to me alreves
    pixel_y = map(p.y, TS_BOT, TS_TOP, 0, tft.height());
    Serial.println("pixelX:"+String(pixel_x)+','+"pixelY:"+String(pixel_y)+','+"profZ:"+String(p.z));
    if(e_cortina){
      tft.fillScreen(WHITE);
      Show_lbl1_m1();
      Show_lbl1_m2(); 
      e_cortina=false;
      delay(2000);                  //Tiempo y borrar coordenadas cada que se despierte de la cortina
      pixel_x=0;
      pixel_y=0;
    }
  }
  return pressed;
}

void Envie_SMS(String T) {
  Serial.setTimeout(5000);
  msg=("SMS a Numero:"+T+"\r\n"+mensaje1);
  Presente_datos(true);
  Serial1.println(F("AT+CMGF=1")); 
  String D=Serial1.readString();
  msg=(D);
  Presente_datos(true);
  Serial1.print("AT+CMGS=\""+T+"\"\r\n");
  D=Serial1.readString();
  msg=(D);
  Presente_datos(true);  
  Serial1.println(mensaje1+(char)26+"\"\r\n"); 
  D=Serial1.readStringUntil("\r\n+CMGS:");               //Se adhirio y soluciono un problema
  D=Serial1.readString();
  msg="SMS enviado...";
  msg+=String(D); 
  Presente_datos(true);
  delay(3000L);
  Serial.setTimeout(100);
}

void Mensajea(){
  Pre_lcd();
  msg=(F("Inicia Envio SMS..."));
  Presente_datos(true);
  NumAdmon=EEPROM.read(599);
  int d=600;
  for(int j=0;j<NumAdmon;j++){
    unsigned long num=EEPROM.get(d,num);
    Envie_SMS((String)num);
    d+=4;
    delay(3000);
  }
  msg=("Finaliza Envio SMS...");
  Presente_datos(true);
}

boolean Configuracion() {
  msg="Configurando modulo GSM...";
  Presente_datos(true);
  const String C[12]={"AT+IPR=115200","ATE0","ATV1","AT+CLIP=1","AT+CMGF=1",
                    "AT+CSDH=1","AT+CNMI=2,2,0,0,0","AT+CMGD=1,4","ATS0=0",
                    "AT&W"};  //"AT+QGNSSC=1" para modulos con GNSS antes de guardar
  int j;
  int p=0;
  for(j=0;j<10;j++){
    Serial1.println(C[j]);
    String T=Serial1.readString();
    if(T=="\r\nOK\r\n"){p++;msg="--";Presente_datos(false);}else{msg="###";Presente_datos(false);}
    msg="->";
    Presente_datos(false);
  }
  if(p>8){
    msg=" ";
    Presente_datos(true);
    msg=(F("Modulo GSM Ok..."));
    Presente_datos(true);
    return true;
  }else{
    msg=" ";
    Presente_datos(true);
    msg=(F("Modulo GSM en fallo..."));
    Presente_datos(true); 
    return false;   
  }
  Serial.flush();
}

void printPercent(uint32_t readLength, uint32_t contentLength) {
  if (contentLength!=-1) {
    msg=("\r ");
    msg+=((100.0*readLength)/contentLength);
    msg+='%';
    SerialMon.print(msg);tft.print("*");      
  } else {
    msg=(readLength);
    Presente_datos(false); 
  }
}


void Guarde_registros(){
  int a=data[2].length();
  unsigned int dir=1000;
  for(int j=0;j<a;j++){                     //Guarda APN desde 1000 de EEPROM
    EEPROM.write(dir+j,data[2][j]);
  }
  a=data[3].length();
  dir=1100;
  for(int j=0;j<a;j++){                     //Guarda Servidor desde 1100 de EEPROM
    EEPROM.write(dir+j,data[3][j]);
  }
  a=data[4].length();
  dir=1200;
  for(int j=0;j<a;j++){                     //Guarda Resource desde 1200 de EEPROM
    EEPROM.write(dir+j,data[4][j]);
  }  
}

boolean Cargue_archivo(){      //2345 void
int  port=80;                  //80 es el comun para http
char user[]="";
char pass[]="";
unsigned int dir=1000;
int a=data[2].length();
  msg="Iniciando descarga, intento(";
  msg+=String(intentos_dw);
  msg+=")...\n";
  Presente_datos(true);
  if(data[5]=="E"){Guarde_registros();}       //el  quinto dato habilita o no el guardado de variables
  char apn[a+1]={};
  dir=1000;
  for(int j=0;j<a;j++){
    apn[j]=EEPROM.read(dir+j);
  }
  SerialMon.println("APN: "+String(apn)+'('+String(a)+')');
  a=data[3].length();
  char server[a+1]={};
  dir=1100;
  for(int j=0;j<a;j++){
    server[j]=EEPROM.read(dir+j);
  }  
  SerialMon.println("SERVER: "+String(server)+'('+String(a)+')');
  a=data[4].length();
  char resource[a+1]={};
  dir=1200;
  for(int j=0;j<a;j++){
    resource[j]=EEPROM.read(dir+j);
  } 
  SerialMon.println("RESOURCE: "+String(resource)+'('+String(a)+')');

  tft.fillScreen(WHITE);
  tft.setCursor(0,0);
  tft.setTextSize(2);
  String f=data[6];
  File temp_file=SD.open(f,FILE_WRITE);
  if(temp_file){  
    msg=(String(f)+"<--"+String(server)+"...");
    Presente_datos(true);
  }
  
  msg=(F("Esperando por RED..."));                      //Conectando a red por GPRS en Terminal y LCD
  Presente_datos(false);

  if (!modem.waitForNetwork()) {
    msg=(F(" fallo de RED GPRS"));
    Presente_datos(true);
    resumen=(F(" fallo de RED GPRS"));
    delay(10000);
    return  0;
  }
  msg=(F(" OK"));
  Presente_datos(true);

  msg=(F("Conectando a "));                             //Conectando a apn en Terminal y LCD
  Presente_datos(false);

  msg=(apn);
  Presente_datos(false);
  
  if (!modem.gprsConnect(apn,user,pass)) {
    msg=(F(" fallo de conexion"));
    Presente_datos(true);;
    resumen=(F(" fallo de conexion"));
    delay(10000);
    return  0;
  }
  msg=(F(" OK"));
  Presente_datos(true);

  msg=(F("Conectando a "));                             //Conectando a servidor en Terminal y LCD
  Presente_datos(false);
  
  msg=(server);
  Presente_datos(false);

  if (!client.connect(server, port)) {
    msg=(F(" fallo de servidor o puerto"));
    Presente_datos(true);
    resumen=(F(" fallo de servidor o puerto"));
    delay(10000);
    return  0;
  }
  msg=(F(" OK"));
  Presente_datos(true);

  // Make a HTTP GET request:
  client.print(String("GET ")+resource +" HTTP/1.0\r\n");
  client.print(String("Host: ")+server+"\r\n");
  client.print("Connection: close\r\n\r\n");

  long timeout = millis();
  while (client.available()==0) {
    if (millis()-timeout>5000L) {
      msg=(F(">>> Tiempo cliente agotado !"));
      Presente_datos(true);
      client.stop();
      resumen=(F(">>> Tiempo cliente agotado !"));
      delay(10000L);
      return  0;        //Si se atasca
    }
  }

  msg=(F("Leyendo header respuesta... "));
  Presente_datos(false);

  uint32_t contentLength = knownFileSize;

  while (client.available()) {
    String line=client.readStringUntil('\n');
    line.trim();
    SerialMon.println(line);    // Uncomment this to show response header
    line.toLowerCase();
    if (line.startsWith("content-length:")) {
      contentLength=line.substring(line.lastIndexOf(':')+1).toInt();
      msg=String (contentLength);
      Presente_datos(true);
    } else if (line.length()==0) {
      resumen=(F(">>> Tiempo cliente agotado !"));
      break;
    }
  }

  msg=(F("Leyendo respuesta..."));
  Presente_datos(false);
  
  tft.drawRect(240,80,165,15,BLACK);              //Dibuja el rectangulo del porcentaje
  tft.setCursor(240,80);
  
  timeout = millis();
  uint32_t readLength=0;
  CRC32 crc;
  unsigned long timeElapsed=millis();
  printPercent(readLength, contentLength);
  while (readLength<contentLength&&client.connected()&&millis()-timeout<10000L) {
    while (client.available()) {
      uint32_t c=client.read();
      temp_file.write(c);
                                 // char si es texto
//      SerialMon.print((char)c);         // Uncomment this to show data
      crc.update(c);
      readLength++;
      if (readLength%(contentLength/13)==0) {
        printPercent(readLength, contentLength);
      }
      timeout=millis();
    }
  }
  temp_file.close();

  timeElapsed=millis()-timeElapsed;
  SerialMon.println();
  client.stop();
  msg=(F("Servidor desconectado"));
  Presente_datos(true);
  modem.gprsDisconnect();
  msg=(F("GPRS desconectado"));
  Presente_datos(true);
  float duration=float(timeElapsed)/1000;
  SerialMon.println();
  resumen="";
  msg=(F("Content-Length:"));Presente_datos(false);resumen+=msg;
  msg=(contentLength);Presente_datos(true);resumen+=msg;
  msg=(F(",Leido actual:"));Presente_datos(false);resumen+=msg;   
  msg=(readLength);Presente_datos(true);resumen+=msg;
  msg=(F(",Calc. CRC32:0x"));Presente_datos(false);resumen+=msg; 
  msg=(crc.finalize(),HEX);Presente_datos(true);resumen+=msg;
  msg=(F(",Conoc. CRC32:0x"));Presente_datos(false);resumen+=msg; 
  msg=(knownCRC32,HEX);Presente_datos(true);resumen+=msg;
  msg=(F(",Duracion:"));Presente_datos(false);resumen+=msg;   
  msg=(duration);Presente_datos(false);resumen+=msg; 
  msg=("s");Presente_datos(true);resumen+=msg;
  tamano=readLength;
  if(contentLength==readLength){
    intentos_dw=0;
    unsigned int archivo=EEPROM.get(700,archivo);
    archivo++;
    if(archivo>500){archivo=0;EEPROM.put(700,archivo);}
    return 1;
  }else{
    intentos_dw++;
    return 0;
  }
}

void Presente_datos(bool t){
  if(t){SerialMon.println(msg);tft.println(msg);}else{SerialMon.print(msg);tft.print(msg);}  
}

void printDirectory(File f, int numTabs) {
  char *nm=namebuf+pathlen;
  mensaje1="Archivos en SD\n";
  while (true) {
    File f=root.openNextFile();
    if (!f){
      msg=(F("No hay + archivos..."));
      mensaje+=msg+'('+String(cont_arch)+')'+'\n';
      Presente_datos(true);
      if(cont_arch>0&&h_sd){h_sd=true;}else{h_sd=false;}
      cont_arch=0;
      break;
    }
    for (uint8_t i=0;i<numTabs;i++){
      Serial.print('\t');
    }
    f.getName(nm,32-pathlen);
    strlwr(nm);
    if (strstr(nm,".bmp")!=NULL&&strstr(nm,NAMEMATCH)!=NULL){        //Esto solo deja imprimir los archivos con extension .bmp
      msg=nm;
      cont_arch++;
      Presente_datos(false);
      mensaje1+="("+String(cont_arch)+")";
      mensaje1+=msg+"...";
      msg=" ";
      Presente_datos(false);
      msg=String(f.size());
      Presente_datos(true);
      mensaje1+=msg+"\n";
      if(cont_arch==19||cont_arch==38){delay(5000L);tft.fillScreen(WHITE);tft.setCursor(0,0);}
      h_archivos=true;
      f.close();
    }
  }
}

void Wipe_arch(){
   if(!SD.wipe(&Serial)) {
     SD.errorHalt("Wipe failed.");
     return;
  }
}
