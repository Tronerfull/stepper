//pins Joystick
const int SW_pin = 11;
const int Y_pin = 0; 
//pins del motor
const int  motorPin1=8;
const int  motorPin2=9;
const int  motorPin3=7;
const int  motorPin4=6;
//pin foto (pin 10)
const int  fotoPin=10;
//librerias incluidas
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//parte de la versionn 4 con accelstepper
#include <AccelStepper.h>
//define halfstepp para precision
#define HALFSTEP 8


//VARIABLES DE CONTROL
//se mueve de manera perpetua hasta indicar que pare
int automatico=1;
//numero de vueltas hasta parar
int vueltas=0;
//direccion de la rotacion
String dir="DERECHA";
int dirval=1;
//velocidad de rotacion
int motorSpeed=50;
//segundos entre fotos
int SecEntreFoto=33;
//segundos entre paradas
int SecEntreStop=0;
//tiempo de paradas
int SecStop=0;

//BLUETOOTH
// datos recibidos por bluetooth ( variables empleadas)
int bluetooth = 0;
String readString;

//PANTALLA
//datos de la pantalla
int LCDRows=4;
int LCDColumns=20;
//libreria de la pantalla
LiquidCrystal_I2C lcd(0x27, LCDColumns, LCDRows);


//VARIABLES DEL MENU
//variable de la flecha (numero de char guardado en el lcd)
 const byte iARROW = 0; 
// Bits icono flecha 
uint8_t bARROW[]   = {  
    B00000, B00100, B00110, B11111,
    B00110, B00100, B00000, B00000
};
//inicia en la pagina de opciones y en la opcion 1
int page=1;
int menuitem=1;
//variable que declara si esta funcionando
int iniciado=1;
// Los textos del menu principal, la longitud maxima = columnsLCD-1
//, rellenar caracteres sobrantes con espacios.
const char *txMENU[] = { 
    "Iniciar            ",
    "Modo Automatico    ",                             
    "Vueltas            ",
    "Direccion          ",
    "Velocidad          ",
    "Frequencia Foto    ",
    "Frequencia Stop    ",
    "Duracion de Stop   "
    
};
//constante que da el numero de filas del menu (para el caso de necesitar moverse por el)
const byte iMENU = 8; 

//controla el estado (iniciado= 1 todo seleccionado, detenido= 2 seleccionando)RAW
int state=1;
int stateChanged=0;

//valor de la y del joystick
int y =0;

//valore de medicion de tiempos(actual,stop y foto)
unsigned long currentmillis=0;
unsigned long currentmillisStop=0;
unsigned long currentmillisFoto=0;
unsigned long previousStop=0;
unsigned long previousFoto=0;
//tiene en cuenta el numero de fotos por intervalo ya que cada fotos es un delay de 100
//si se desea retirar el sistema eliminar linea dentro de CONTROL DE FOTOS
int numeroFotosIntervalo=0;

// initialize the stepper library on pins 8,9 y 6,7:
AccelStepper stepper(HALFSTEP, motorPin1, motorPin4, motorPin2, motorPin3);

int stepCount = 0; 
//ESTADOS
//estados a considerar
//stop
boolean stopState=false;
//listo para tomar foto
boolean takeFoto=false;
//cambiodevuletas
boolean vueltasChanged=false;
//cambiodevel
boolean velChanged=false;
//cambiodedir
boolean dirChanged=false;


//SETUP
void setup() { 
  //MODO pin SW
  pinMode(SW_pin, INPUT_PULLUP);
  //Iniciar pin jackFotos
  pinMode(fotoPin, OUTPUT);
  //INICIA serial
  Serial.begin(9600);
  
 //LCD(PANTALLA
  // inicia
  lcd.init(); 
  // Print a message to the LCD.
  lcd.backlight();
  //crea el char de la flecha unido a la variable iArrow
  lcd.createChar(0,bARROW);
  // Imprime la informacion del proyecto Pantalla de entrada:
  lcd.setCursor(0,0); lcd.print("Sistema de rotacion ");
  lcd.setCursor(0,1); lcd.print("  Prototipo  ");
  delay(2000);
  lcd.clear();
  
  //Crea el contador de stops
  previousStop=millis();
  //selecciona la velocidad maxima del stepper (evitabloqueo en velocidad =1)
  stepper.setMaxSpeed(1000.0);
  
}

//LOOP
void loop() { 

  //se lee la y del joystick
  y=analogRead(Y_pin);
  stateChanged=0;
  //maneja el menu
  updateMenu();

}

//se encarga de la seleccion del menu(depende de la pagina seleccionada)
void updateMenu()
{
  
  switch (page)
  {
    case 0:
    //pantalla de inicio
      menuInicio();
      iniciado=1;
    break;
    case 1:
    //pantalla de opciones
    //encierra el proceso en un while hasta darle a salir
       menuOpciones();
       //si salimos de opciones volvemos al menu principal
       page=0;
    iniciado=0;
    break;
  }//switch
}

//menu con informacion inicial
void menuInicio()
{   
  //pantalla muestra los 3 parametros principales
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Parametros actuales");
    lcd.setCursor(0,1); lcd.print("Vueltas: ");
    //muestra las vueltas o  el texto "automatico"
    if(!automatico) 
    {
      lcd.setCursor(9,1);lcd.print(vueltas);
    }
    else
    {
    lcd.setCursor(9,1);lcd.print("AUTOMATICO");
    }
    lcd.setCursor(0,2);lcd.print("Velocidad: ");
    lcd.setCursor(11,2);lcd.print(motorSpeed);
    lcd.setCursor(0,3);lcd.print("Direccion: ");
    lcd.setCursor(11,3);lcd.print(getDireccion());

   previousStop=millis();
   previousFoto=millis();
   stopState=false;

   //controla 1 solo print cada vez que entremos en esta pagina
   boolean printOnce= true;
  while(page==0)
  {
    //actualiza vueltas
    if(vueltasChanged)
    {
      if(!automatico)
      {
       lcd.setCursor(9,1);lcd.print("          ");
       lcd.setCursor(9,1);lcd.print(vueltas);
       vueltasChanged=false;
       printOnce=true;
      }
     else if (automatico && printOnce)
      {
        lcd.setCursor(9,1);lcd.print("          ");
        lcd.setCursor(9,1);lcd.print("AUTOMATICO");
        printOnce=false;
      }
    }

    //actualiza vel
    if(velChanged)
    {
       lcd.setCursor(11,2);lcd.print("         ");
       lcd.setCursor(11,2);lcd.print(motorSpeed);
       velChanged=false;
    }

    //actualiza dir
    if(dirChanged)
    {
       lcd.setCursor(11,3);lcd.print("         ");
       lcd.setCursor(11,3);lcd.print(getDireccion());
       dirChanged=false;
    }
    
    //ejecutar movimiento
    iniciar();
    //comprobar bluetooth
    bluetoothControl();

    
    
    if(readButtonJoystick()==1)
    {
      page=1;
    }

    yield();
  }
}

//menu con las opciones a elegir
void menuOpciones()
{
    //indica el punto del menu en el que se encuentra el selector (->)
    byte idMenu = 0;
    //indica si se va a salir del menu
    boolean exitMenu   = false;
    //indica si sera necesario avanzar el menu(ir hacia abajo o arriba dentro del menu)
    boolean forcePrint = true;
    //limpiamos lo que hubiera en la pantalla
    lcd.clear();

//mientras no salgamos
    while( !exitMenu && page==1 )
    {
        
        int btnPressed = readButtonJoystick();
        int updownUsed = readYJoystick(); 
       // Serial.println(idMenu);
        bluetoothControl();
        
        if( updownUsed == 1 && idMenu-1 >= 0 )
        {
            idMenu--;
           
            delay(100);
        }
        else if( updownUsed == 2 && idMenu+1 < iMENU )
        {
            idMenu++;
            delay(100);
        }
        else if( btnPressed ==1)
        {
            switch( idMenu )
            {   
                case 0:  exitMenu = true; break; //Salir    
                case 1: modoAutomatico(); break;
                case 2: editVueltas(); break;
                case 3: setDireccion(); break;
                case 4: editVelocidad(); break;
                case 5: editSecEntreFoto(); break;
                case 6: editSecEntreStop(); break;
                case 7: editSecStop();break;
                
                                                  
            }
            forcePrint = true;
        }

        
        if( !exitMenu && (forcePrint || updownUsed!=0) )
        {
            forcePrint = false;
            //determina el tamaño
            static const byte endFor1 = (iMENU+LCDRows-1)/LCDRows;
            //determina el menu
            int graphMenu = 0;

            for( int i=1 ; i<=endFor1 ; i++ )
            {
                if( idMenu < i*LCDRows )
                {
                    graphMenu = (i-1) * LCDRows;
                    break;
                }
            }

            byte endFor2 = graphMenu+LCDRows;

            for( int i=graphMenu, j=0; i< endFor2 ; i++, j++ )
            {
                lcd.setCursor(1, j);
                lcd.print( (i<iMENU) ? txMENU[i] : "                    " );
            }

            for( int i=0 ; i<LCDRows ; i++ )
            {
                lcd.setCursor(0, i);
                lcd.print(" ");
            }
            lcd.setCursor(0, idMenu % LCDRows );
            lcd.write(iARROW);
        }
       yield();
    }

    lcd.clear();
}

//modo inicial del motor ( ejecutandose)
void iniciar()
{
  
  //iniciar contador de tiempo actual
    if(!stopState)
   currentmillis=millis();

 //direccion+ velocidad
   int vel= motorSpeed*dirval*10;

  //SISTEMA FINAL (movimiento segun atomatico y duracion total
if(automatico && !stopState)
{
   stepper.setSpeed(vel);
   stepper.runSpeed();
  // Serial.print(vel);
}
else
{
  if(vueltas>0 && !stopState)
  {
    stepper.setSpeed(vel);
    stepper.runSpeed();
    //Serial.print(vel);
  }
  else
  {
    stepper.setSpeed(0);
    stepper.runSpeed();
    //Serial.print(vel);
  }
}

//PROCESO DE CONTROL STOPS
if(SecEntreStop>0){
  //Tiempo entre intervalos en milisegundos
  unsigned long timeInMillis=SecEntreStop*1000;//iniciar contador duracion del stop

//Serial.println(currentmillis-previousStop);
if(!stopState)  
{
  //se toma en cuenta la ultima parada mas el tiempo de la parada
  if((currentmillis-(previousStop + SecStop*1000+numeroFotosIntervalo*100)>=(timeInMillis)) && (SecEntreStop>=0))
  {
  //variable controla el stop
  stopState=true;

    //cambia el counter de delays por foto a 0
  numeroFotosIntervalo=0;
  
  //millis va despues para no contar el tiempo de stop
  previousStop=millis();
  
  }
}

//contador de tiempo del stop
if(stopState)
   currentmillisStop=millis();
   
unsigned long timeInMillis2=SecStop*1000;

if(stopState)
{
  //Serial.println(currentmillisStop-previousStop);
//Duracion de los intervalos
 if((((currentmillisStop-(previousStop+numeroFotosIntervalo*100))>=(timeInMillis2)) && (SecStop!=0))||SecStop<=0)
 {
  //termina el stop
  stopState=false;
  //cambia el counter de delays por foto a 0
  numeroFotosIntervalo=0;
 }
}

}
//PROCESO CONTROL FOTOS
//Tiempo entre intervalos en milisegundos
  unsigned long AuxSecEntreFoto =SecEntreFoto;//es necesario pasarlo a unsigned long para la operacion
  unsigned long timeInMillisFoto=AuxSecEntreFoto*1000;//iniciar contador para la foto
  currentmillisFoto=millis(); 
  unsigned long AuxSecStop= SecStop;//mismo que con entre fotos
  unsigned long tempDelay=AuxSecStop*1000/2;

//ajuste de tiempo(cuenta la toma de la foto
  unsigned long auxAjuste=1000;

  if(AuxSecStop>0)
  {
    auxAjuste=1000*AuxSecStop;
  }

  //se toma en cuenta la ultima parada mas el tiempo de la parada
  if(((currentmillisFoto-previousFoto)>=(timeInMillisFoto+ auxAjuste)) && (AuxSecEntreFoto>0))
  {
    //variable controla el stop
    takeFoto=true;
   
 
    //millis va despues para no contar el tiempo de stop
    previousFoto=millis();
  
  }
  
  
  if(takeFoto)
  { 
    //Se puede comentar para eliminar el sistema de delays
    //numeroFotosIntervalo++;
  if(tempDelay>0)
  {
    takeFoto=false;
    digitalWrite(fotoPin,HIGH);
    delay(tempDelay);
    digitalWrite(fotoPin,LOW);
    delay(tempDelay);
  }
  else
  {
    takeFoto=false;
    digitalWrite(fotoPin,HIGH);
    delay(500);
    digitalWrite(fotoPin,LOW);
    delay(500);
  }
  }
  



//PROCESO CONTROL VUELTAS
//cada vuelta rebaja en 1 el contador
if((stepper.currentPosition()%4200)==0 && vueltas>0)
{
  vueltas--;
  vueltasChanged=true;
}



  
  //si el motor es automatico o si el numero de vueltas se completa
 /* if (motorSpeed > 0 && automatico==0 &&vueltas>0 ) 
  {*/
  //  steps=vueltas*stepsPerRevolution;
    //Serial.println(steps);
  

   

  /*  iniciado=1;*/
  //}
//  Serial.println(steps);
}

void bluetoothControl()
{
 //lee lo recibido por bluetooth
 while(Serial.available()>0)
  {
    char c= Serial.read();
    readString+=c;
   // Serial.println(readString);
  }
 

//cuando se ha recibido un mensaje
  if(readString.length()>0)
  {
    //caso 1: detener remotamente (se recibe una D)
    if(readString.charAt(0)=='D')
    {
      //comprueba
    
       page=1;
    }
    
    //Serial.println(readString);
    //readString.charAt(0)=='P'
    //caso 2: recibe parametros marcados con 'p' al principio
    if(readString.startsWith("P")&&readString.endsWith("P"))
    {
      //busca las comas separadoras
      int commaIndex = readString.indexOf(',');
       
      int secondCommaIndex = readString.indexOf(',', commaIndex + 1);

      int thirdCommaIndex = readString.indexOf(',', secondCommaIndex + 1);

      int fourthCommaIndex = readString.indexOf(',', thirdCommaIndex + 1);

      int fifthCommaIndex = readString.indexOf(',', fourthCommaIndex + 1);

      int sixthCommaIndex = readString.indexOf(',', fifthCommaIndex + 1);

      int delimiter= readString.indexOf('P',sixthCommaIndex+1);
      
 //extrae los valores en forma de texto
     String firstValue = readString.substring(1, commaIndex);
     //necesita añadirse doble P al mensaje por si ocurre algun delay a la hora de recibirlo
      if(readString.startsWith("PP"))
      {
         firstValue = readString.substring(2, commaIndex);
      }
      String secondValue = readString.substring(commaIndex + 1);
      String thirdValue = readString.substring(secondCommaIndex + 1,thirdCommaIndex);
      String fourthValue = readString.substring(thirdCommaIndex + 1, fourthCommaIndex);
      String fifthValue = readString.substring(fourthCommaIndex+1, fifthCommaIndex);
      String sixthValue = readString.substring(fifthCommaIndex+1, sixthCommaIndex);
      String seventhValue = readString.substring(sixthCommaIndex+1, delimiter);

     //Convierte valores de string a int
     //velocidad(1)
      motorSpeed = firstValue.toInt();
     //vueltas(2)
      vueltas = secondValue.toInt();
    //segundos entre fotos(3)
      SecEntreFoto = thirdValue.toInt();
    //tiempo de paradas(4)
      SecStop= fourthValue.toInt();
    //segundos entre paradas(5)
      SecEntreStop= fifthValue.toInt();
    //modo automatico(6)
      automatico = sixthValue.toInt();
    //direccion(7)
      dirval = seventhValue.toInt();

    //Serial.println(SecEntreFoto);
      //se anuncia a la pantalla el cambio de parametros
      dirChanged=true;
      vueltasChanged=true;
      velChanged=true;

      
    //se activa el motor, se elimina el stop anterior
     page=0;
   previousStop=millis();
   previousFoto=millis();
  //si esta en un stop se le saca de el
     stopState=false;
      
   }

    

  //limpia readstring 
  //Serial.println(readString);
  bluetooth=readString.toInt();
  readString="";
 
  //limpia el canal
  Serial.flush();
  }
}
//edicion de la velocidad
void editVelocidad()
{
   boolean exitSubMenu = false;
   boolean forcePrint  = true;
   lcd.clear();
   int memovel=motorSpeed;
  //menu edicion de direccion
 lcd.setCursor(0,0);lcd.print("Velocidad rotacion");
 lcd.setCursor(0,1);lcd.print(motorSpeed);    
 lcd.setCursor(0,3);lcd.print("Pulse para regresar");
   while( !exitSubMenu )
   {
    //actualiza Y
    int d=readYJoystick();
    if(d==1 &&motorSpeed>0)
  {
    motorSpeed= motorSpeed-10;
    delay(100);
  }
  if (d==2 && motorSpeed*10<1000)
  {
     motorSpeed= motorSpeed+10;
     delay(100);
  }
    //se sale del menu al pulsar el boton
    if(readButtonJoystick())
    {
      exitSubMenu=true;
    }

     
  
     //actualiza si el numero cambio
     if(memovel!=motorSpeed ||motorSpeed==0)
     {
     lcd.setCursor(0,1);lcd.print("                   ");
     lcd.setCursor(0,1);lcd.print(motorSpeed);    
     delay(100);   
      velChanged=true;
     }
     memovel=motorSpeed;
    
     yield();
    }

    lcd.clear();
}

//edicion de los segundos entre fotos
void editSecEntreFoto()
{
   boolean exitSubMenu = false;
   boolean forcePrint  = true;
   lcd.clear();
   int memoefoto=SecEntreFoto;
     //menu edicion de direccion
     lcd.setCursor(0,0);lcd.print("Frequencia foto(sec)");
     lcd.setCursor(0,1);lcd.print(SecEntreFoto);  
     lcd.setCursor(0,3);lcd.print("Pulse para regresar");
     
   while( !exitSubMenu )
   {
    //actualiza Y
    int d=readYJoystick();
   if(d==1 &&SecEntreFoto>0)
  {
    SecEntreFoto= SecEntreFoto-1;
    delay(100);
  }
  if (d==2 && SecEntreFoto<=999)
  {
     SecEntreFoto= SecEntreFoto+1;
     delay(100);
  }
    //se sale del menu al pulsar el boton
    if(readButtonJoystick())
    {
      exitSubMenu=true;
    }

     
    
     //actualiza si el numero cambio
     if(memoefoto!=SecEntreFoto)
     {
     lcd.setCursor(0,1);lcd.print("                   ");
     lcd.setCursor(0,1);lcd.print(SecEntreFoto);    
     delay(100);   
     }
     memoefoto=SecEntreFoto;
    
     yield();
    }

    lcd.clear();
 
}

void editSecEntreStop()
{
    boolean exitSubMenu = false;
   boolean forcePrint  = true;
   lcd.clear();
   int memostop=SecEntreStop;
     //menu edicion de direccion
     lcd.setCursor(0,0);lcd.print("Frequencia Stop(sec)");
     lcd.setCursor(0,1);lcd.print(SecEntreStop);
     lcd.setCursor(0,3);lcd.print("Pulse para regresar"); 
     
   while( !exitSubMenu )
   {
    //actualiza Y
    int d=readYJoystick();
  if(d==1 &&SecEntreStop>0)
  {
    SecEntreStop= SecEntreStop-1;
    delay(200);
  }
  if (d==2 && SecEntreStop<=999)
  {
     SecEntreStop= SecEntreStop+1;
     delay(200);
  }
    //se sale del menu al pulsar el boton
    if(readButtonJoystick())
    {
      exitSubMenu=true;
    }

     //actualiza si el numero cambio
     if(memostop!=SecStop)
     {
     lcd.setCursor(0,1);lcd.print("                   ");
     lcd.setCursor(0,1);lcd.print(SecEntreStop);    
     delay(200);   
     }
     
     memostop=SecEntreStop;
     
     yield();
    }

    lcd.clear();

}

void editSecStop()
{
   boolean exitSubMenu = false;
   boolean forcePrint  = true;
   lcd.clear();
   int memostop=SecStop;
    //menu edicion de direccion
     lcd.setCursor(0,0);lcd.print("Duracion Stop(sec.)");
     lcd.setCursor(0,1);lcd.print(SecStop); 
     lcd.setCursor(0,3);lcd.print("Pulse para regresar");  
   
   while( !exitSubMenu )
   {
    //actualiza Y
    int d=readYJoystick();
    
      if(d==1 &&SecStop>=0)
      {
         SecStop= SecStop-1;
         delay(200);
      }
     if (d==2 && SecStop<=999)
     {
         SecStop= SecStop+1;
          delay(200);
     }
    //se sale del menu al pulsar el boton
    if(readButtonJoystick())
    {
      exitSubMenu=true;
    }

     //actualiza si el numero cambio
     if(memostop!=SecStop)
     {
     lcd.setCursor(0,1);lcd.print("                   ");
     lcd.setCursor(0,1);lcd.print(SecStop);    
     delay(200);   
     }
     memostop=SecStop;
  
     yield();
    }

    lcd.clear();
}

//seleccion de vueltas
void editVueltas()
{
   boolean exitSubMenu = false;
   boolean forcePrint  = true;
   lcd.clear();
   int memovueltas=vueltas;
     //menu edicion de direccion
     lcd.setCursor(0,0);lcd.print("Numero de vueltas");
     lcd.setCursor(0,1);lcd.print(vueltas);  
     lcd.setCursor(0,3);lcd.print("Pulse para regresar");
   while( !exitSubMenu )
   {
    //actualiza Y
    int d=readYJoystick();
    
     
    if(d==1 &&vueltas>0)
    {
      vueltas= vueltas-1;
      delay(100);
    }
    if (d==2 && vueltas<=99)
    {
     vueltas= vueltas+1;
     delay(100);
   }
    //se sale del menu al pulsar el boton
    if(readButtonJoystick())
    {
      exitSubMenu=true;
    }

     
    
     //actualiza si el numero cambio
     if(memovueltas!=vueltas)
     {
     lcd.setCursor(0,1);lcd.print("                   ");
     lcd.setCursor(0,1);lcd.print(vueltas);    
     delay(100);
      vueltasChanged=true;   
     }
     memovueltas=vueltas;
    
     yield();
    }

    lcd.clear();
}

//activar o desactivar modo automatico
void modoAutomatico()
{
   boolean exitSubMenu = false;
   boolean forcePrint  = true;
   lcd.clear();
   
   while( !exitSubMenu )
   {
    //actualiza Y
    int d=readYJoystick();
    
    if(d==1 & automatico)
    {
      automatico= 0;
      delay(100);
    }
    if (d==2 & !automatico)
    {
     automatico= 1;
     delay(100);
   }
    //se sale del menu al pulsar el boton
    if(readButtonJoystick())
    {
      exitSubMenu=true;
    }

     
      //menu edicion de direccion
     lcd.setCursor(0,0);lcd.print("Modo automatico");
     
     //actualiza si el esta activado o no
     if(automatico)
     {
      lcd.setCursor(0,1);lcd.print("Activado           "); 
     }
     else
     {
      lcd.setCursor(0,1);lcd.print("Desactivado        "); 
     }
      
     lcd.setCursor(0,3);lcd.print("Pulse para regresar");
     
     yield();
    }

    lcd.clear();
}


//Modifica si el motor va a derecha o izquierda dependiendo de dirval
void setDireccion()
{
  String memodir=getDireccion();
   boolean exitSubMenu = false;
   boolean forcePrint  = true;
   lcd.clear();
   
   while( !exitSubMenu )
   {
    //actualiza Y
    int d=readYJoystick();
     if(d==1 && dirval>0)
     {
        dirval= dirval*(-1);
        delay(100);
     }
     if (d==2 && dirval<0)
     {
        dirval= dirval*(-1);
        delay(100);
     }
    //se sale del menu al pulsar el boton
    if(readButtonJoystick())
    {
      exitSubMenu=true;
    }

     
      //menu edicion de direccion
     lcd.setCursor(0,0);lcd.print("Direccion Elegida");

     lcd.setCursor(0,1);lcd.print("<");  
     lcd.setCursor(1,1);lcd.print(getDireccion());
     lcd.setCursor(LCDColumns-1,1);
     lcd.print(">");  
     lcd.setCursor(0,3);lcd.print("Pulse para regresar"); 
     delay(100);   

     if(memodir!=getDireccion())
     {
      dirChanged=true; 
     }
     
    }

    lcd.clear();
 
}
//Indica si el motor va a derecha o izquierda dependiendo de las revolutionsteps
String getDireccion()
{
   if(dirval>0)
   {
      dir="DERECHA  ";
   }

    if(dirval<0)
   {
      dir="IZQUIERDA";
   }
  
  return dir;
}


//lee el joystick y devuelve 1 si el estado ha cambiado(si se ha pulsado)
int readButtonJoystick()
{  
  if((!digitalRead(SW_pin))&& state==1)
  {
    delay(200);
    state=2;
    stateChanged=1;

  }
  else if((!digitalRead(SW_pin))&& state==2)
  {
    delay(200);
    state=1;
    stateChanged=1;
 
  }
  else
  {
    stateChanged=0;
  }
  return stateChanged;
}
//lee el joystick y devuelve 2(derech) o 1(izq) dependiendo de la direccion de y a la que se desplace
int readYJoystick()
{  
  int stateDir=0;
  y=analogRead(Y_pin);
 // Serial.println(y);
 
   if(y>=580)
  {
    stateDir=1;
    delay(200);
  }
  if (y<=460)
  {
     stateDir=2;
     delay(200);
  }
 
  return stateDir;
}

