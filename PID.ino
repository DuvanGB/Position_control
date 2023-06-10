#include <PID_v1.h>    // Librería PID de Brett Beauregard: https://playground.arduino.cc/Code/PIDLibrary

// Velocidad
byte Encoder_C1Last;
boolean Direccion;
double Sensi = 0.0;
// ********************************************** I/O *****************************************************************************
const byte    encA = 19;              // Entrada de la señal A del encoder.
const byte    encB = 18;              // Entrada de la señal B del encoder.
const byte    PWMA = 6;              // Salida PWM al primer pin.
const byte    PWMB = 7;              // Salida PWM al segundo pin.
// ************************************************ Variables PID *****************************************************************
double        Setpoint = 0.0, Input = 0.0, Output = 0.0;  // Setpoint = Posición designada; Input = Posición del motor; Output = Tensión de salida para el motor.
double        kp = 0.0, ki = 0.0, kd = 0.0;               // Constante proporcional, integral y derivativa.
double        outMax = 0.0, outMin = 0.0;                 // Límites para no sobrepasar la resolución del PWM.
double        Grados = 0.0, Respuesta = 0.0;
// **************************************************** Otras Variables ***********************************************************
volatile long contador = 0;           // Guarda los pulsos del encoder.
byte          ant = 0, act = 0;       // Sólo se utiliza los dos primeros bits de estas variables y servirán para decodificar el encoder (ant = anterior, act = actual).
byte          cmd = 0;                // Byte para la comunicación serie (cmd = comando).
unsigned int  tmp = 0;                // Tiempo de muestreo.
const byte    ledok = 13;             // Led para mostrar que el motor ya ha llegado a la posición designada.
// ********************************************************************************************************************************

PID myPID(&Input, &Output, &Setpoint, 0.0, 0.0, 0.0, DIRECT); // Parámetros y configuración para invocar la librería.

void setup()                         
{
  Serial.begin(2000000);               
  pinMode(PWMA, OUTPUT);              // Declara las dos salidas PWM para el control del motor (pin 5).
  pinMode(PWMB, OUTPUT);              //                                                       (pin 6).
  digitalWrite(PWMA, LOW);            // Y ambas salidas las inicializa a cero.
  digitalWrite(PWMB, LOW);
  
  TCCR0B = TCCR0B & B11111000 | 1;   // Configuración de la frecuencia del PWM para los pines 5 y 6.
                                     // Podemos variar la frecuencia del PWM con un número de 1 (32KHz) hasta 7 (32Hz). El número que pongamos es un divisor de frecuencia. Min.=7, Max.=1.

  attachInterrupt(digitalPinToInterrupt(encA), encoder, CHANGE); // En cualquier flanco ascendente o descendente
  
  outMax =  255.0;                    // Límite máximo del PWM.
  outMin = -outMax;                   // Límite mínimo del PWM.
  
  tmp = 1;                           // Tiempo de muestreo en milisegundos.
  
  kp = 2.1;//2.701;                          // Constantes PID iniciales. 
  ki = 3.2;//9.324;   // m2 = 2.0                       
  kd = 0.1956;                                
  
  myPID.SetSampleTime(tmp);             // Envía a la librería el tiempo de muestreo.
  myPID.SetOutputLimits(outMin, outMax);// Límites máximo y mínimo; corresponde a Max.: 0=0V hasta 255=5V (PWMA), y Min.: 0=0V hasta -255=5V (PWMB). Ambos PWM se convertirán a la salida en valores absolutos, nunca negativos.
  myPID.SetTunings(kp, ki, kd);         // Constantes de sintonización.
  myPID.SetMode(AUTOMATIC);             // Habilita el control PID (por defecto).
  Setpoint = (double)contador;          // Para evitar que haga cosas raras al inciarse, igualamos los dos valores para que comience estando el motor parado.
  
  imprimir(3);                        // Muestra los datos de sintonización y el tiempo de muestreo por el terminal serie.
}

void loop()
{
  
//El valor de 2.291666667 sale entre la razón de 825 pulsos y 360 grados.    0.29166666666666666666666666666667
 
 Setpoint = Grados*Sensi; //Es para ingresar valores de 0 a 360 grados como setpoint.
 Respuesta = contador/Sensi; //En este caso el contador que seria la respuestas en forma de pulsos.
 
  Input = (double)contador;           // Lectura del encoder óptico. El valor del contador se incrementa/decrementa a través de las interrupciones externas (pines 2 y 3).
  
  while(!myPID.Compute());            // Mientras no se cumpla el tiempo de muestreo, se queda en este bucle.

  // *********************************************** Control del Motor *************************************************
  if (((long)Setpoint - contador) == 0)// Cuando está en el punto designado, parar el motor.
  {
    digitalWrite(PWMA, LOW);          // Pone a 0 los dos pines del puente en H.
    digitalWrite(PWMB, LOW);
    digitalWrite(ledok, HIGH);        // Se enciende el led (pin 13) para avisar visualmente que está en la posición designada.
  }
  else                                // En caso contrario hay que mirar si el motor va hacia delante o hacia atrás, con el signo de la variable Output.
  {
    if (Output >0.0)                 // Mueve el motor hacia delante con el PWM correspondiente a su posición.
    {
      digitalWrite(PWMB, LOW);        // Pone a 0 el segundo pin del puente en H.
      analogWrite(PWMA, abs(Output)); // Por el primer pin sale la señal PWM.
    }
    else                              // Mueve el motor hacia  atrás   con el PWM correspondiente a su posición.
    {
      digitalWrite(PWMA, LOW);        // Pone a 0 el primer pin del puente en H.
      analogWrite(PWMB, abs(Output)); // Por el segundo pin sale la señal PWM.
    }
  }
  
  // Recepción de datos para posicionar el motor, modificar las constantes PID o el tiempo de muestreo.
  if (Serial.available() > 0)           // Comprueba si ha recibido algún dato por el terminal serie.
  {
    cmd = 0;                            // Por seguridad se limpia cmd.
    cmd = Serial.read();                // cmd guarda el byte recibido.
    if (cmd > 31)
    {
      byte flags = 0;                                     // Borramos la bandera que decide lo que hay que imprimir.
      if (cmd >  'Z') cmd -= 32;                          // Si una letra entra en minúscula la covierte en mayúscula.
      
      // Decodificador para modificar las constantes PID.
      switch(cmd)                                                                            // p2.5 i0.5 d40 carga valores en kp, ki y kd.
      {                                                                                      // Tambien se puede uno por uno (Chicaneo :v).
        case 'P': kp  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); flags = 1; break; // Carga las constantes y muestra los valores de las variables que hayan sido modificadas.
        case 'I': ki  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); flags = 1; break;
        case 'D': kd  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); flags = 1; break;
        case 'T': tmp = Serial.parseInt();   myPID.SetSampleTime(tmp);     flags = 1; break;
        case 'G': Grados = Serial.parseFloat();                            flags = 2; Sensi = 0.4321; break;  // Posicion en grados g180, se posiciona en 180.
        case 'K':                                                          flags = 3; break;  // K para ver los parametros anteriores del PID.
      }
      digitalWrite(ledok, LOW);       // Cuando entra una posición nueva se apaga el led y no se volverá a encender hasta que el motor llegue a la posición dada.
      
      imprimir(flags); 
    }
  }
Serial.print(Grados);
Serial.print(" ");
Serial.println(contador);

}

void encoder()                        
{
  int Lstate = digitalRead(encA);
  if ((Encoder_C1Last == LOW) && Lstate == HIGH)
  {
    int val = digitalRead(encB);
    if (val == LOW && Direccion)
    {
      Direccion = false; //Reverse
    }
    else if (val == HIGH && !Direccion)
    {
      Direccion = true;  //Forward
    }
  }
  Encoder_C1Last = Lstate;
  if (!Direccion)  contador++;
  else  contador--;
  ant=act;                          // Guardamos el valor 'act' en 'ant' para convertirlo en pasado.
  act=PIND & 12;                    // Guardamos en 'act' el valor que hay en ese instante en el encoder.
}

void imprimir(byte flag) // Imprime en el terminal serie los datos de las contantes PID, tiempo de muestreo y posición. En los demás casos sólo imprime la posición del motor.
{
  if ((flag == 1) || (flag == 3))
  {
    Serial.print("KP=");     Serial.print(kp);
    Serial.print(" KI=");    Serial.print(ki);
    Serial.print(" KD=");    Serial.print(kd);
    Serial.print(" Time=");  Serial.println(tmp);
  }
}
