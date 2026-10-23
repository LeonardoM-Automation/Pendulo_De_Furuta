#include<Wire.h>
#include<L298N.h>




//Definições________________________________________________________
constexpr int8_t ENA = 18;  //Controla velocidade motor via PWM
constexpr int8_t IN1 = 19;    //Controla direção do motor
constexpr int8_t IN2 = 23;    //Controla direção do motor
constexpr float Pi = 3.14159f;
constexpr int16_t frequencia = 5000; //frequencia do PWM
constexpr int8_t resolucao = 8; // PWM vai de 0 a 255
constexpr int8_t AS5600_ENDERECO = 0x36;//Endereço padrão do Sensor
constexpr int REG_HIGH = 0x0E;//Endereço do Reg com bit mais signif
constexpr int REG_LOW = 0x0F;//Endereço do Reg com bit menos signif
constexpr float_t KP=40; // Ganho Proporcional
constexpr float_t KI=0; // Ganho Integrador
constexpr float_t KD=0; // Ganho Derivativo
constexpr float_t anguloMaxHorario = -2.10f; // Ang max quando dir = 1
constexpr float_t anguloMaxAntiHorario = 2.4f;// Ang max quando dir = 2;
constexpr float_t setPoint = 0; //angulo em que o pendulo deve ficar equilibrado
constexpr float_t k1 = 3.0f;
constexpr float_t k2 = 4.4f;
constexpr float_t k3 = 88.60f;
constexpr float_t k4 = 15.0f;
//constexpr float_t C1 = 0; // constante  dada por (Resistencia de Armadura(Ra)/Constante do torque(kt))que multiplica o torque
//constexpr float_t C2 = 1; // constante dada por [(constanteElétrica(kv)/constante do torque(kt)] que multiplica a velocidade angular do motor. acho que são iguais, então C2=1
constexpr float_t V2Pgain = 30.0f; //valor que converte 0V-5V em 0 - 255
constexpr float_t margemEquilibrio = 0.5;
constexpr float_t filtro_alfa = 0.20f;//Constante do Filtro Passa-Baixas das velocidades
constexpr float_t TS = 10;
constexpr float_t dt = TS/1000;
constexpr float_t margemSwingUp = 0.5f;




//Declaração de variáveis__________________________________________
int16_t PWM;// guarda o pwm que será enviado a ponteH
float_t anguloHaste;//Guarda ângulo da haste
float_t anguloPendulo;//Guarda ângulo do pêndulo
float_t anguloHasteAnterior;//Guarda ângulo da haste
float_t anguloPenduloAnterior;//Guarda ângulo do pêndulo
float_t vHaste;//Guarda velocidade filtrada da haste
float_t vPendulo;//Guarda velocidade filtrada do pêndulo
float_t vHasteBruta;
float_t vPenduloBruta;
float_t erro;
float_t ultimoErro;
float_t integrador;
float_t derivativo;
float_t torque;




float_t tempo;
unsigned long ultimoTempo;
TwoWire sensorHaste = TwoWire(0);
TwoWire sensorPendulo = TwoWire(1);








//Funções para ler dados dos Sensores ______________________________
float lerAngulo(TwoWire& sensor)
{
  sensor.beginTransmission(AS5600_ENDERECO);
  sensor.write(REG_HIGH);

  if(sensor.endTransmission() != 0)
  {
    Serial.println("Erro de comunicacao com AS5600");
  return 10.0f;
  }

  sensor.requestFrom(AS5600_ENDERECO,2);
  if(sensor.available()>=2)
  {
    byte highByte=sensor.read();
    byte lowByte=sensor.read();
    int angulo = ((highByte & 0x0F) << 8)| lowByte;
    return ((angulo*2*Pi)/4096.0f);
  }
  return 10.0f;
}














//CALCULAR E FILTRAR VELOCIDADES_______________________________________________________
inline void calcularVelocidades()
{

  vHasteBruta = (anguloHaste - anguloHasteAnterior)/dt;
  vPenduloBruta = (anguloPendulo - anguloPenduloAnterior)/dt;


  anguloHasteAnterior = anguloHaste;
  anguloPenduloAnterior = anguloPendulo;


  vHaste = (filtro_alfa*vHasteBruta)+((1.0f - filtro_alfa)*vHaste);
  vPendulo = (filtro_alfa*vPenduloBruta)+((1.0f - filtro_alfa)*vPendulo);
   
}










//ATUAÇÃO DO MOTOR______________________________________________
inline void atuarMotor()
{
  //define a direção
  if(PWM<0)
  {
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
  }else if(PWM > 0)
  {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
  }else
  {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
  }
 
  ledcWrite(ENA,abs(PWM)); // aplica o PWM na PonteH
}












//Função de segurança_______________________________________________
inline void seguranca()
{
  if(PWM<0 && anguloHaste <= (anguloMaxHorario))
  {
    PWM = 0;
  }else if(PWM>0 && anguloHaste >= (anguloMaxAntiHorario))
  {
    PWM = 0;
  }
  if(anguloHaste==6.96f)
  {
    PWM = 0;
  }
}





/*
inline void PenduloToCentro()
{
  if(anguloHaste>0)
  {
    do
    {
      PWM = -180;
      atuarMotor();
    }while(anguloHaste>0);
    return;
  }
  if(anguloHaste<0)
  {
    do
    {
      PWM = 180;
      atuarMotor();
    }while(anguloHaste<0);
    return;
  }
}




//Função de Controle PID____________________________________________
inline void PID()
{
    erro = anguloPendulo;
    integrador += erro * dt;
    derivativo = ((erro-ultimoErro)/dt);
    ultimoErro = erro;
    PWM = (int)((KP*erro + KI*integrador + KD*derivativo)*51);
}
*/









//Função SwingUP______________________________________________________
inline void swingUP()
{
 	//Implementar Lógica Swing Up
}








//Função LQR__________________________________________________________
inline void LQR()
{
      calcularVelocidades();
      torque=(k1*anguloHaste + k2*vHaste + k3*anguloPendulo + k4*vPendulo);
      PWM = torque*V2Pgain;
      //PWM = (torque*C1-vHaste*C2)*V2Pgain;
   
}








//SETUP_________________________________________________________
void setup()
{
  //Inicializando comunicação Serial
  Serial.begin(115200);
  //Inicialização de pinos
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  ledcAttach(ENA,frequencia,resolucao);// Habilita o pino ENA como pino PWM e configura o PWM
  //Inicialização AS5600
  sensorHaste.begin(21,22);
  sensorPendulo.begin(25,26);
  PWM = 0;
  ultimoTempo = 0;
  ultimoErro = 0;
  integrador = 0;
  atuarMotor();
  anguloHaste = lerAngulo(sensorHaste)-3.04;
  anguloPendulo = lerAngulo(sensorPendulo)-3.32f;
  anguloHasteAnterior = anguloHaste;
  anguloPenduloAnterior = anguloPendulo;
  vHaste = 0;
  vPendulo = 0;
}






//LOOP__________________________________________________________
void loop()
{
 
  anguloHaste = lerAngulo(sensorHaste)-3.04;
  anguloPendulo = lerAngulo(sensorPendulo)-3.32f;
 
  //if(anguloPendulo<margemEquilibrio && anguloPendulo>-margemEquilibrio)
  //{
    tempo = millis()-ultimoTempo;
    if(tempo>=TS)
    {
      LQR();
      ultimoTempo+=TS;
    }
    seguranca();
    atuarMotor();
  //}else
  /*{
    //PWM = 180;
    tempo = millis()-ultimoTempo;
    if(tempo>=TS)
    {
      swingUP();
      ultimoTempo+=TS;
    }
   
    seguranca();
    atuarMotor();
  }*/
  Serial.print("anguloH:");
  Serial.print(anguloHaste);
  Serial.print(" anguloP:");
  Serial.print(anguloPendulo);
  Serial.print(" PWM:");
  Serial.print(PWM);
  Serial.print(" erro:");
  Serial.print(vPendulo);
  Serial.println("");
}
