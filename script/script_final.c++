Código para fazer leitura:

#include "TimerOne.h"

#define runEvery(t) for (static uint16_t _lasttime; (uint16_t)((uint16_t)millis() - _lasttime) >= (t); _lasttime += (t))
#define runEveryus(t) for (static uint16_t _lasttime; (uint16_t)((uint16_t)micros() - _lasttime) >= (t); _lasttime += (t))

#define PINOA_ENCODER 3
#define PINO1_PWM 9
#define PINO2_PWM 10

const unsigned Ts = 5000; // Período de amostragem;
const unsigned PWM_FREQ = 10; // Frequêna do PWM [kHz];
const double VFONTE = 12.0; // Tensão máxima da fonte (saturação);

volatile long _pulsosencoder;

// Variaveis para geração do sinal de entrada
double r=0, s, S, P=1;
bool Q = 0, flag, MA = 0;

// Variaveis para calculo da velocidade:
double w;                     // velocidade angular;
double theta_ant;             // Para calculo da velocidade;
unsigned long tempo, deltat; // Para calculo da velocidade;

// Variaveis para o PRBS:
uint16_t a = 0x01; // Para criar sinal PRBS;
unsigned short n = 1, m = 10;

// Variaveis utilizadas na leitura do encoder:
volatile byte abOld;           // Old state
volatile byte changedCount;    // Change-indication
const unsigned NENC = 2;       // Número de encoders;
volatile long count[NENC];     // current rotary counT
 //const unsigned NENC = 1;       // Número de encoders;
const unsigned MASCARA = 0x3c; // Máscara para tratamento dos encoders;
const double ENC_RES = 32.0;   // Resolução do encoder (pulsos por revolução - PPR);
int u=0;
void setup() {
  Serial.begin(115200);                   // Inicializa comunicacao serial
  pinMode(PINO1_PWM, OUTPUT);              // Configura pino do PWM como de saida
  pinMode(PINO2_PWM, OUTPUT);              // Configura pino do PWM como de saida
  // attachInterrupt( digitalPinToInterrupt( PINOA_ENCODER ), ContaPulsos, CHANGE );
  setaEncoders(INPUT_PULLUP, true); 
  Timer1.initialize(1000 / PWM_FREQ);    // Configura timer do PWM (40 us = 25 kHz => 42,75 us = 23,4 kHz)
  Timer1.pwm(PINO1_PWM, 0);
  Timer1.pwm(PINO2_PWM, 0);
}

void loop()
{
  runEvery(50){leSerial();}

  runEveryus(Ts){

	// Cria PRBS: ---
  if(P){
	    if (n < m) n++;
	    else {
	      a = geraPRBS11 (a);
	      n = 1;
	    }
	    u = (a & 1) ? 8 : -8;
  }

	// Fim do PRBS  ---

	// Cria sina tipo "escada": -------
	    //  runEvery(500){
	    //    if(u<11)
	    //      u++;
	    //    else
	    //      u = 0;
	    //  }
	// Fim do sinal tipo "escada". ---

//--- Cria sinal tipo onda quadrada ---
if(Q){
 runEvery(200){
       if(u!=s)
         u = s;
       else
         u = -s;
     }
}
    
    // Fim - sinal onda quadrada. ---

    w = leang();
    acionaMotor(u);
    enviaSerial();
  }
}

uint16_t geraPRBS7 (uint16_t a)
  // Funcao para gerar PRBS de 7 bits
{
  uint16_t newbit = (((a >> 6) ^ (a >> 5)) & 1);
  return a = ((a << 1) | newbit) & 0x7f;
}

uint16_t geraPRBS9 (uint16_t a)
  // Funcao para gerar PRBS de 9 bits
{
  uint16_t newbit = (((a >> 8) ^ (a >> 4)) & 1);
  return a = ((a << 1) | newbit) & 0x1ff;
}

uint16_t geraPRBS11(uint16_t a)
  // Funcao para gerar PRBS de 11 bits
{
  uint16_t newbit = (((a >> 10) ^ (a >> 8)) & 1);
  return a = ((a << 1) | newbit) & 0x07ff;
}

long int lenumero(char caracter)
{
  /*
    Função resonsável por interpredar o que é dígito enviado pela comunicação serial ao arduino e concatená-los
    maneira a formar um número inteiro.
  */
  long int valor = 0;
  int sinal = 1;
  while (caracter != 10)                   // verifica se eh um digito entre 0 e 9
  {
    caracter = Serial.read();
  if (caracter >= '0' && caracter <= '9')  // verifica se eh um digito entre 0 e 9
  valor = (valor * 10) + (caracter - '0'); // se sim, acumula valor
    else if ( caracter == '-')
      sinal = -1;
  else                                     // em caso de nao ser um numero ou simbolo de menos termina o valor
    {
  valor = valor * sinal ;                  // seta a variavel valor com o valor acumulado
      return (valor);
      // valor = 0;                        // reseta valor para 0 para a provima sequencia de digitos
      sinal = 1;
    }
  }
}

long map2(double v, double in_min, double in_mav, long out_min, long out_mav)
  /*
    Função resonsável por mapear uma faixa de valores de um sinal de entrada do
    tipo 'double' para outra faixa de valores de sinal do tipo 'long'.
  */
{
  return (v - in_min) * (out_mav - out_min) / (in_mav - in_min) + out_min;
}

// double map3( int v, int in_min, int in_mav, double out_min, double out_mav ) {
  /*
    Função resonsável por mapear uma faixa de valores de um sinal de entrada do
    tipo 'int' para outra faixa de valores de sinal do tipo 'double'.
  */
//   return ( v - in_min ) * ( out_mav - out_min ) / ( in_mav - in_min ) + out_min;
// }

void enviaSerial() {
  /*
    Função para eviar valores via comunicação serial usando codificação do tipo
    ASCII.
  */
  Serial.print(count[0]);
  Serial.print(',');
  Serial.print(count[1]);
  Serial.print(',');
  Serial.print(w);
  Serial.print(',');
  Serial.print(u);
  Serial.println();
  
}


void leSerial()
  {
    // LEITURA DOS DADOS SERIAIS
    if ( Serial.available() > 0)
    {
      char ch = Serial.read();
      switch (ch) {
        case 'r':
          r = lenumero(ch); // le o valor de referencia recebido
          Q = 0;
          break;
        case 'v':
          r = lenumero(ch);
          break;
        case 's':
          s = lenumero(ch); 
          Q = 1;
          P = false;
          break;
        case 'S':
          S = lenumero(ch); 
          Q = 1;
          P = false;
          break;
        case 'm':
          m = lenumero(ch); 
          break;
        case 'a':
          MA = true;
          break;
        case 'f':
          MA = false;
          break;
        case 'p':
          P = true;
          Q = false;
          break;
      } // Fim do switch
    } // Fim da leitura serial
}

double leVeloc() {
  // Calcula a velocidade
  double theta     = double(count[0]) / ENC_RES / 4.0 * 2.0 * PI;
  deltat    = micros() - tempo;                // Calcula intervalo entre amostras;
  tempo     = micros();
  double omega     = (theta - theta_ant) / double(deltat) * 1.0e6; // Calcula velocidade;
  theta_ant = theta;
  return omega;
}

double leang () {
  // Calcula o angulo
  double theta = double(count[1]) *0.15;
  if (theta > 360 ) {
    theta = theta -360;
  } else {
      double theta = double(count[0]) *0.15;
  }
 
  return theta;
}



void acionaMotor( double v ) {
  // Função para criar o sinal PWM de acordo com o valor passado 'v';
    v = (v > VFONTE) ? VFONTE : v;
    v = (v < -VFONTE) ? -VFONTE : v;
  if(v<0){
    Timer1.setPwmDuty( PINO1_PWM, map2( v, 0, -VFONTE, 1, 1024 ) );   // aciona motor em um sentido
    Timer1.setPwmDuty( PINO2_PWM, LOW );   // aciona motor em um sentido
  } else {
    Timer1.setPwmDuty( PINO1_PWM, LOW );   // aciona motor em um sentido
    Timer1.setPwmDuty( PINO2_PWM, map2( v, 0, VFONTE, 1, 1024 ) );   // aciona motor em um sentido
  }

}

void setaEncoders(byte ioMode, byte enablePCI) {
  // Função para ajustar configurações dos encoderes 
  switch (NENC) {
    case 1:
      pinMode( 2, ioMode); //  2, PD2   PCINT 18
      pinMode( 3, ioMode); //  3, PD3   PCINT 19
      break;
    case 2:
      pinMode( 2, ioMode); //  2, PD2   PCINT 18
      pinMode( 3, ioMode); //  3, PD3   PCINT 19
      pinMode( 4, ioMode); //  4, PD4   PCINT 20
      pinMode( 5, ioMode); //  5, PD5   PCINT 21
      break;
    case 3:
      pinMode( 2, ioMode); //  2, PD2   PCINT 18
      pinMode( 3, ioMode); //  3, PD3   PCINT 19
      pinMode( 4, ioMode); //  4, PD4   PCINT 20
      pinMode( 5, ioMode); //  5, PD5   PCINT 21
      pinMode( 6, ioMode); //  6, PD6   PCINT 22
      pinMode( 7, ioMode); //  7, PD7   PCINT 23
      break;
    default:
      pinMode( 2, ioMode); //  2, PD2   PCINT 18
      pinMode( 3, ioMode); //  3, PD3   PCINT 19
      break;
  }
  if (enablePCI) {
    PCMSK2 |= MASCARA;
    PCIFR  |= 0x04; // Clear PC interrupts if any
    PCICR  |= 0x04; // Enable PC interrupts
  }
  abOld = changedCount = 0;
  for (byte i = 0; i < NENC; ++i) {
    count[i] = 0;
  }
}

ISR(PCINT2_vect) {
  // Função do tipo interrupção externa, para contar pulsos dos encoderes (até
  // 4), tratando-os com uma lógica mais sofisticada, de modo que a leitura é
  // feita levando em conta o sentido de giro do encoder, e a resolução é
  // multiplicada por 4.
  //
  byte abNew = PIND & MASCARA; // Read pins  2, 3,
  byte changes = abNew ^ abOld; // Use XOR to flag changed bits
  char wayflag = 2 * abNew ^ abNew; // Use XOR to get state-bits
  byte test2;
  for (byte i = 0; i < NENC; ++i) {
    // For PD2-7, low 2 bits aren't relevant.  Move the shifts to
    // end of for loop if low 2 bits carry encoder info.
    wayflag >>= 2;      // Discard low state-bits
    changes >>= 2;      // Discard low change-bits
    test2 = changes & 3;    // Get 2 bits of interest
    if (test2 == 1) {
      count[i] += 1 - (wayflag & 2);
      changedCount = true;
    } else if (test2 == 2) {
      count[i] -= 1 - (wayflag & 2);
      changedCount = true;
    }
  }
  abOld = abNew;        // Save new state
}