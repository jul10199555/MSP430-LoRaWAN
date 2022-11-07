/////////////// Inclusión de librerias /////////////
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

/////////////// Definición de pines ////////////////
#define RFM95_RST 13
#define RFM95_CS  18
#define RFM95_DIO0 12
#define RFM95_DIO1 11
#define RFM95_DIO2 5
#define CLOCK_PIN 9
#define CSLORA 18
#define CSL 8
#define CSR 2
#define P_Bat A7
#define EN_Bat 3
#define SCK 7
#define MISO 14
#define RF_En 19
#define mic_En 4

//////////////// Variables del programa ////////////
const int samples = 4500;
int x[samples];
float dBL = 0;
float dBR = 0;
float dB = 0;
float L = 0;
float offset __attribute__((section(".text")));
float bat_Level = 0;
float Mean = 0;         //Media
float VRMS = 0;     //Voltaje RMS
float Volt2 = 0;    //Voltaje cuadrático
float VoltA = 0;    //Voltaje Filtrado
int VoltsL[samples] __attribute__((section(".text"))); //Volts RAW
int VoltsR[samples] __attribute__((section(".text"))); //Volts RAW
const unsigned SLEEP_TIME = 80; //Sleep_Time =(t-27.8)/3.38 s

static osjob_t sendjob;

float Y[7] = {0, 0, 0, 0, 0, 0, 0};
float X[7] = {0, 0, 0, 0, 0, 0, 0};

//////////////////////////////////////////////////////////// Direcciones de los nodos //////////////////////////////////////////////////////////////////////

//ID: 00027
static const PROGMEM u1_t NWKSKEY[16] = { 0xE6, 0x79, 0x4A, 0x5C, 0x1A, 0x7C, 0x54, 0x46, 0x51, 0xEE, 0x90, 0xA6, 0x4E, 0x7D, 0x77, 0x61 };
static const u1_t PROGMEM APPSKEY[16] =  { 0xD0, 0xC9, 0x6C, 0xD3, 0x85, 0xBC, 0x4A, 0x6E, 0x9F, 0xD3, 0x3A, 0x8D, 0xAA, 0xEE, 0xB7, 0xA5 };
static const u4_t DEVADDR = 0x260115A0;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void setup() {
  ///////////// Configurar Blink /////////////
  Interrup_Setting();

  ///////////// Inicialización de pines /////////////
  attachInterrupt(PUSH1, decrement, FALLING);
  attachInterrupt(PUSH2, increment, FALLING);
  pinMode(PUSH1, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP);
  pinMode(MISO, INPUT_PULLUP);
  pinMode(EN_Bat, INPUT);
  pinMode(SCK, OUTPUT);
  pinMode(CSL, OUTPUT);
  pinMode(CSR, OUTPUT);
  pinMode(CSLORA, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  pinMode(RF_En, OUTPUT);
  pinMode(mic_En, OUTPUT);

  /////////// Parámetros de comunicación  /////////////////
  Serial.begin(115200);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV4);

  /////////// Configuración Inicial  //////////////////
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(CSLORA, HIGH);
  digitalWrite(CSL, HIGH);
  digitalWrite(CSR, HIGH);
  digitalWrite(RFM95_CS, HIGH);
  digitalWrite(RF_En, HIGH);
  digitalWrite(mic_En, LOW);

  if (digitalRead(PUSH1) == 0) Reset_Offset();

  LoRa(true);
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

float a[7] = {1.00000, -3.910397, 5.771770, -3.850766, 1.028231, -0.039243, 0.000405};
float b[7] = {0.281475, -0.562950, -0.281475, 1.125900, -0.281475, -0.562950, 0.281475};

float A(float Volts) {

  for (int i = 6; i > 0; i--)
  {
    X[i] = X[i - 1];
    Y[i] = Y[i - 1];
  }

  X[0] = Volts;
  Y[0] = b[6] * X[6] + b[5] * X[5] + b[4] * X[4] + b[3] * X[3] + b[2] * X[2] + b[1] * X[1] + b[0] * X[0] - ( a[6] * Y[6] + a[5] * Y[5] + a[4] * Y[4] + a[3] * Y[3] + a[2] * Y[2] + a[1] * Y[1]);

  return  Y[0];

}

void Interrup_Setting()
{
  P4DIR |= BIT6;               // P1.0 salida
  TA1CTL = TASSEL_2 | TACLR | MC_1 | ID_3;
  TA1CCR0 = 65535;
  TA1CCTL0 = CCIE; // Habilita interrupción (bit CCIE)
  __bis_SR_register(LPM0_bits | GIE);
  __no_operation();
}

// Rutina de interrupción de TIMER1_A0
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void) {
  P4OUT ^= BIT6;     // conmuta LED en P1.0
}

void increment() {
  offset = offset + 0.5;
}

void decrement() {
  offset = offset - 0.5;
}

void Reset_Offset() {
  offset = 0.0;
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(50);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  delay(50);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(50);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  delay(50);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(50);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  delay(50);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(50);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  delay(50);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(50);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  delay(50);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(50);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  delay(50);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(50);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  delay(50);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(50);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  delay(50);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(50);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
}

void LoRa(boolean S) {

  if (S == true) {
    digitalWrite(RF_En, HIGH);
    os_init();
    LMIC_reset();

#ifdef PROGMEM
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868) //Canales para el gateway
    LMIC_setupChannel(0, 915000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 915000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 915000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 915000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
#elif defined(CFG_us915)
    LMIC_selectSubBand(1);
#endif


    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
    LMIC.txpow = 27;
    LMIC_setDrTxpow(DR_SF12, 14); //0db

  }
  else {
    digitalWrite(RF_En, HIGH);
  }

}

int pmodMic3L() {
  byte Buffer[2];
  uint16_t X = 0;

  P3OUT &= ~BIT4;
  for (int i = 0; i < 2; i++)
  {
    Buffer[i] = SPI.transfer(0); // Obtener los 2 bytes del micrófono
  }

  P3OUT |= BIT4;

  X = Buffer[1]; //Se asigna los primeros 8 bits a X
  X |= (Buffer[0] << 8); //Se le aplica una or y un corrimiento para recuperar los 12 bits
  return X;
}

int pmodMic3R() {
  byte Buffer[2];
  int X = 0;

  P4OUT &= ~BIT2;
  for (int i = 0; i < 2; i++)
  {
    Buffer[i] = SPI.transfer(0); // Obtener los 2 bytes del micrófono
  }
  P4OUT |= BIT2;

  X = Buffer[1]; //Se asigna los primeros 8 bits a X
  X |= (Buffer[0] << 8); //Se le aplica una or y un corrimiento para recuperar los 12 bits
  return X;
}

float getBatLevel() {
  float level = 0.0;
  pinMode(EN_Bat, OUTPUT);
  digitalWrite(EN_Bat, LOW);
  level = analogRead(P_Bat) * 0.001612;
  pinMode(EN_Bat, INPUT);
  return level;
}

void default_Clock_Settings() {

  CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
  CSCTL1 = DCOFSEL_6;                       // Set DCO to 8MHz
  CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;  // Set SMCLK = ACLK = VLO
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers to 1
  CSCTL0_H = 0;                             // Lock CS registers
  Serial.begin(115200);

}

void Clock_Settings() {

  P2DIR |= BIT0;
  P2SEL0 |= BIT0;
  P2SEL1 |= BIT0;
  P3DIR |= BIT4;
  P3SEL0 |= BIT4;
  P3SEL1 |= BIT4;
  PJSEL0 |= BIT4 | BIT5 | BIT6 | BIT7;
  FRCTL0 = FRCTLPW | NWAITS_1;
  PM5CTL0 &= ~LOCKLPM5;
  CSCTL0_H = CSKEY >> 8;
  CSCTL1 = DCORSEL | DCOFSEL_4;
  CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
  CSCTL0_H = 0;

  Serial.begin(57600);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV4);

}

void do_send(osjob_t* j) {

  getData();
  delay(100);
  Serial.println("/////// DATA: ////////");
  Serial.print(dB);
  Serial.println(" dB");

  Serial.print(bat_Level);
  Serial.println(" V");

  float celsius = 102.43;
  float humi = 201.23;

  u2_t temperature = (u2_t)(celsius * 100);
  u2_t humidity = (u2_t)(humi * 100);
  u2_t slp = (u2_t)(dB * 100);
  u2_t bat_level_I = (u2_t)(bat_Level * 100);

  LMIC.frame[0] = (slp >> 8) & 0xFF;
  LMIC.frame[1] = slp & 0xFF;
  LMIC.frame[2] = (temperature >> 8) & 0xFF;
  LMIC.frame[3] = temperature & 0xFF;
  LMIC.frame[4] = (humidity >> 8) & 0xFF;
  LMIC.frame[5] = humidity & 0xFF;
  LMIC.frame[6] = (bat_level_I >> 8) & 0xFF;
  LMIC.frame[7] = bat_level_I & 0xFF;

  delay(100);

  default_Clock_Settings();

  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    LMIC_setTxData2(1, LMIC.frame, 8, 0); // (port 1, data, 4 bytes, unconfirmed)
    Serial.println(F("Send Data..."));
  }
}

void getData() {

  digitalWrite(mic_En, HIGH);
  delay(2000);

  Clock_Settings();

  /////// OBTENER EL VALOR  ENTREGADO POR LOS MICRÓFONOS ////////
  for (int i = 0; i < samples; i++) {
    VoltsL[i] = pmodMic3L();
    VoltsR[i] = pmodMic3R();
  }

  digitalWrite(mic_En, LOW);
  //digitalWrite(RF_En, HIGH);
  //delay(2000);

  ////////// OBTENER VALOR RMS DE MICRÓFONO IZQUIERDO /////////
  for (int i = 0; i < samples; i++) {
    Volt2 = (VoltsL[i] - 2048) * 0.001221;
    VoltA = A(Volt2);
    Volt2 = VoltA * VoltA;
    Mean += Volt2;
  }
  VRMS = sqrt(Mean / samples);


  //////// Obtención de la intensidad en dB DE MICRÓFONO IZQUIERDO ///////////
  if (VRMS < 1)L = d_log(VRMS);
  else L = log_10(VRMS / 0.01258);
  dBL = 20 * L + 94 - 26 + offset;
  ////////////// Limpiar variables ///////////
  for (int i = 0; i < 7; i++) {
    X[i] = 0;
    Y[i] = 0;
  }
  Mean = 0;
  VRMS = 0;
  VoltA = 0;

  ///////// OBTENER VALOR RMS DE MICRÓFONO DERECHO /////////
  for (int i = 0; i < samples; i++) {
    Volt2 = (VoltsR[i] - 2048) * 0.001221;
    VoltA = A(Volt2);
    Volt2 = VoltA * VoltA;
    Mean += Volt2;
  }
  VRMS = sqrt(Mean / samples);

  //////// Obtención de la intensidad en dB DE MICRÓFONO DERECHO ///////////
  if (VRMS < 1)L = d_log(VRMS);
  else L = log_10(VRMS / 0.01258);
  dBR = 20 * L + 94 - 26 + offset;

  ////////////// Limpiar variables ///////////
  for (int i = 0; i < 7; i++) {
    X[i] = 0;
    Y[i] = 0;
  }
  Mean = 0;
  VRMS = 0;
  VoltA = 0;

  dB = (dBL + dBR) / 2;
  dB = dBR;
  bat_Level = getBatLevel();

}

const lmic_pinmap lmic_pins = {
  RFM95_CS,   //nss
  LMIC_UNUSED_PIN,    //rxtx
  RFM95_RST,  //rst
  {RFM95_DIO0, RFM95_DIO1, LMIC_UNUSED_PIN},  //DIO0,DIO1,DIO2
};

///////////////////   Logaritmo base 10  /////////////////////
float log_10(float n) {
  float val = 0;
  float b = 10.0;
  int i, accurate = 10, reps = 0;
  while (n != 1 && accurate >= 0) {
    for (i = 0; n >= b; i++) n /= b;
    n = pow_(n, 10);
    val = 10 * (val + i);
    accurate--; reps++;
  }
  return (float)val / pow_(10, reps);
}

float pow_(float x, int i) {
  float r = 1.0;
  for (i; i > 0; i--) r *= x;
  return r;
}


/////////////// Logaritmo Natural (Base e) ////////////////////
float log_n(float n) {
  float b = 2.718281;
  float val = 0;
  int i, accurate = 10, reps = 0;
  while (n != 1 && accurate >= 0) {
    for (i = 0; n >= b; i++) n /= b;
    n = pe(n, 10);
    val = 10 * (val + i);
    accurate--; reps++;
  }
  return (float)val / pe(10.0, reps);
}

float pe(float x, int i) {
  float r = 1.0;
  for (i; i > 0; i--) r *= x;
  return r;
}

///////////////////  Logaritmo de número decimal //////////////////////////
float d_log(float n) {
  float result = 0;
  result = - log_10(1 / n) + 1.900319354;
  return result;
}

void onEvent (ev_t ev) { //
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("Sent!"));

      P1OUT |= BIT0;
      delay(200);
      P1OUT &= ~BIT0;
      delay(200);
      P1OUT |= BIT0;
      delay(200);
      P1OUT &= ~BIT0;
      delay(200);
      P1OUT |= BIT0;
      delay(200);
      P1OUT &= ~BIT0;

      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }

      Serial.println(F("Sleep!"));
      //digitalWrite(LED2, HIGH);
      LoRa(false);
      sleepSeconds(SLEEP_TIME);
      //digitalWrite(LED2, LOW);
      LoRa(true);
      Serial.println(" ");
      Serial.println(F("Wake Up!"));
      delay(100);

      os_setCallback(&sendjob, do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}
