/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <lmic.h>  //LMIC Bibliothek
#include <hal/hal.h> //LMIC Bibliothek
#include <SPI.h> //LMIC Bibliothek

#include <LowPower.h> //Low-Power Bibliothek
// PINchangeinterrupt von hier

#define PCINT_PIN A5 //Pin festelung für pinchangeinterrupt
#define PCINT_MODE CHANGE //Auswahl des Moduses für PinChangeinterrupt





// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;



// Sendeintervall in sec (könnte während der Laufzeit länger werden)
unsigned TX_INTERVAL = 600;        //seconds of sleep between sends


unsigned SLEEPCYLCES= TX_INTERVAL/8; // calculate the number of sleepcycles (2s/8s) needed for the given TX_INTERVAL
unsigned senleha =9600;
// Use pin 2 as wake up pin



// Pin mapping, hier legt man die Pinbelegung für Arduino mit Lora Modul fest

//   RFM95 / Arduino 
const lmic_pinmap lmic_pins = {
  .nss = 10,                     //Slave/Master
  .rxtx = LMIC_UNUSED_PIN,       // Antenne
  .rst = LMIC_UNUSED_PIN,        //Reset
  .dio = {2, 7, 8},              // (DIO0,DIO1,DIO2)
};

volatile uint16_t interruptCount=0; // The count will go back to 0 after hitting 65535.


int ledPin = 13;//LED


volatile boolean powerdown=false; // neu sleepmode

unsigned int Vbat; //Batteriestatus in Volt
unsigned int VbatP; //Batteriestatus in Prozent

//Einstellungen für Lora, nicht verändern
void onEvent (ev_t ev) {

  Serial.println("TX_Intervall lautet:");
  Serial.println(TX_INTERVAL);
  Serial.println("Anzahl der Sleepcycles:");
  Serial.println(SLEEPCYLCES);
    Serial.print((unsigned long)(os_getTime()/ OSTICKS_PER_SEC));
    Serial.print(": ");
    switch(ev) {
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
            Serial.println(F("EV_JOINING")); //The node has started joining the network
            break;
        case EV_JOINED:                     //The node has successfully joined the network and is now ready for dara exchange
            Serial.println(F("EV_JOINED"));
            /*
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (int i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (int i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
            */
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
            // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));  //The node could not join the network
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED")); //The node did not join a new network but is still connected to the old network
            break;

    case EV_TXCOMPLETE:          //The dara prepared via LMIC-setTxData() has been sent, and the receive window for downstream data ist complete.
      
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

        if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));

        Serial.println(F("sent"));

        Serial.print(F("Received "));


        
//Downlink
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));


        

         
        
          switch(LMIC.frame[LMIC.dataBeg ]) // das erste byte ist enweder 1 der 2. Je nachdem welche es ist führt es zu den unterschiedlichen cases.
          {
          case 1:  // Wenn case 1 ausgelöst wird, dann ändert sich das Sendeintervall
        
          TX_INTERVAL=((LMIC.frame[LMIC.dataBeg + 1]) <<8 | LMIC.frame[LMIC.dataBeg+2])*60;
          Serial.println(" New TX-interval is:");
          Serial.println(TX_INTERVAL);
          break;
          
          case 2:  // Wenn case 2 ausgelöst wird, dann ändert sich Senderlesehäufigkeit
         
        
          senleha=(LMIC.frame[LMIC.dataBeg + 1]) <<8 | LMIC.frame[LMIC.dataBeg+2]; // Unsigned weil es keine negativven Sensorwerte gibt. 
                                                                                           //Nachgucken wie ich das am besten erklären soll.
          Serial.println("New sensor reading rate is:");                                                                                
          Serial.println(senleha); 
          break; // Falls keine Veränderung eintritt bzw. weder 1 noch 2 beim case ist, dann bricht es ab.
          default: Serial.println("Keine Veränderung");
          }
         
         SLEEPCYLCES = TX_INTERVAL/8; // Änderung des Schlafintervalls
        

      
// Downlink Ende
      
      powerdown=true; // sleepmode neu
      break;
    
        case EV_LOST_TSYNC: //Beacon was missed repeatedly and time synchronization has been lost. Tracking or pinging needs to be restarted
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET: // Session reset due to rollover of sequence counters. Network will be rejoined automatically to acquiere new session
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:  // NO confirmation has been received from the network server fpr an extended period of time. Transmissions are still possible but their 
                           //reception is uncertain
            Serial.println(F("EV_LINK_DEAD")); 
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));  //Link was dead, but now is alive
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART: //This event is reported just before telling the radio driver to start transmission.
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

// Was hier steht wird pro Sendeintervall immer einmal durchgeführt
void do_send(osjob_t* j) {
byte ID=1;
byte Status= 0; 

 if(digitalRead(A5)==LOW){
   Serial.println("Tuer ist zu ");
  Status = 0;
 }else{
  Serial.println("Tuer ist offen ");
  Status =1;
  }
  
//Payload part

static uint8_t mydata[3];
      mydata[0]= ID;
      mydata[1]= Status; 
      mydata[2]= VbatP;  //Batteriestatus in Prozent  
  
      
      

      


  
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
       
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);  
        Serial.println(F("Packet queued"));
        
    }    
}













void wakeUp()
{
    // Just a handler for the pin interrupt.
}

void setup()
{
  
    pinMode(PCINT_PIN, INPUT_PULLUP);

  // attach the new PinChangeInterrupt
  attachPinChangeInterrupt();

  pinMode(ledPin, OUTPUT); //LED
   digitalWrite(ledPin, HIGH); //LED
  Serial.begin(senleha); // Auslese bzw. Übertragungsrate. Aufpassen, wenn es zu hoch ist dann kann der Sensor nicht mitkommen und gibt Fehler aus
  Serial.println(F("Starting"));

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);  //Relax RX timing window

  // Start job (sending automatically starts OTAA too)
  
   
  BatMes();//Do a first measurement at startup
  powerdown=false;
  os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(10), do_send); 
}

void loop() 
{





 extern volatile unsigned long timer0_overflow_count;
  os_runloop_once();  //check send status

  if (powerdown) {
    
    //delay(1000);
    Serial.println(F("go to sleep ... "));
    Serial.flush();
   
    for (int i=0; i<SLEEPCYLCES; i++) {
         

 
      // Enter power down state for 8 s with ADC and BOD module disabled 
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      
      //LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF); 
            
      //Give the AVR back the slept time back 


         // Disable external pin interrupt on wake up pin.
    
      cli(); //disable interrupt
      timer0_overflow_count += 8 * 64 * clockCyclesPerMicrosecond(); //give back 60 seconds of sleep
      sei(); //enable interrupt
      os_getTime();   //VERY IMPORTANT after sleep to update os_time and not cause txbeg and os_time out of sync which causes send delays with the RFM95 on and eating power

      
  
}

    Serial.begin(senleha);
    Serial.println(F("... done sleeping")); 
  
    //delay(1000);
    powerdown=false;


   // measure();  //get some data 
   BatMes();
 os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(10), do_send);  //do a send // os_getTime() ist die Systemzeit in ticks // mit diesem Befehl initialiesiern 
 //wir das versenden eines Packetes in 10 Millisekunden
  }

















  
}

void BatMes() //Batteriemessung
{

int adc_low, adc_high;  //Zwischenspeicher für die Ergebnisse des ADC
long adc_result;  //Gesamtergebnis der Messung des ADC
long vcc;  //Versorgungsspannung
   
  

  ADMUX |= (1<<REFS0); //VCC als Referenzspannung für den AD-Wandler
  ADMUX |= (1<<MUX3) | (1<<MUX2) | (1<<MUX1);  //1.1V Referenzspannung als Eingang für ADC 
  delay(10);  //warten bis sich die Referenzspannung eingestellt hat
  ADCSRA |= (1<<ADEN);   //ADC aktivieren



 
  ADCSRA |= (1<<ADSC);  //Messung starten

  while (bitRead(ADCSRA, ADSC));  //warten bis Messung beendet ist
  //Ergebnisse des ADC zwischenspeichern. Wichtig: zuerst ADCL auslesen, dann ADCH
  adc_low = ADCL;
  adc_high = ADCH;

  adc_result = (adc_high<<8) | adc_low; //Gesamtergebniss der ADC-Messung 
  vcc = 1125300L / adc_result;  //Versorgungsspannung in mV berechnen (1100mV * 1023 = 1125300)

 Serial.println("der Batteriewert");
  Serial.println(vcc);
  VbatP = vcc/100;



}

#define PCMSK *digitalPinToPCMSK(PCINT_PIN)
#define PCINT digitalPinToPCMSKbit(PCINT_PIN)
#define PCIE  digitalPinToPCICRbit(PCINT_PIN)
#define PCPIN *portInputRegister(digitalPinToPort(PCINT_PIN))

#if (PCIE == 0)
#define PCINT_vect PCINT0_vect
#elif (PCIE == 1)
#define PCINT_vect PCINT1_vect
#elif (PCIE == 2)
#define PCINT_vect PCINT2_vect
#else
#error This board doesnt support PCINT ?
#endif

volatile uint8_t oldPort = 0x00;

void attachPinChangeInterrupt(void) {
  // update the old state to the actual state
  oldPort = PCPIN;

  // pin change mask registers decide which pins are enabled as triggers
  PCMSK |= (1 << PCINT);

  // PCICR: Pin Change Interrupt Control Register - enables interrupt vectors
  PCICR |= (1 << PCIE);
}

void detachPinChangeInterrupt(void) {
  // disable the mask.
  PCMSK &= ~(1 << PCINT);

  // if that's the last one, disable the interrupt.
  if (PCMSK == 0)
    PCICR &= ~(0x01 << PCIE);
}

ISR(PCINT_vect) {
  // get the new and old pin states for port
  uint8_t newPort = PCPIN;

  // compare with the old value to detect a rising or falling
  uint8_t change = newPort ^ oldPort;

  // check which pins are triggered, compared with the settings
  uint8_t trigger = 0x00;
#if (PCINT_MODE == RISING) || (PCINT_MODE == CHANGE)
  uint8_t rising = change & newPort;
  trigger |= (rising & (1 << PCINT));
#endif
#if (PCINT_MODE == FALLING) || (PCINT_MODE == CHANGE)
  uint8_t falling = change & oldPort;
  trigger |= (falling & (1 << PCINT));
#endif

  // save the new state for next comparison
  oldPort = newPort;

  // if our needed pin has changed, call the IRL interrupt function
  /*
  if (trigger & (1 << PCINT))
    PCINT_FUNCTION();

    */
}
