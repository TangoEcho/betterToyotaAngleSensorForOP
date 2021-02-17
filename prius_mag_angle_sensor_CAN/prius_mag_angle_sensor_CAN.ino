#include <Encoder_Buffer.h>
#include <CAN.h>
//#include <can.h>
//#include <mcp2515.h>
#include <SPI.h>

//CAN
struct can_frame canMsg1;
struct can_frame canMsg2;
//MCP2515 mcp2515(3);

//ANGSENSOR
#define EncoderCS1 10
int32_t encoder1Reading = 0;
int32_t lastencoder1Reading = 0;
int32_t rate = 0;
Encoder_Buffer Encoder1(EncoderCS1);

void setup() {
  
  Serial.begin(115200); //Serial set to ZSS speed
    while (!Serial); 

  SPI.begin();

  //Adafruit setup

  Serial.println("CAN Sender");
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster

  // start the CAN bus 
  if (!CAN.begin(500E3)) { //start at 500 kbps
    Serial.println("Starting CAN failed!");
    while (1);
  }
  
  //INIT ANGSENSOR
  Encoder1.initEncoder();
}

void loop() {
  //ANGSENSOR
  encoder1Reading = Encoder1.readEncoder(); //READ the ANGSENSOR
  //Serial.println(encoder1Reading);

  rate = abs(encoder1Reading) - abs(lastencoder1Reading);

  lastencoder1Reading = encoder1Reading;

 //CAN Array Setup
  canMsg1.can_id = 0x23;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = (encoder1Reading >> 24) & 0xFF; //Bitshift the ANGSENSOR (Cabana errors with 32 bit)
  canMsg1.data[1] = (encoder1Reading >> 16) & 0xFF;
  canMsg1.data[2] = (encoder1Reading >> 8) & 0xFF;
  canMsg1.data[3] = (encoder1Reading >> 0) & 0xFF;
  canMsg1.data[4] = (rate >> 16) & 0xFF;
  canMsg1.data[5] = (rate >> 8) & 0xFF;
  canMsg1.data[6] = (rate >> 0) & 0xFF;
  canMsg1.data[7] = can_cksum (canMsg1.data, 7, 0x230); //Toyota CAN CHECKSUM

  //CAN for m4
  Serial.print("Sending CAN Packet");

  CAN.beginPacket(0x200, 8); //Start-(id, dlc) https://github.com/adafruit/arduino-CAN/blob/master/API.md

  //Send by line 
 
  CAN.write(canMsg1.data[0]); //0-Array Bit 0
  CAN.write(canMsg1.data[1]); //1
  CAN.write(canMsg1.data[2]); //2
  CAN.write(canMsg1.data[3]); //3
  CAN.write(canMsg1.data[4]); //4
  CAN.write(canMsg1.data[5]); //5
  CAN.write(canMsg1.data[6]); //6
  CAN.write(canMsg1.data[7]); //7-Array Bit 7

  CAN.endPacket(); //End sequence of sending packget. Returns 1 on success 0 on failure

  //Serial.println(canMsg1.data[4]);
  
  delay(10);

}

//TOYOTA CAN CHECKSUM
int can_cksum (uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  //uint16_t temp_msg = msg;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}
