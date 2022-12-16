


// #define LED_PIN 3

// void setup() {
//   pinMode(LED_PIN, OUTPUT);
//   Serial.begin(115200);
// }

// void loop() {
//   static int cnt;
//   Serial.println(cnt++);
//   digitalWrite(LED_PIN, HIGH);
//   delay(1000);
//   digitalWrite(LED_PIN, LOW);
//   delay(1000);
// }

#include <OneWire.h>

OneWire  ds(10);  // on pin 10

void test_one_wire_lib_setup(void) {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, HIGH);
}

void test_one_wire_lib_loop(void) {
  static bool cycle;
  // static int cnt;
  // byte i;
  // byte present = 0;
  // byte data[12];
  uint8_t scratch[9];       
  byte addr[8] = {0x9F, 0x9D, 0x87, 0x67, 0x99, 0xC4, 0xF7, 0x07};
  

  digitalWrite(3, cycle ? HIGH : LOW);
  cycle = !cycle;
  Serial.println(cycle ? "." : "+");
  delay(5);
  // return;

  if ( !ds.search(addr)) {
    Serial.print("No more addresses.\n");
    ds.reset_search();
    return;
  }





  Serial.print("found device!\n");
  ds.reset_search();
  delay(1);
  Serial.print(millis()); Serial.print(" resetting device before write\n");
  ds.reset();
  delay(2);
  Serial.print(millis()); Serial.print(" skipping device\n");
  ds.skip();
  delay(3);
// ds.select(addr);
  Serial.print(millis()); Serial.print(" writing scratchpad.\n");
  ds.write(0x4E);         
  ds.write(0x37);         
  ds.write(0x89);         
  ds.write(0x13);         


  delay(1);
  Serial.print(millis()); Serial.print(" resetting device before read\n");
  ds.reset();
  Serial.print(millis()); Serial.print(" skipping device\n");
  ds.skip();
  delay(1);
  Serial.print(millis()); Serial.print(" reading scratchpad.\n");
  ds.write(0xBE);         
  for (int i=0; i<9;i++){
    uint8_t b = ds.read();
    Serial.print("read byte "); Serial.print(1); Serial.print(", received: "); Serial.println(b,HEX);
  }
  Serial.print(millis());  Serial.print(" reading scratchpad done.\n");


  delay(1);
  Serial.print(millis()); Serial.print(" resetting device before read\n");
  ds.reset();
  Serial.print(millis()); Serial.print(" reading rom.\n");
  ds.write(0x33);  
  uint8_t rom[8];       
  for (int i=0; i<8;i++){
    uint8_t b = ds.read();
    Serial.print("read rom byte "); Serial.print(1); Serial.print(", received: "); Serial.println(b,HEX);
  }
  bool match = memcmp(rom, addr, 8);
  Serial.print(millis());  Serial.print(" reading done done. addr match: "); Serial.println(match);

  Serial.print(millis()); Serial.print(" writing scratchpad.\n");
  ds.write(0x4E);         
  ds.write(0x73);         
  ds.write(0x54);         
  ds.write(0x87);         


  delay(1);
  Serial.print(millis()); Serial.print(" resetting device before select & convert\n");
  ds.reset();
  Serial.print(millis()); Serial.print(" selecting rom.\n");
  ds.select(addr);
  Serial.print(millis()); Serial.print(" converting.\n");
  ds.write(0x44);  
  Serial.print(millis()); Serial.print(" converted.\n");



  delay(1);
  Serial.print(millis()); Serial.print(" resetting device before select & read\n");
  ds.reset();
  Serial.print(millis()); Serial.print(" selecting rom.\n");
  ds.select(addr);
  Serial.print(millis()); Serial.print(" reading scratchpad.\n");
  ds.write(0xBE);  
  for (int i=0; i<9;i++){
    uint8_t b = ds.read();
    Serial.print("read rom byte "); Serial.print(1); Serial.print(", received: "); Serial.println(b,HEX);
  }


  delay(1);
  Serial.print(millis()); Serial.print(" resetting device before select & copy scratch pad\n");
  ds.reset();
  Serial.print(millis()); Serial.print(" selecting rom.\n");
  ds.select(addr);
  Serial.print(millis()); Serial.print(" copying scratchpad.\n");
  ds.write(0x48);  
  Serial.print(millis()); Serial.print(" resetting device before select & recall pad\n");
  ds.reset();
  ds.skip();
  ds.write(0xB8);  
  bool bit = ds.read_bit();
  Serial.print(millis());  Serial.print(" recall done: "); Serial.println(bit);
  Serial.print(millis()); Serial.print(" resetting device before select & read after recall\n");
  ds.reset();
  ds.skip();
  ds.write(0xBE);  
  for (int i=0; i<9;i++){
    uint8_t b = ds.read();
    Serial.print("read rom byte "); Serial.print(1); Serial.print(", received: "); Serial.println(b,HEX);
  }



  Serial.println("===================");
  delay(5000);

}
