#include <OneWire.h>
#include <DallasTemperature.h>

extern DallasTemperature sensor;

extern DeviceAddress addr;
extern uint8_t scratch[];

void test_dyn_temp_setup(void) {

  Serial.begin(115200);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, LOW); // change to HIGH to capture signalling info




  Serial.println("Starting Delay");
  delay(10000);
  Serial.println("Starting");



  delay(2);
  sensor.begin();

  Serial.print("Device count: "); Serial.println(sensor.getDeviceCount());
  if (!sensor.getAddress(addr, 0)) {
    Serial.println("Device not found\n");
  };

  for (int i=0; i<8;i++){
    Serial.print(addr[i], HEX); Serial.print(" ");
  }
  Serial.println();

}

void test_dyn_temp_loop(void) {
  sensor.requestTemperatures();
  Serial.print(millis()); 
  Serial.print(" Read Temp: "); 
  float t = sensor.getTempCByIndex(0);
  Serial.println(t);
}


