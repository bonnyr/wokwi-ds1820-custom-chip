#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(10);
DallasTemperature sensor(&oneWire);

void test_dallas_temp_lib_setup(void) {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, HIGH);

  delay(2);
  sensor.begin();
  delay(20);

  Serial.print("Device count: "); Serial.println(sensor.getDeviceCount());

  DeviceAddress addr;
  if (!sensor.getAddress(addr, 0)) {
    Serial.println("Device not found\n");
  };

  for (int i=0; i<8;i++){
    Serial.print(addr[i], HEX); Serial.print(" ");
  }
  Serial.println();

}

void test_dallas_temp_lib_loop(void) {
  sensor.requestTemperatures();
  Serial.print("Temperature is: ");
  Serial.println(sensor.getTempCByIndex(0));
  delay(1000);
}
