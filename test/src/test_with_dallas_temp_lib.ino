#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(10);
DallasTemperature sensor(&oneWire);

DeviceAddress addr;
uint8_t scratch[9];

void test_dallas_temp_lib_setup(void) {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, HIGH);

  delay(2);
  sensor.begin();
  delay(20);

  Serial.print("Device count: "); Serial.println(sensor.getDeviceCount());

  if (!sensor.getAddress(addr, 0)) {
    Serial.println("Device not found\n");
  };

  for (int i=0; i<8;i++){
    Serial.print(addr[i], HEX); Serial.print(" ");
  }
  Serial.println();

}

void test_dallas_temp_lib_loop(void) {
  bool b;

  
  
  sensor.requestTemperatures();
  Serial.print(millis()); Serial.print(" deviceConnected: "); Serial.println(sensor.isConnected(addr));
  Serial.print(millis()); Serial.print(" scratch: res="); 
  b = sensor.readScratchPad(addr, scratch);
  Serial.println(b);
  for (int i=0; i<9;i++){
    Serial.print(" "); Serial.print(scratch[i], HEX); 
  }
  Serial.println();

  // configure chip for 0x7F high, 0x00 Low, 9 bit res
  Serial.print(millis()); Serial.println("setting High Alarm threshold: "); 
  sensor.setHighAlarmTemp(addr, (uint8_t)125);
  Serial.println(sensor.getHighAlarmTemp(addr)); 
  Serial.print(millis()); Serial.println(" setting Low Alarm threshold: "); 
  sensor.setLowAlarmTemp(addr, (uint8_t)0);
  Serial.println(sensor.getLowAlarmTemp(addr)); 


  testResolution(9);
  testResolution(10);
  testResolution(11);
  testResolution(12);

  // test alarm condition
  testAlarmConditions(125, 30);
  testAlarmConditions(125, 20);
  testAlarmConditions(21, 20);
  testAlarmConditions(99, 55);
  testAlarmConditions(55, 15);


  delay(5000); 
}


static void testResolution(int res) {
  bool b;
  Serial.print(millis()); Serial.println(" === Testing Resolution "); 
  b = sensor.setResolution(addr, res);
  sensor.requestTemperatures();
  Serial.print(millis()); Serial.print("      "); Serial.print(res); Serial.print(" bit (set: "); Serial.print(b?"yes":"no"); 
  Serial.print(") Temperature: "); Serial.println(sensor.getTempCByIndex(0));
}

static void testAlarmConditions(int high, int low) {
  DeviceAddress almAddr;
  Serial.print(millis()); Serial.print(" ==== Testing Alarm Condition (high: "); Serial.print(high); Serial.print(", low: "); Serial.print(low); Serial.println(")"); 
  Serial.print(millis()); Serial.print("      Setting High Alarm Threshold ... "); 
  sensor.setHighAlarmTemp(addr, high);
  Serial.println(sensor.getHighAlarmTemp(addr));
  Serial.print(millis()); Serial.print("      Setting Low Alarm Threshold ... "); 
  sensor.setLowAlarmTemp(addr, low);
  Serial.println(sensor.getLowAlarmTemp(addr));
  Serial.print(millis()); Serial.print("      Read Temp: "); 
  sensor.requestTemperatures();
  float t = sensor.getTempCByIndex(0);
  Serial.println(t);
  delay(1);
  Serial.print(millis()); Serial.print("      Performing Alarm Search (t: "); Serial.print(t); Serial.print(", high: "); Serial.print(high); Serial.print(", low: "); Serial.print(low); Serial.print(") :"); 
  sensor.resetAlarmSearch();
  bool b = sensor.alarmSearch(almAddr); 
  Serial.println(b);
}


static const uint8_t dscrc2x16_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};


// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Use tiny 2x16 entry CRC table)
static uint8_t crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
  	uint8_t b = *addr++;
		crc = b ^ crc;  // just re-using crc as intermediate
		// crc = *(dscrc2x16_table + (crc & 0x0f)) ^ *(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
    Serial.print("crc8: b: "); Serial.print(b); Serial.print(", crc: "); Serial.print(crc, HEX); Serial.print(", low: ");Serial.print(dscrc2x16_table[crc & 0x0f], HEX);Serial.print(", high: ");Serial.println(dscrc2x16_table[ 16 + ((crc >> 4) & 0x0f)], HEX); 
		crc = dscrc2x16_table[crc & 0x0f] ^ dscrc2x16_table[ 16 + ((crc >> 4) & 0x0f)];
	}

	return crc;
}

