#include <bluefruit.h>
#include <math.h>
#include <RAK13010_SDI12.h>
#include <U8g2lib.h>

#define DEBUG 0
#define SAMPLING_RATE 1000  // milliseconds

// DISPLAY
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2);  // R2 = Rotate display 180°
char displayMsg[32];

// METER ATMOS22
#define TX_PIN WB_IO6
#define RX_PIN WB_IO5
#define OE_PIN WB_IO4
RAK_SDI12 mySDI12(RX_PIN, TX_PIN, OE_PIN);
float windSpd = 0.0;
float windDir = 0.0;
float windTmp = 0.0;
time_t nextPollingTime = 0;

// BLUETOOTH
#define MAX_SAMPLERS 4
typedef struct {
  char name[16 + 1];
  uint16_t conn_handle;
  BLEClientUart clientUart;
} Peripheral;
Peripheral peripherals[MAX_SAMPLERS];
uint8_t connections = 0;
#define BLE_BUF_SIZE 20  // default BLEUart packet size
char bleMsg[BLE_BUF_SIZE];
char bleName[15] = "ATMOS22 LOGGER";

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
#if DEBUG
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.printf("Starting %s\n", bleName);
#endif
  oled_init();
  sonic_init();
  ble_init();
}

void loop() {
  if (millis() < nextPollingTime) { delay(nextPollingTime - millis()); }
  nextPollingTime = millis() + SAMPLING_RATE;
  // Poll the sonic anemometer
  digitalWrite(LED_GREEN, HIGH);
  sonic_get();
  digitalWrite(LED_GREEN, LOW);
  // Send BLE data
  if (Bluefruit.Central.connected()) {
    digitalWrite(LED_BLUE, HIGH);
    ble_send();
    delay(2);
    digitalWrite(LED_BLUE, LOW);
  }
  oled_update();
}

// DISPLAY
void oled_init(void) {
  u8g2.begin();
  u8g2.setContrast(255);  // Brightness range [0-255]
  u8g2.setFont(u8g2_font_9x15_tf);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 15, bleName);
  u8g2.sendBuffer();
}

void oled_update(void) {
  u8g2.clearBuffer();
  u8g2.drawStr(0, 15, bleName);
  memset(displayMsg, 0, sizeof(displayMsg));
  snprintf(displayMsg, sizeof(displayMsg), "Spd  %.2f m/s", windSpd);
  u8g2.drawStr(0, 30, displayMsg);
  memset(displayMsg, 0, sizeof(displayMsg));
  snprintf(displayMsg, sizeof(displayMsg), "Dir  %s", compass_direction(windDir));
  u8g2.drawStr(0, 45, displayMsg);
  memset(displayMsg, 0, sizeof(displayMsg));
  snprintf(displayMsg, sizeof(displayMsg), "Tmp  %.1fC", windTmp);
  u8g2.drawStr(0, 60, displayMsg);
  u8g2.sendBuffer();
}

const char* compass_direction(float heading) {
  const char* directions[] = {
    "N", "NE", "E", "SE",
    "S", "SW", "W", "NW"
  };
  return directions[(int)((heading + 22.5) / 45.0) % 8];
}

// Sonic (ATMOS22)
void sonic_init(void) {
  mySDI12.begin();
  delay(500);
  // Sensor identification
  mySDI12.sendCommand("0I!");
  String response = mySDI12.readStringUntil('\n');
  response.trim();
  nextPollingTime = millis() + SAMPLING_RATE;
#if DEBUG
  Serial.println(response);
#endif
}

void sonic_get(void) {
  mySDI12.clearBuffer();
  mySDI12.sendCommand("0R4!");
  String response = mySDI12.readStringUntil('\n');
  response.trim();
  response_handler(response);
#if DEBUG
  // Serial.print("R4! response: ");
  // Serial.println(response);
#endif
}

void response_handler(String response) {
  // aR4! response format:
  // a<TAB><NorthWindSpeed> <EastWindSpeed> <gustWindSpeed> <airTemperature> <xOrientation> <yOrientation> <nullValue><CR><sensortype><Checksum><CRC>
  //
  // Remove SDI-12 address
  response.remove(0, 1);
  // Tokenize on ' '
  char buffer[128];
  response.toCharArray(buffer, sizeof(buffer));
  char* token = strtok(buffer, " ");
  int field = 0;
  float nVector = 0.0;
  float eVector = 0.0;
  while (token != NULL) {
    switch (field) {
      case 0:
        nVector = atof(token);
        break;
      case 1:
        eVector = atof(token);
        break;
      case 2:
        // Gust speed not needed
        break;
      case 3:
        windTmp = atof(token);
        break;
    }
    field++;
    token = strtok(NULL, " ");
  }
  // Skip error codes or truncated data
  if (field < 4 || nVector <= -9990 || eVector <= -9990) { return; }
  // Calculate wind speed and heading using N/E vectors
  windSpd = sqrt(pow(nVector, 2) + pow(eVector, 2));
  windDir = atan2(eVector, nVector) * (180.0 / M_PI);
  if (windDir < 0) { windDir += 360.0; }
  if (windDir > 360) { windDir -= 360.0; }
#if DEBUG
  Serial.printf("Speed: %.2f m/s, Heading: %.1f, Temp: %.1f°C\n",
                windSpd, windDir, windTmp);
#endif
}

bool ble_init(void) {
  // Disable connection LED
  Bluefruit.autoConnLed(false);
  if (!Bluefruit.begin(0, MAX_SAMPLERS)) { return false; }
  Bluefruit.setTxPower(4);  // Check bluefruit.h for supported values
  Bluefruit.setName(bleName);
  Bluefruit.Central.setConnectCallback(central_connect_callback);
  Bluefruit.Central.setDisconnectCallback(central_disconnect_callback);
  for (uint8_t id = 0; id < MAX_SAMPLERS; id++) {
    memset(peripherals[id].name, 0, sizeof(peripherals[id].name));
    peripherals[id].conn_handle = BLE_CONN_HANDLE_INVALID;
    peripherals[id].clientUart.begin();
  }
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);
  return true;
}

void ble_send(void) {
  // Only send sonic data to the peripherals now; build out options as we go
  snprintf(bleMsg,
           sizeof(bleMsg),
           "%.2f,%.1f,%.1f",
           windSpd,
           windDir,
           windTmp);
  // Send data to all peripherals, let them decide whether or not to sample
  if (Bluefruit.Central.connected()) {
    for (uint8_t id = 0; id < MAX_SAMPLERS; id++) {
      Peripheral* peer = &peripherals[id];
      if (peer->clientUart.discovered()) {
        peer->clientUart.write(bleMsg);
      }
    }
  }
}

void scan_callback(ble_gap_evt_adv_report_t* report) {
#if DEBUG
  Serial.print("BLE UART service detected. Connecting ... ");
#endif
  Bluefruit.Central.connect(report);
}

void central_connect_callback(uint16_t conn_handle) {
  int id = findConnHandle(BLE_CONN_HANDLE_INVALID);
  if (id < 0) { return; }
  Peripheral* peer = &peripherals[id];
  peer->conn_handle = conn_handle;
  Bluefruit.Connection(conn_handle)->getPeerName(peer->name, sizeof(peer->name) - 1);
#if DEBUG
  Serial.printf("Connected to %s\n", peer->name);
#endif
  if (peer->clientUart.discover(conn_handle)) {
#if DEBUG
    Serial.println("Found it");
#endif
    peer->clientUart.enableTXD();
    connections++;
    if (connections < MAX_SAMPLERS) { Bluefruit.Scanner.start(0); }
  } else {
#if DEBUG
    Serial.println("Found NONE");
#endif
    Bluefruit.disconnect(conn_handle);
  }
}

void central_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  int id = findConnHandle(conn_handle);
  if (id < 0) { return; }
  memset(peripherals[id].name, 0, sizeof(peripherals[id].name));
  peripherals[id].conn_handle = BLE_CONN_HANDLE_INVALID;
  connections--;
  Bluefruit.Scanner.start(0);
#if DEBUG
  Serial.printf("%s disconnected, reason = %#X\n", peripherals[id].name, reason);
#endif
}

int findConnHandle(uint16_t conn_handle) {
  for (uint8_t id = 0; id < MAX_SAMPLERS; id++) {
    if (conn_handle == peripherals[id].conn_handle) { return id; }
  }
  return -1;
}
