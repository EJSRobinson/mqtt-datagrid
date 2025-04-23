#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#define MQTT_MAX_PACKET_SIZE 2048
#include <PubSubClient.h>
#include <FastLED.h>
#include <vector>
#include <ArduinoJson.h>

// ------------------- LED Setup -------------------
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define NUM_PANELS 4
#define PANEL_WIDTH 16
#define PANEL_HEIGHT 16
#define NUM_LEDS_PER_PANEL (PANEL_WIDTH * PANEL_HEIGHT)

#define DATA_PIN_1 27
#define DATA_PIN_2 12
#define DATA_PIN_3 13
#define DATA_PIN_4 14

#define GRID_SIZE 32
#define MAX_RADIUS 10

CRGB leds[NUM_PANELS][NUM_LEDS_PER_PANEL];
CRGB previousImage[32][32];
CRGB targetImage[32][32];

// ------------------- Panel Rotation Setup -------------------
enum Rotation { ROT_0, ROT_90, ROT_180, ROT_270 };
Rotation panelRotations[NUM_PANELS] = {ROT_0, ROT_0, ROT_0, ROT_0};

// ------------------- Logger -------------------
bool diagnosticLoggingEnabled = false;
unsigned long lastDiagnosticLogTime = 0;
const unsigned long diagnosticInterval = 30000;

// ------------------- WiFi Setup -------------------
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// ------------------- MQTT Setup -------------------
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_topic = MQTT_TOPIC;
const char* mqtt_user = MQTT_USER;
const char* mqtt_pass = MQTT_PASS;

// -------------------- Frame Timing -------------------
unsigned long lastFrameTime = 0;
unsigned long lastMQTTUpdateTime = 0;
const int frameInterval = 30; // ms
const int mqttUpdateInterval = 1000; // ms (1 second expected)

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

void setup_wifi();
void reconnect();
void displayFrame();
void generateCalibrationImage(CRGB color = CRGB::White);
void callback(char* topic, byte* payload, unsigned int length);
void logDiagnostics();

const CRGB heatPalette[] = {
  CRGB(0x10, 0x20, 0x80),
  CRGB(0x10, 0x90, 0xA0),
  CRGB(0x20, 0xB0, 0x20),
  CRGB(0xD0, 0xA0, 0x10),
  CRGB(0xF0, 0x50, 0x00),
  CRGB(0xC0, 0x00, 0x00)
};
const int heatPaletteSize = sizeof(heatPalette) / sizeof(CRGB);
const int decayMap[MAX_RADIUS + 1] = {100, 90, 75, 60, 45, 30, 18, 10, 5, 2, 1};

void setup() {
  Serial.begin(115200);
  delay(1000);
  FastLED.addLeds<LED_TYPE, DATA_PIN_1, COLOR_ORDER>(leds[0], NUM_LEDS_PER_PANEL);
  FastLED.addLeds<LED_TYPE, DATA_PIN_2, COLOR_ORDER>(leds[1], NUM_LEDS_PER_PANEL);
  FastLED.addLeds<LED_TYPE, DATA_PIN_3, COLOR_ORDER>(leds[2], NUM_LEDS_PER_PANEL);
  FastLED.addLeds<LED_TYPE, DATA_PIN_4, COLOR_ORDER>(leds[3], NUM_LEDS_PER_PANEL);
  FastLED.setBrightness(5);

  setup_wifi();
  secureClient.setInsecure();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);

  panelRotations[0] = ROT_0;
  panelRotations[1] = ROT_270;
  panelRotations[2] = ROT_90;
  panelRotations[3] = ROT_180;
  generateCalibrationImage(CRGB::Red);
}

void loop() {
  unsigned long now = millis();
  if (now - lastFrameTime >= frameInterval) {
    lastFrameTime = now;
    displayFrame();
  }

  if (!mqttClient.connected()) reconnect();
  mqttClient.loop();

  if (diagnosticLoggingEnabled && now - lastDiagnosticLogTime > diagnosticInterval) {
    lastDiagnosticLogTime = now;
    logDiagnostics();
  }
}

void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      mqttClient.subscribe(mqtt_topic, 1);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived. Length: ");
  Serial.println(length);

  StaticJsonDocument<2048> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }

  memcpy(previousImage, targetImage, sizeof(targetImage));
  lastMQTTUpdateTime = millis();

  float heatGrid[GRID_SIZE][GRID_SIZE] = {0};
  for (JsonObject obj : doc.as<JsonArray>()) {
    int x = obj["pos_x"] | -1;
    int y = obj["pos_y"] | -1;
    bool isPerson = obj["is_person"] | false;
    if (x >= 0 && y >= 0 && x < GRID_SIZE && y < GRID_SIZE) {
      float intensity = isPerson ? 100.0 : 50.0;
      for (int dy = -MAX_RADIUS; dy <= MAX_RADIUS; dy++) {
        for (int dx = -MAX_RADIUS; dx <= MAX_RADIUS; dx++) {
          int dist = abs(dx) + abs(dy);
          if (dist > MAX_RADIUS) continue;
          int nx = x + dx;
          int ny = y + dy;
          if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE) {
            heatGrid[ny][nx] += intensity * decayMap[dist] / 100.0;
          }
        }
      }
    }
  }

  float maxHeat = 0;
  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      if (heatGrid[y][x] > maxHeat) maxHeat = heatGrid[y][x];

  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      if (heatGrid[y][x] > 0 && maxHeat > 0) {
        float norm = heatGrid[y][x] / maxHeat;
        int colorIdx = round(norm * (heatPaletteSize - 1));
        colorIdx = constrain(colorIdx, 0, heatPaletteSize - 1);
        targetImage[y][x] = heatPalette[colorIdx];
      } else {
        targetImage[y][x] = heatPalette[0];
      }
    }
  }
}

void generateCalibrationImage(CRGB color) {
  for (int y = 0; y < 32; y++)
    for (int x = 0; x < 32; x++)
      targetImage[y][x] = CRGB::Black;

  const int centerStartX = 8;
  const int centerStartY = 8;

  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      targetImage[centerStartY + y][centerStartX + x] = CRGB::Red;
      targetImage[centerStartY + y][centerStartX + x + 8] = CRGB::Yellow;
      targetImage[centerStartY + y + 8][centerStartX + x] = CRGB::Green;
      targetImage[centerStartY + y + 8][centerStartX + x + 8] = CRGB::Blue;
    }
  }
}

void displayFrame() {
  float t = min(1.0f, (float)(millis() - lastMQTTUpdateTime) / mqttUpdateInterval);

  CRGB interpolated[32][32];
  for (int y = 0; y < 32; y++) {
    for (int x = 0; x < 32; x++) {
      interpolated[y][x].r = previousImage[y][x].r + (targetImage[y][x].r - previousImage[y][x].r) * t;
      interpolated[y][x].g = previousImage[y][x].g + (targetImage[y][x].g - previousImage[y][x].g) * t;
      interpolated[y][x].b = previousImage[y][x].b + (targetImage[y][x].b - previousImage[y][x].b) * t;
    }
  }

  const int panelsPerRow = 2;
  for (int panel = 0; panel < NUM_PANELS; panel++) {
    int panelX = panel % panelsPerRow;
    int panelY = panel / panelsPerRow;
    for (int y = 0; y < PANEL_HEIGHT; y++) {
      for (int x = 0; x < PANEL_WIDTH; x++) {
        int globalX = panelX * PANEL_WIDTH + x;
        int globalY = panelY * PANEL_HEIGHT + y;
        int rx = x, ry = y;
        switch (panelRotations[panel]) {
          case ROT_90:  rx = PANEL_HEIGHT - 1 - y; ry = x; break;
          case ROT_180: rx = PANEL_WIDTH - 1 - x; ry = PANEL_HEIGHT - 1 - y; break;
          case ROT_270: rx = y; ry = PANEL_WIDTH - 1 - x; break;
          default: break;
        }
        int index = (ry % 2 == 0) ? (ry * PANEL_WIDTH + rx) : (ry * PANEL_WIDTH + (PANEL_WIDTH - 1 - rx));
        leds[panel][index] = interpolated[globalY][globalX];
      }
    }
  }
  FastLED.show();
}

void logDiagnostics() {
  Serial.println("=== Diagnostic Log ===");
  Serial.print("WiFi Status: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("MQTT State: ");
  Serial.println(mqttClient.state());
  Serial.print("Free Heap: ");
  Serial.println(ESP.getFreeHeap());
  Serial.println("======================\n");
}
