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

// defined pins
#define DATA_PIN_1 27
#define DATA_PIN_2 12
#define DATA_PIN_3 13
#define DATA_PIN_4 14

#define GRID_SIZE 32
#define MAX_RADIUS 10

CRGB leds[NUM_PANELS][NUM_LEDS_PER_PANEL];
CRGB inputImage[32][32];  // Virtual LED canvas

// ------------------- Panel Rotation Setup -------------------
enum Rotation { ROT_0, ROT_90, ROT_180, ROT_270 };
Rotation panelRotations[NUM_PANELS] = {ROT_0, ROT_0, ROT_0, ROT_0};  // default: no rotation

// ------------------- Logger -------------------
bool diagnosticLoggingEnabled = false;  // Set to false to disable logging
unsigned long lastDiagnosticLogTime = 0;
const unsigned long diagnosticInterval = 30000;  // 30 seconds

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
const int frameInterval = 30; // ms

// Secure client for TLS
WiFiClientSecure secureClient;
// MQTT client using the secure client
PubSubClient mqttClient(secureClient);

// ------------------- Function Declarations -------------------
void setup_wifi();
void reconnect();
void displayFrame();
void generateCalibrationImage(CRGB color = CRGB::White);
void callback(char* topic, byte* payload, unsigned int length);
void logDiagnostics();


// Your heatmap color palette
const CRGB heatPalette[] = {
  CRGB(0x10, 0x20, 0x80), // Deep Blue (cool base)
  CRGB(0x10, 0x90, 0xA0), // Soft Teal
  CRGB(0x20, 0xB0, 0x20), // Leafy Green
  CRGB(0xD0, 0xA0, 0x10), // Mellow Yellow
  CRGB(0xF0, 0x50, 0x00), // Burnt Orange (less green = less white)
  CRGB(0xC0, 0x00, 0x00)  // True Red (no green/blue to avoid pink)
};

const int heatPaletteSize = sizeof(heatPalette) / sizeof(CRGB);

// Decay map using Manhattan distance
const int decayMap[MAX_RADIUS + 1] = {100, 90, 75, 60, 45, 30, 18, 10, 5, 2, 1};

void setup() {
  Serial.begin(115200);
  delay(1000);
  // Initialize LEDs
  FastLED.addLeds<LED_TYPE, DATA_PIN_1, COLOR_ORDER>(leds[0], NUM_LEDS_PER_PANEL);
  FastLED.addLeds<LED_TYPE, DATA_PIN_2, COLOR_ORDER>(leds[1], NUM_LEDS_PER_PANEL);
  FastLED.addLeds<LED_TYPE, DATA_PIN_3, COLOR_ORDER>(leds[2], NUM_LEDS_PER_PANEL);
  FastLED.addLeds<LED_TYPE, DATA_PIN_4, COLOR_ORDER>(leds[3], NUM_LEDS_PER_PANEL);
  FastLED.setBrightness(2);

  setup_wifi();

  secureClient.setInsecure();  // Use only for testing if you don't validate certs
  

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);

  // Set up panel rotations
  panelRotations[0] = ROT_0;
  panelRotations[1] = ROT_270;
  panelRotations[2] = ROT_90;
  panelRotations[3] = ROT_180;
  generateCalibrationImage(CRGB::Red);  // Generate a calibration image
  // print out image for debugging
}

void loop() {

  unsigned long now = millis();
  if (now - lastFrameTime >= frameInterval) {
    lastFrameTime = now;
    displayFrame();
  }

  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();
  // Serial.println("Tick");
  if (diagnosticLoggingEnabled && millis() - lastDiagnosticLogTime > diagnosticInterval) {
    lastDiagnosticLogTime = millis();
    logDiagnostics();
  }
}

// ------------------- WiFi Connection -------------------
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

// ------------------- MQTT Reconnection -------------------
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

// ------------------- MQTT Callback -------------------
// NOTE: This is a placeholder. You’ll want to define your payload format.
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived: ");
  Serial.print("Length: ");
  Serial.print(length);
  Serial.println(topic);

  StaticJsonDocument<2048> doc;  // Reduce size if possible

  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }

  float heatGrid[GRID_SIZE][GRID_SIZE] = {0};

  // Process points directly during parsing
  for (JsonObject obj : doc.as<JsonArray>()) {
    // mqttClient.loop();
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

  // Generate LED colors from heatGrid (reuse your existing logic)
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
        inputImage[y][x] = heatPalette[colorIdx];
      } else {
        inputImage[y][x] = heatPalette[0]; // Coldest color
      }
    }
  }
}


void generateCalibrationImage(CRGB color) {
  Serial.println("Generating calibration image...");

  // Clear the entire 32x32 canvas
  for (int y = 0; y < 32; y++) {
    for (int x = 0; x < 32; x++) {
      inputImage[y][x] = CRGB::Black;
    }
  }

  // Define the center 16x16 square starting at (8, 8)
  const int centerStartX = 8;
  const int centerStartY = 8;

  // Fill each 8x8 quadrant of the 16x16 center with a unique color
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      inputImage[centerStartY + y][centerStartX + x] = CRGB::Red;        // Top-left (Panel 0)
      inputImage[centerStartY + y][centerStartX + x + 8] = CRGB::Yellow; // Top-right (Panel 1)
      inputImage[centerStartY + y + 8][centerStartX + x] = CRGB::Green;  // Bottom-left (Panel 2)
      inputImage[centerStartY + y + 8][centerStartX + x + 8] = CRGB::Blue; // Bottom-right (Panel 3)
    }
  }

  Serial.println("16x16 center block split into 4 colored quadrants.");
}

// ------------------- Frame Rendering -------------------
void displayFrame() {
  const int panelsPerRow = 2; // 2x2 grid

  for (int panel = 0; panel < NUM_PANELS; panel++) {
    int panelX = panel % panelsPerRow;           // 0 or 1
    int panelY = panel / panelsPerRow;           // 0 or 1

    for (int y = 0; y < PANEL_HEIGHT; y++) {
      for (int x = 0; x < PANEL_WIDTH; x++) {
        // Global coordinates in the full 32x32 image
        int globalX = panelX * PANEL_WIDTH + x;
        int globalY = panelY * PANEL_HEIGHT + y;

        // Apply per-panel rotation
        int rx = x, ry = y;
        switch (panelRotations[panel]) {
          case ROT_90:
            rx = PANEL_HEIGHT - 1 - y;
            ry = x;
            break;
          case ROT_180:
            rx = PANEL_WIDTH - 1 - x;
            ry = PANEL_HEIGHT - 1 - y;
            break;
          case ROT_270:
            rx = y;
            ry = PANEL_WIDTH - 1 - x;
            break;
          default:
            break;
        }

        int index = (ry % 2 == 0)
                      ? (ry * PANEL_WIDTH + rx)
                      : (ry * PANEL_WIDTH + (PANEL_WIDTH - 1 - rx));

        leds[panel][index] = inputImage[globalY][globalX];
      }
    }
  }

  FastLED.show();
}

// ------------------- Diagnostic Logging -------------------
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
