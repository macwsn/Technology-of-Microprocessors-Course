#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

// Piny dla ILI9341
#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  -1

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// za malo ramu na stronie bylo zeby pociagnac
const int N = 40;
const int PIXEL_SIZE = 4;
const float DENSITY = 0.3;
const int NEIGHBOR_TYPE = 1;
byte image[N * N / 8 + 1]; 

// pamaretry wyzarznia
float temperature;
const float TEMP_START = 30.0;
const float COOLING_RATE = 0.99;
const float MIN_TEMP = 0.1;
const int ITER_PER_TEMP = 50;

// eneria
const float J_NEAR = -1.0;
const float J_FAR = 0.5;

bool getPixel(int x, int y) {
  int idx = y * N + x;
  int byteIdx = idx / 8;
  int bitIdx = idx % 8;
  return (image[byteIdx] & (1 << bitIdx)) != 0;
}

void setPixel(int x, int y, bool value) {
  int idx = y * N + x;
  int byteIdx = idx / 8;
  int bitIdx = idx % 8;
  if(value) {
    image[byteIdx] |= (1 << bitIdx);
  } else {
    image[byteIdx] &= ~(1 << bitIdx);
  }
}

void flipPixel(int x, int y) {
  int idx = y * N + x;
  int byteIdx = idx / 8;
  int bitIdx = idx % 8;
  image[byteIdx] ^= (1 << bitIdx);
}

void generateRandomImage() {
  for(int i = 0; i < N; i++) {
    for(int j = 0; j < N; j++) {
      setPixel(i, j, random(1000) < DENSITY * 1000);
    }
  }
}

// simplified energy calculations
float calculatePixelEnergy(int x, int y) {
  float energy = 0.0;
  int pixel = getPixel(x, y) ? 1 : -1;
  
  int maxR = (NEIGHBOR_TYPE == 2) ? 2 : 1;
  
  for(int dx = -maxR; dx <= maxR; dx++) {
    for(int dy = -maxR; dy <= maxR; dy++) {
      if(dx == 0 && dy == 0) continue;
      
      int nx = (x + dx + N) % N;
      int ny = (y + dy + N) % N;
      
      int dist = max(abs(dx), abs(dy));
      
      bool isNeighbor = false;
      if(NEIGHBOR_TYPE == 0) {
        isNeighbor = (abs(dx) + abs(dy) == 1);
      } else if(NEIGHBOR_TYPE == 1) {
        isNeighbor = (dist == 1);
      } else {
        isNeighbor = (dist <= 2);
      }
      
      if(!isNeighbor) continue;
      
      int neighbor = getPixel(nx, ny) ? 1 : -1;
      
      if(dist == 1) {
        energy += J_NEAR * pixel * neighbor;
      } else {
        energy += J_FAR * pixel * neighbor;
      }
    }
  }
  
  return energy;
}

// mownit w cy
void simulatedAnnealing() {
  temperature = TEMP_START;
  int iter = 0;
  
  while(temperature > MIN_TEMP) {
    for(int i = 0; i < ITER_PER_TEMP; i++) {
      int x = random(N);
      int y = random(N);
      
      float oldE = calculatePixelEnergy(x, y);
      flipPixel(x, y);
      float newE = calculatePixelEnergy(x, y);
      float deltaE = newE - oldE;
      
      if(deltaE >= 0 && random(1000) >= exp(-deltaE / temperature) * 1000) {
        flipPixel(x, y);
      }
      
      iter++;
    }
    
    temperature *= COOLING_RATE;
    
    if(iter % 200 == 0) {
      displayImage(20, 40);
      tft.fillRect(0, 220, 320, 20, ILI9341_NAVY);
      tft.setCursor(5, 225);
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(1);
      tft.print("T: ");
      tft.print(temperature, 1);
    }
  }
}

void displayImage(int startX, int startY) {
  for(int i = 0; i < N; i++) {
    for(int j = 0; j < N; j++) {
      uint16_t color = getPixel(i, j) ? ILI9341_BLACK : ILI9341_WHITE;
      tft.fillRect(startX + j * PIXEL_SIZE, startY + i * PIXEL_SIZE, 
                   PIXEL_SIZE, PIXEL_SIZE, color);
    }
  }
}

void setup() {
  Serial.begin(9600);
  
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  
  Serial.println(F("Symulowane Wyzarzanie"));
  Serial.print(F("RAM: "));
  Serial.print(N * N / 8);
  Serial.println(F(" bajtow"));
  
  randomSeed(analogRead(A0));
  
  tft.setCursor(30, 10);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.print(F("WYZARZANIE"));
  
  delay(1000);

  generateRandomImage();
  displayImage(20, 40);
  
  tft.setCursor(5, 225);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(1);
  tft.print(F("POCZATKOWY"));
  
  delay(2000);
  
  tft.fillRect(0, 220, 320, 20, ILI9341_RED);
  tft.setCursor(5, 225);
  tft.setTextColor(ILI9341_WHITE);
  tft.print(F("SYMULACJA..."));
  
  unsigned long t1 = millis();
  simulatedAnnealing();
  unsigned long t2 = millis();
  displayImage(20, 40);
  
  tft.fillRect(0, 220, 320, 20, ILI9341_GREEN);
  tft.setCursor(5, 225);
  tft.setTextColor(ILI9341_BLACK);
  tft.print(F("GOTOWE! "));
  tft.print((t2 - t1) / 1000);
  tft.print(F("s"));
  Serial.print(F("Czas: "));
  Serial.print((t2 - t1) / 1000.0);
  Serial.println(F("s"));
}

void loop() {
  delay(1000);
}