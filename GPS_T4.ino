#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_GFX.h>
#include <TinyGPS++.h>
#include <math.h>


// Definicja rozdzielczości ekranu
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320

TFT_eSPI tft = TFT_eSPI();
BH1750 lightMeter;
Adafruit_AHTX0 aht;
TinyGPSPlus gps;

const int BUTTON_LEFT = 23;
const int BUTTON_RIGHT = 1;
const int BUZZER_PIN = 32;
//const int BACKLIGHT_PIN = 27;

const int ledPin = 27;  // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 5000;
const int resolution = 8;

const int SDA_PIN = 22;
const int SCL_PIN = 21;

const int GPS_RX_PIN = 13;
const int GPS_TX_PIN = 12;

const int NUM_SCREENS = 15;
int currentScreen = 0;  // EKRAN NA START PROGRAMU !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
float lastLux = -1;
float lastTemp = -1000;
float lastHum = -1;
float globalLux = -1;
int currentBrightness = 600;
float gpsSpeed = 0.0;
int gpsHour = 0, gpsMinute = 0, gpsSecond = 0;
int gpsDay = 0, gpsMonth = 0, gpsYear = 0;

int satellites = 0;
float totalDistance = 0.0;
float dailyDistance = 0.0;
float lastValidLatitude = 0.0;
float lastValidLongitude = 0.0;
bool isFirstValidLocation = true;
int lastResetDay = -1;

unsigned long screen9StartTime = 0;                // Czas, kiedy ekran 9 został aktywowany
const unsigned long SCREENSAVER_TIMEOUT = 120000;  // 2 minuty (120 000 ms)
bool isScreenSaverActive = false;                  // Flaga, czy wygaszacz jest aktywny


const unsigned long BACKLIGHT_UPDATE_INTERVAL = 100;  // Aktualizacja co 100ms
const int MAX_CHANGE_PER_SECOND = 1;
const int MAX_CHANGE_PER_UPDATE = MAX_CHANGE_PER_SECOND * BACKLIGHT_UPDATE_INTERVAL / 1000;

unsigned long lastBacklightUpdate = 0;


unsigned long drivingTime = 0;      // Czas jazdy w milisekundach
unsigned long stopTime = 0;         // Czas postoju w milisekundach
unsigned long lastSpeedUpdate = 0;  // Ostatni czas aktualizacji prędkości

unsigned long drivingTime2 = 0;  // Czas jazdy w milisekundach dla drugiego stopera
unsigned long stopTime2 = 0;     // Czas postoju w milisekundach dla drugiego stopera
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int SAT_ICON_X = 370;
const int SAT_ICON_Y = 250;
const int SAT_ICON_SIZE = 20;
const int BAR_WIDTH = 3;
const int BAR_GAP = 2;
const int MAX_BARS = 10;

#define BUFFER_SIZE 100

float speedBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int validReadings = 0;
float totalSpeed = 0;

float maxSpeed = 0.0;
float averageSpeed = 0.0;
float totalSpeedReadings = 0;
int speedReadingsCount = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long lastUpdateTime = 0;
unsigned long updateInterval = 100;  // Aktualizuj co 100ms
int brightnessStep = 1;              // Krok zmiany jasności
int targetBrightness = 0;            // Krok zmiany jasności
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int CENTER_X = 240;         // Center of the screen horizontally
const int CENTER_Y = 160;         // Center of the screen vertically
const int RADIUS = 140;           // Radius of the speedometer
const float START_ANGLE = 150.0;  // Starting angle for the arc (in degrees)
const float END_ANGLE = 30.0;     // Ending angle for the arc (in degrees)
const int MAX_SPEED = 80;         // Maximum speed on the speedometer
float previousAngle = START_ANGLE;
float previousSpeed = -1;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned long lastUserInteractionTime = 0;
const unsigned long USER_INTERACTION_TIMEOUT = 30000;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define CENTER_X (SCREEN_WIDTH / 2)
#define CENTER_Y (SCREEN_HEIGHT / 2)

////////////////////////////////////////////////20.10.2024/////////////////////////////////////////////////////////////////////
// Struktura dla strefy GPS
struct GPSZone {
  const char* name;
  float latitude;
  float longitude;
  float radius;  // w metrach
};

// Tablica zdefiniowanych stref
const GPSZone zones[] = {
  { "WARYNSKIEGO", 50.30951909501132, 18.615725105277633, 100 },
  { "KOPERNIKA", 50.321340096774456, 18.655650256711283, 200 },
  { "WIKA", 50.32171260000084, 18.62055068085404, 100 },
  { "PSP GLIWICE", 50.292725154944925, 18.674098859623378, 50 },
  { "TOMEK WOJTULA", 50.285687757755916, 18.626288498823932, 50 },

  // Dodaj więcej stref według potrzeb
};

const int NUM_ZONES = sizeof(zones) / sizeof(zones[0]);

// Funkcja do sprawdzania, w której strefie znajduje się urządzenie
int checkCurrentZone(float currentLat, float currentLon) {
  for (int i = 0; i < NUM_ZONES; i++) {
    float distance = calculateDistance(currentLat, currentLon, zones[i].latitude, zones[i].longitude);
    if (distance <= zones[i].radius / 1000.0) {  // Konwersja promienia na kilometry
      return i;                                  // Zwróć indeks strefy
    }
  }
  return -1;  // Poza wszystkimi strefami
}
//////////////////////////////////////////20/10/2024 v2///////////////////////////////////////////////
struct ZoneTimeTracking {
  unsigned long totalTime;      // Całkowity czas spędzony w strefie (w sekundach)
  unsigned long lastEntryTime;  // Czas ostatniego wejścia do strefy (sekundy od północy)
  bool isInside;                // Flaga wskazująca, czy aktualnie jesteśmy w strefie
};



ZoneTimeTracking zoneTimeTrack[NUM_ZONES + 1];
int currentZoneIndex = -1;  // Indeks aktualnej strefy, -1 oznacza brak strefy

// Funkcja do uzyskiwania aktualnego czasu GPS (sekundy od północy)
unsigned long getGPSTime() {
  if (gps.time.isValid()) {
    int adjustedHour = (gps.time.hour() + 2) % 24;  // Dodaj 2 godziny i zapewnij, że mieści się w zakresie 0-23
    return adjustedHour * 3600 + gps.time.minute() * 60 + gps.time.second();
  }
  return 0;  // Zwróć 0 jeśli czas GPS jest nieprawidłowy
}

void resetZoneCounters() {
  // Iterujemy przez wszystkie strefy plus strefę "Poza strefami" (NUM_ZONES + 1)
  for (int i = 0; i <= NUM_ZONES; i++) {
    // Zerujemy całkowity czas spędzony w strefie
    zoneTimeTrack[i].totalTime = 0;
    // Zerujemy czas ostatniego wejścia do strefy
    zoneTimeTrack[i].lastEntryTime = 0;
    // Wyłączamy flagę obecności w strefie
    zoneTimeTrack[i].isInside = false;
  }
  // Resetujemy indeks aktualnej strefy na -1 (poza wszystkimi strefami)
  currentZoneIndex = -1;
}

const int RESET_HOUR = 23;
const int RESET_MINUTE = 59;
const int RESET_SECOND = 59;
bool wasReset = false;  // Flaga zabezpieczająca przed wielokrotnym resetem

void checkAndResetCounters() {
  if (gps.time.isValid()) {
    int currentHour = (gps.time.hour() + 2) % 24;  // Uwzględniamy strefę czasową +2
    int currentMinute = gps.time.minute();
    int currentSecond = gps.time.second();

    // Sprawdzamy czy jest czas na reset
    if (currentHour == RESET_HOUR && currentMinute == RESET_MINUTE && currentSecond == RESET_SECOND) {

      if (!wasReset) {        // Sprawdzamy czy reset już nie nastąpił
        resetZoneCounters();  // Wywołujemy reset
        wasReset = true;      // Ustawiamy flagę
                              // Tutaj możesz dodać np. zapis danych przed resetem
      }
    } else {
      wasReset = false;  // Resetujemy flagę jeśli nie jest czas resetu
    }
  }
}

// Zmodyfikowana funkcja do aktualizacji czasu spędzonego w strefach
void updateZoneTime() {
  if (gps.location.isValid() && gps.time.isValid()) {
    float currentLat = gps.location.lat();
    float currentLon = gps.location.lng();
    unsigned long currentTime = getGPSTime();

    int newZoneIndex = checkCurrentZone(currentLat, currentLon);

    if (newZoneIndex != currentZoneIndex) {
      // Wyjście z poprzedniej strefy
      if (currentZoneIndex != -1) {
        unsigned long timeSpent = currentTime - zoneTimeTrack[currentZoneIndex].lastEntryTime;
        zoneTimeTrack[currentZoneIndex].totalTime += timeSpent;
        zoneTimeTrack[currentZoneIndex].isInside = false;
      } else {
        // Wyjście ze stanu "poza strefą"
        unsigned long timeSpent = currentTime - zoneTimeTrack[NUM_ZONES].lastEntryTime;
        zoneTimeTrack[NUM_ZONES].totalTime += timeSpent;
      }

      // Wejście do nowej strefy lub poza strefę
      if (newZoneIndex != -1) {
        zoneTimeTrack[newZoneIndex].lastEntryTime = currentTime;
        zoneTimeTrack[newZoneIndex].isInside = true;
      } else {
        // Wejście w stan "poza strefą"
        zoneTimeTrack[NUM_ZONES].lastEntryTime = currentTime;
      }

      currentZoneIndex = newZoneIndex;
    } else {
      // Aktualizacja czasu w aktualnej strefie lub poza strefą
      if (currentZoneIndex != -1) {
        unsigned long timeSpent = currentTime - zoneTimeTrack[currentZoneIndex].lastEntryTime;
        zoneTimeTrack[currentZoneIndex].totalTime += timeSpent;
        zoneTimeTrack[currentZoneIndex].lastEntryTime = currentTime;
      } else {
        // Aktualizacja czasu poza strefą
        unsigned long timeSpent = currentTime - zoneTimeTrack[NUM_ZONES].lastEntryTime;
        zoneTimeTrack[NUM_ZONES].totalTime += timeSpent;
        zoneTimeTrack[NUM_ZONES].lastEntryTime = currentTime;
      }
    }
  }
}


// Funkcja do formatowania czasu
String
formatTime(unsigned long totalSeconds) {
  unsigned long hours = totalSeconds / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  unsigned long seconds = totalSeconds % 60;
  char timeStr[20];
  sprintf(timeStr, "%02lu:%02lu:%02lu", hours, minutes, seconds);
  return String(timeStr);
}
/////////////////////////////////////////////REJESTRATOR TRAC KIEDYS CASE 14////////////////////////////////////////////////////////////////////////
// Struktury danych
struct Trip {
  unsigned long startTime;
  unsigned long endTime;
  float distance;
  bool isActive;
  float maxSpeed;
  unsigned long movementStartTime;
};

struct CompletedTrip {
  float distance;
  unsigned long duration;
  float maxSpeed;
};

// Stałe
const int MAX_TRIPS = 10;
const unsigned long MOVEMENT_THRESHOLD = 10000;  // 10 sekund
const unsigned long STOP_THRESHOLD = 600000;     // 10 minut

// Zmienne globalne
Trip currentTrip;
CompletedTrip completedTrips[MAX_TRIPS];
int completedTripCount = 0;
unsigned long lastMovementTime = 0;
float lastTripValidLat = 0;
float lastTripValidLon = 0;
float totalTripDistance = 0;
int lastTripResetDay = -1;

// Funkcja do sprawdzania resetu o północy
void checkTripMidnightReset() {
  if (gps.time.isValid() && gps.date.isValid()) {
    int currentDay = gps.date.day();
    int currentHour = (gps.time.hour() + 2) % 24;
    int currentMinute = gps.time.minute();
    int currentSecond = gps.time.second();

    if (currentHour == 0 && currentMinute == 0 && currentSecond == 0 && currentDay != lastTripResetDay) {
      completedTripCount = 0;
      totalTripDistance = 0;
      lastTripResetDay = currentDay;
    }
  }
}

// Funkcja do aktualizacji podróży
void updateTripTracking() {
  unsigned long currentTime = millis();
  float speed = gps.speed.kmph();

  if (gps.location.isValid()) {
    float currentLat = gps.location.lat();
    float currentLon = gps.location.lng();

    if (speed > 5.0) {
      if (!currentTrip.isActive) {
        if (currentTrip.movementStartTime == 0) {
          currentTrip.movementStartTime = currentTime;
        } else if (currentTime - currentTrip.movementStartTime >= MOVEMENT_THRESHOLD) {
          currentTrip.startTime = currentTime;
          currentTrip.distance = 0;
          currentTrip.isActive = true;
          currentTrip.maxSpeed = speed;
          lastTripValidLat = currentLat;
          lastTripValidLon = currentLon;
        }
      } else {
        float tripDistance = TinyGPSPlus::distanceBetween(
                               lastTripValidLat, lastTripValidLon,
                               currentLat, currentLon)
                             / 1000.0;
        currentTrip.distance += tripDistance;
        totalTripDistance += tripDistance;
        if (speed > currentTrip.maxSpeed) {
          currentTrip.maxSpeed = speed;
        }
        lastTripValidLat = currentLat;
        lastTripValidLon = currentLon;
      }
      lastMovementTime = currentTime;
    } else {
      if (currentTrip.isActive && (currentTime - lastMovementTime > STOP_THRESHOLD)) {
        currentTrip.isActive = false;
        currentTrip.endTime = currentTime;
        currentTrip.movementStartTime = 0;

        if (completedTripCount < MAX_TRIPS) {
          completedTrips[completedTripCount].distance = currentTrip.distance;
          completedTrips[completedTripCount].duration = currentTrip.endTime - currentTrip.startTime;
          completedTrips[completedTripCount].maxSpeed = currentTrip.maxSpeed;
          completedTripCount++;
        } else {
          // Przesuń historię
          for (int i = 0; i < MAX_TRIPS - 1; i++) {
            completedTrips[i] = completedTrips[i + 1];
          }
          completedTrips[MAX_TRIPS - 1].distance = currentTrip.distance;
          completedTrips[MAX_TRIPS - 1].duration = currentTrip.endTime - currentTrip.startTime;
          completedTrips[MAX_TRIPS - 1].maxSpeed = currentTrip.maxSpeed;
        }
      } else if (!currentTrip.isActive) {
        currentTrip.movementStartTime = 0;
      }
    }
  }

  checkTripMidnightReset();
}

// Funkcja do rysowania interfejsu podróży
void drawTripInterface(TFT_eSPI& tft) {
  unsigned long currentTime = millis();

  // Rysuj statyczne etykiety
  const char* labels[] = {
    "Dystans:", "km", "Czas:", "Pozost:", "Max:", "km/h", "Akt:",
    "Historia podrozy:", "Całkowity dystans:", "Czas do resetu:"
  };
  const int positions[][2] = {
    { 10, 40 }, { 230, 40 }, { 10, 70 }, { 10, 100 }, { 10, 130 }, { 230, 130 }, { 290, 130 }, { 10, 160 }, { 10, 370 }, { 10, 10 }
  };

  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);

  for (int i = 0; i < 10; i++) {
    tft.drawString(labels[i], positions[i][0], positions[i][1]);
  }

  // Aktualizacja danych
  tft.setTextPadding(120);
  tft.drawFloat(currentTrip.distance, 3, 110, 40);

  // Czas podróży
  unsigned long duration = currentTrip.isActive ? (currentTime - currentTrip.startTime) : 0;
  int hours = duration / 3600000;
  int minutes = (duration % 3600000) / 60000;
  int seconds = (duration % 60000) / 1000;
  tft.setTextPadding(100);
  tft.drawString(
    String(hours) + ":" + (minutes < 10 ? "0" : "") + String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds),
    130, 70);

  // Pozostały czas
  tft.setTextPadding(120);
  if (currentTrip.isActive) {
    unsigned long elapsedSinceLastMovement = currentTime - lastMovementTime;
    if (elapsedSinceLastMovement < STOP_THRESHOLD) {
      unsigned long remainingStopTime = STOP_THRESHOLD - elapsedSinceLastMovement;
      int stopMinutes = remainingStopTime / 60000;
      int stopSeconds = (remainingStopTime % 60000) / 1000;
      tft.drawString(
        String(stopMinutes) + ":" + (stopSeconds < 10 ? "0" : "") + String(stopSeconds),
        130, 100);
    } else {
      tft.drawString("00:00", 130, 100);
    }
  } else {
    tft.drawString("--:--", 130, 100);
  }

  // Prędkości
  tft.setTextPadding(100);
  tft.drawFloat(currentTrip.maxSpeed, 1, 70, 130);
  tft.drawFloat(gps.speed.kmph(), 2, 360, 130);

  // Całkowity dystans
  tft.drawFloat(totalTripDistance, 3, 220, 370);

  // Czas do resetu
  if (gps.time.isValid()) {
    int adjustedHour = (gps.time.hour() + 2) % 24;
    int timeToReset = (24 * 60 * 60) - (adjustedHour * 3600 + gps.time.minute() * 60 + gps.time.second());
    int resetHours = timeToReset / 3600;
    int resetMinutes = (timeToReset % 3600) / 60;
    int resetSeconds = timeToReset % 60;
    tft.drawString(
      String(resetHours) + ":" + (resetMinutes < 10 ? "0" : "") + String(resetMinutes) + ":" + (resetSeconds < 10 ? "0" : "") + String(resetSeconds),
      190, 10);
  }

  // Historia podróży
  tft.setTextPadding(0);
  int displayedTrips = min(5, completedTripCount);
  for (int i = 0; i < displayedTrips; i++) {
    int tripIndex = completedTripCount - 1 - i;
    int yPos = 190 + i * 25;
    tft.setCursor(10, yPos);
    tft.printf("%d. %.3f km // %02d:%02d:%02d // %.0f km/h",
               completedTripCount - tripIndex,
               completedTrips[tripIndex].distance,
               (int)(completedTrips[tripIndex].duration / 3600000),
               (int)((completedTrips[tripIndex].duration % 3600000) / 60000),
               (int)((completedTrips[tripIndex].duration % 60000) / 1000),
               completedTrips[tripIndex].maxSpeed);
  }

  // Wskaźnik aktywnej podróży
  static bool lastTripActive = !currentTrip.isActive;
  if (currentTrip.isActive != lastTripActive) {
    tft.fillCircle(400, 15, 10,
                   currentTrip.isActive ? TFT_RED : TFT_GREEN);
    lastTripActive = currentTrip.isActive;
  }
}
/////////////////////////////////////////WYKRESY GODZINOWE OBLICZENIA //////////////////////////////////////////////////////////////////////////
// Stałe dla wykresu godzinowego
const float MIN_DISTANCE = 0.0;
const float MAX_DISTANCE = 50.0;
const float MIN_SPEED = 3.0;

// Struktura do przechowywania danych godzinowych
// Struktura do przechowywania danych godzinowych
struct HourlyData {
  float hourlyDistance[24];   // Przechowuje dystans dla każdej godziny
  float currentHourDistance;  // Dystans w bieżącej godzinie
  float lastLatitude;         // Ostatnia zapisana szerokość geograficzna
  float lastLongitude;        // Ostatnia zapisana długość geograficzna
  int currentHour;            // Bieżąca godzina
  int lastDay;                // Ostatni zapisany dzień
  float maxDistance;          // Maksymalny dystans w jednej godzinie
  bool isFirstPosition;       // Flaga pierwszej pozycji w godzinie
};

// Globalna instancja
HourlyData hourlyData = {
  .hourlyDistance = { 0 },
  .currentHourDistance = 0,
  .lastLatitude = 0,
  .lastLongitude = 0,
  .currentHour = -1,
  .lastDay = -1,
  .maxDistance = 0,
  .isFirstPosition = true
};

// Funkcja do aktualizacji danych godzinowych
void updateHourlyDistance() {
  if (!gps.location.isValid() || !gps.time.isValid()) {
    return;
  }

  int currentHour = (gps.time.hour() + 2) % 24;
  int currentDay = gps.date.day();
  float speed = gps.speed.kmph();
  float currentLat = gps.location.lat();
  float currentLon = gps.location.lng();

  // Reset danych przy zmianie dnia
  if (currentDay != hourlyData.lastDay) {
    memset(hourlyData.hourlyDistance, 0, sizeof(hourlyData.hourlyDistance));
    hourlyData.currentHourDistance = 0;
    hourlyData.maxDistance = 0;
    hourlyData.lastDay = currentDay;
    hourlyData.isFirstPosition = true;
  }

  // Obsługa zmiany godziny
  if (currentHour != hourlyData.currentHour) {
    if (hourlyData.currentHour != -1) {
      // Zapisz dystans z poprzedniej godziny
      hourlyData.hourlyDistance[hourlyData.currentHour] = hourlyData.currentHourDistance;
      if (hourlyData.currentHourDistance > hourlyData.maxDistance) {
        hourlyData.maxDistance = hourlyData.currentHourDistance;
      }
    }
    // Rozpocznij nową godzinę
    hourlyData.currentHour = currentHour;
    hourlyData.currentHourDistance = 0;
    hourlyData.isFirstPosition = true;
  }

  // Obliczanie dystansu tylko gdy prędkość > 3 km/h
  if (speed > 3.0) {
    if (!hourlyData.isFirstPosition) {
      float distance = TinyGPSPlus::distanceBetween(
                         hourlyData.lastLatitude,
                         hourlyData.lastLongitude,
                         currentLat,
                         currentLon)
                       / 1000.0;  // Konwersja na kilometry

      hourlyData.currentHourDistance += distance;
      hourlyData.hourlyDistance[currentHour] = hourlyData.currentHourDistance;
    }

    hourlyData.lastLatitude = currentLat;
    hourlyData.lastLongitude = currentLon;
    hourlyData.isFirstPosition = false;
  }
}

// Funkcja pomocnicza do pobierania dystansu dla konkretnej godziny
float getHourlyDistance(int hour) {
  if (hour == hourlyData.currentHour) {
    return hourlyData.currentHourDistance;
  }
  return hourlyData.hourlyDistance[hour];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Location {
  float latitude;
  float longitude;
  String name;  // Możesz dodać nazwę lokalizacji
};

// Przykładowe lokalizacje
Location locations[] = {
  { 50.32141172624091, 18.65574767867364, "Kopernika:" },
  { 50.309192057536116, 18.61546224134024, "Warynskiego:" },
  { 50.321630074497044, 18.620567826134533, "Wika:" },
  { 54.3520252, 18.6466392, "GdaNsk" },
  { 50.29271402249626, 18.67417349192423, "PSP" },
  { 50.247977220050544, 18.83659086808358, "Martin" }
};

const float EARTH_RADIUS = 6371.0;  // Promień Ziemi w kilometrach

float haversine(float lat1, float lon1, float lat2, float lon2) {
  float dLat = (lat2 - lat1) * (M_PI / 180.0);
  float dLon = (lon2 - lon1) * (M_PI / 180.0);

  lat1 = lat1 * (M_PI / 180.0);
  lat2 = lat2 * (M_PI / 180.0);

  float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return EARTH_RADIUS * c;  // Odległość w kilometrach
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

void updateDistance() {
  if (gps.location.isValid() && gps.speed.kmph() > 3.0) {
    float currentLat = gps.location.lat();
    float currentLon = gps.location.lng();

    if (!isFirstValidLocation) {
      float distance = calculateDistance(lastValidLatitude, lastValidLongitude, currentLat, currentLon);
      totalDistance += distance;
      dailyDistance += distance;
    } else {
      isFirstValidLocation = false;
    }

    lastValidLatitude = currentLat;
    lastValidLongitude = currentLon;
  }
}


void drawSignalBars(int x, int y, int numSatellites) {
  // Ustalamy wysokość kresek
  int barHeight = (SAT_ICON_SIZE * 2) / MAX_BARS;

  // Rysujemy kreski
  for (int i = 0; i < MAX_BARS; i++) {
    if (i < numSatellites) {  // Jeżeli i jest mniejsze niż liczba satelitów
      tft.fillRect(x + i * (BAR_WIDTH * 2 + BAR_GAP),
                   y + (SAT_ICON_SIZE * 2) - (i + 1) * barHeight,
                   BAR_WIDTH * 2,
                   (i + 1) * barHeight,
                   TFT_BLACK);  // Wypełniamy kreskę
    } else {
      tft.fillRect(x + i * (BAR_WIDTH * 2 + BAR_GAP),
                   y + (SAT_ICON_SIZE * 2) - (i + 1) * barHeight,
                   BAR_WIDTH * 2,
                   (i + 1) * barHeight,
                   TFT_WHITE);  // Wypełniamy kreskę kolorem tła, aby ją wyczyścić
      tft.drawRect(x + i * (BAR_WIDTH * 2 + BAR_GAP),
                   y + (SAT_ICON_SIZE * 2) - (i + 1) * barHeight,
                   BAR_WIDTH * 2,
                   (i + 1) * barHeight,
                   TFT_BLACK);  // Rysujemy kontur kreski
    }
  }
}




void updateAverageSpeed() {
  if (gps.speed.isValid()) {
    float currentSpeed = gps.speed.kmph();

    // Odejmij najstarszą wartość z sumy, jeśli bufor jest pełny
    if (validReadings == BUFFER_SIZE) {
      totalSpeed -= speedBuffer[bufferIndex];
    } else {
      validReadings++;
    }

    // Dodaj nową wartość do bufora i sumy
    speedBuffer[bufferIndex] = currentSpeed;
    totalSpeed += currentSpeed;

    // Przesuń indeks bufora
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    // Oblicz średnią prędkość
    averageSpeed = totalSpeed / validReadings;
  }
}

void updateMaxSpeed() {
  if (gps.speed.isValid() && gps.speed.kmph() > maxSpeed) {
    maxSpeed = gps.speed.kmph();
  }
}

float calculateAcceleration() {
  static float lastSpeed = 0;
  static unsigned long lastTime = 0;
  float acceleration = 0;

  if (gps.speed.isValid() && gps.time.isValid()) {
    unsigned long currentTime = gps.time.value();
    float currentSpeed = gps.speed.mps();

    if (lastTime != 0) {
      float timeDiff = (currentTime - lastTime) / 1000.0;  // konwersja na sekundy
      acceleration = (currentSpeed - lastSpeed) / timeDiff;
    }

    lastSpeed = currentSpeed;
    lastTime = currentTime;
  }

  return acceleration;  // m/s^2
}



float calculateSlope() {
  static float lastAltitude = 0;
  static float lastLatitude = 0;
  static float lastLongitude = 0;
  static float slopeBuffer[5] = { 0 };
  static int bufferIndex = 0;
  static unsigned long lastUpdateTime = 0;
  const float MIN_DISTANCE = 5.0;                  // Minimalna odległość w metrach
  const unsigned long MIN_UPDATE_INTERVAL = 1000;  // Minimalny interwał aktualizacji w ms

  if (gps.location.isValid() && gps.altitude.isValid()) {
    float currentAltitude = gps.altitude.meters();
    float currentLat = gps.location.lat();
    float currentLon = gps.location.lng();
    unsigned long currentTime = millis();

    // Sprawdź, czy minął minimalny interwał czasu
    if (currentTime - lastUpdateTime < MIN_UPDATE_INTERVAL) {
      return 0;  // Zwróć 0, jeśli nie minął wystarczający czas
    }

    float distance = TinyGPSPlus::distanceBetween(lastLatitude, lastLongitude, currentLat, currentLon);

    // Oblicz nachylenie tylko jeśli przebyta odległość jest wystarczająca
    if (distance >= MIN_DISTANCE) {
      float altitudeDiff = currentAltitude - lastAltitude;
      float slope = (altitudeDiff / distance) * 100;  // w procentach

      // Dodaj do bufora kołowego
      slopeBuffer[bufferIndex] = slope;
      bufferIndex = (bufferIndex + 1) % 5;

      // Oblicz średnią z bufora
      float averageSlope = 0;
      for (int i = 0; i < 5; i++) {
        averageSlope += slopeBuffer[i];
      }
      averageSlope /= 5;

      // Aktualizuj ostatnie wartości
      lastAltitude = currentAltitude;
      lastLatitude = currentLat;
      lastLongitude = currentLon;
      lastUpdateTime = currentTime;

      return averageSlope;
    }
  }

  return 0;  // Zwróć 0, jeśli warunki nie są spełnione
}

String calculateETA(float destinationLat, float destinationLon) {
  if (gps.location.isValid() && gps.speed.isValid()) {
    float distanceToDestination =
      TinyGPSPlus::distanceBetween(
        gps.location.lat(),
        gps.location.lng(),
        destinationLat,
        destinationLon);

    float speed = gps.speed.kmph();
    if (speed > 0) {
      float timeToArrival = distanceToDestination / speed;  // w godzinach
      int hours = (int)timeToArrival;
      int minutes = (int)((timeToArrival - hours) * 60);

      char eta[10];
      sprintf(eta, "%02d:%02d", hours, minutes);
      return String(eta);
    }
  }
  return "N/A";
}


void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  lightMeter.begin();
  aht.begin();

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);

  ledcAttach(ledPin, freq, resolution);

  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  //pinMode(BACKLIGHT_PIN, OUTPUT);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  updateDisplay(true);
}

void updateGPSData() {
  unsigned long currentTime = millis();

  if (gps.speed.isUpdated()) {
    gpsSpeed = gps.speed.kmph();
    updateDistance();

    // Sprawdź prędkość dla obu stoperów
    if (gpsSpeed < 3.0) {
      stopTime += (currentTime - lastSpeedUpdate);   // Zaktualizuj czas postoju dla pierwszego stopera
      stopTime2 += (currentTime - lastSpeedUpdate);  // Zaktualizuj czas postoju dla drugiego stopera
    } else {
      drivingTime += (currentTime - lastSpeedUpdate);   // Zaktualizuj czas jazdy dla pierwszego stopera
      drivingTime2 += (currentTime - lastSpeedUpdate);  // Zaktualizuj czas jazdy dla drugiego stopera
    }
    lastSpeedUpdate = currentTime;  // Zaktualizuj czas ostatniej aktualizacji prędkości
  }

  if (gps.time.isValid() && gps.date.isValid()) {
    gpsHour = (gps.time.hour() + 2) % 24;
    gpsMinute = gps.time.minute();
    gpsSecond = gps.time.second();
    gpsDay = gps.date.day();
    gpsMonth = gps.date.month();
    gpsYear = gps.date.year();

    // Reset dzienny dla obu stoperów
    if (gpsHour == 23 && gpsMinute == 59 && gpsSecond == 59) {
      if (gpsDay != lastResetDay) {
        dailyDistance = 0.0;
        //drivingTime = 0;   // Reset czasu jazdy dla pierwszego stopera
        //stopTime = 0;      // Reset czasu postoju dla pierwszego stopera
        drivingTime2 = 0;  // Reset czasu jazdy dla drugiego stopera
        stopTime2 = 0;     // Reset czasu postoju dla drugiego stopera
        lastResetDay = gpsDay;
      }
    }
  }

  if (gps.satellites.isUpdated()) {
    satellites = gps.satellites.value();
  }
}



void loop() {
  unsigned long currentTime = millis();


  updateTripTracking();

  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      updateGPSData();
      updateAverageSpeed();
      updateHourlyDistance();  // Dodaj tę linię
      updateMaxSpeed();
      updateZoneTime();
      checkAndResetCounters();

      // Dodaj to:
      if (gps.location.isValid()) {
        int newZoneIndex = checkCurrentZone(gps.location.lat(), gps.location.lng());
        if (newZoneIndex != currentZoneIndex) {
          // Strefa się zmieniła
          if (currentZoneIndex != -1) {
            // Wyjście ze strefy
            zoneTimeTrack[currentZoneIndex].isInside = false;
            // Zaktualizuj czas spędzony w poprzedniej strefie
          }
          if (newZoneIndex != -1) {
            // Wejście do nowej strefy
            zoneTimeTrack[newZoneIndex].isInside = true;
            zoneTimeTrack[newZoneIndex].lastEntryTime = getGPSTime();
          }
          currentZoneIndex = newZoneIndex;

          // Debug: wydrukuj informację o zmianie strefy
          Serial.print("Zmiana strefy na: ");
          Serial.println(currentZoneIndex == -1 ? "Poza strefą" : zones[currentZoneIndex].name);
        }
      }
    }
  }

  if (digitalRead(BUTTON_LEFT) == LOW || digitalRead(BUTTON_RIGHT) == LOW) {
    lastUserInteractionTime = currentTime;

    if (digitalRead(BUTTON_LEFT) == LOW) {
      currentScreen = (currentScreen - 1 + NUM_SCREENS) % NUM_SCREENS;
      updateDisplay(true);
      beep();
      delay(200);
    }

    if (digitalRead(BUTTON_RIGHT) == LOW) {
      currentScreen = (currentScreen + 1) % NUM_SCREENS;
      updateDisplay(true);
      beep();
      delay(200);
    }
    if (isScreenSaverActive) {
      isScreenSaverActive = false;
      currentScreen = 0;  // Domyślnie wróć na ekran 0
      updateDisplay(true);
    }
  }

  if (currentScreen == 9) {
    if (screen9StartTime == 0) {
      screen9StartTime = currentTime;  // Zapisz czas, kiedy ekran 9 został aktywowany
    }

    // Jeśli minęły 2 minuty, przejdź do wygaszacza (ekran 8)
    if (currentTime - screen9StartTime > SCREENSAVER_TIMEOUT) {
      currentScreen = 8;           // Przełącz na ekran wygaszacza
      isScreenSaverActive = true;  // Ustaw flagę, że wygaszacz jest aktywny
      updateDisplay(true);         // Aktualizuj ekran
    }
  } else {
    // Resetuj czas ekranu 9, jeśli jesteśmy na innym ekranie
    screen9StartTime = 0;
  }

  // Sprawdzenie prędkości pojazdu - jeśli prędkość przekracza 3 km/h, wybudź i przejdź na ekran 0
  if (gpsSpeed > 3.0 && currentScreen == 8) {
    isScreenSaverActive = false;  // Dezaktywuj wygaszacz
    currentScreen = 0;            // Przełącz na ekran 0
    updateDisplay(true);          // Aktualizuj ekran
  }

  if (currentTime - lastUserInteractionTime > USER_INTERACTION_TIMEOUT) {
    if (currentScreen == 0 || currentScreen == 9) {
      if (gpsSpeed < 3.0 && currentScreen != 9) {
        currentScreen = 9;
        updateDisplay(true);
      } else if (gpsSpeed >= 3.0 && currentScreen != 0) {
        currentScreen = 0;
        updateDisplay(true);
      }
    }
  }

  float currentLux = lightMeter.readLightLevel();
  if (currentLux != globalLux) {
    updateBacklight(currentLux);
    globalLux = currentLux;
  }

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    updateDisplay(false);
    lastUpdate = millis();
  }
}



void updateDisplay(bool fullUpdate) {
  if (fullUpdate) {
    tft.fillScreen(TFT_WHITE);
  }

  if (gps.speed.isValid() && gps.speed.kmph() > 3.0) {
    float slope = calculateSlope();
    // Wyświetl slope na ekranie lub zapisz do dalszego przetwarzania
  }




  switch (currentScreen) {
    case 0:
      {
        // Wypełnij ekran białym kolorem
        //ledcWrite(ledPin, 600);
        tft.setTextPadding(400);
        tft.setTextSize(4);
        tft.setFreeFont(&FreeSans18pt7b);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  // Czarny tekst na białym tle

        tft.drawFloat(gpsSpeed, 0, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
        tft.setFreeFont(NULL);
        tft.setTextSize(2);
        tft.drawString("km/h", SCREEN_WIDTH / 2, 250);

        tft.setTextSize(3);
        char dateStr[20];
        char timeStr[20];

        // Formatowanie daty
        sprintf(dateStr, "%02d/%02d/%04d", gpsDay, gpsMonth, gpsYear);

        // Formatowanie godziny
        sprintf(timeStr, "%02d:%02d:%02d", gpsHour, gpsMinute, gpsSecond);

        // Wyświetlanie daty na ekranie TFT
        tft.drawString(dateStr, SCREEN_WIDTH / 2, 15);

        // Wyświetlanie godziny na ekranie TFT, poniżej daty (przesunięcie na osi Y)
        tft.drawString(timeStr, SCREEN_WIDTH / 2, 40);
        tft.setTextSize(2);
        tft.setTextDatum(TL_DATUM);
        tft.drawString("Sat:", 400, 305);
        tft.setTextPadding(30);
        tft.drawNumber(satellites, 450, 305);

        tft.drawString("Odo:", 5, 300);
        char distanceStr[20];
        sprintf(distanceStr, "%.2f km", totalDistance);
        tft.drawString(distanceStr, 55, 300);


        drawSignalBars(SAT_ICON_X + SAT_ICON_SIZE + 5, SAT_ICON_Y, satellites);

        if (gps.location.isValid()) {
          int currentZoneIndex = checkCurrentZone(gps.location.lat(), gps.location.lng());
          tft.setTextSize(3);
          tft.setTextDatum(MC_DATUM);
          tft.setTextColor(TFT_BLUE, TFT_WHITE);
          tft.setTextPadding(200);  // Ustaw odpowiedni padding dla nazwy strefy
          if (currentZoneIndex != -1) {
            tft.drawString(zones[currentZoneIndex].name, SCREEN_WIDTH / 2, 80);
          } else {
            tft.drawString("Poza strefa", SCREEN_WIDTH / 2, 80);
          }
          tft.setTextDatum(TL_DATUM);
          tft.setTextColor(TFT_BLACK, TFT_WHITE);
        }

        break;
      }
    case 1:
      {
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(2);
        tft.drawString("Latitude:", 10, 10);
        tft.setTextPadding(200);  // Dodaje trochę miejsca do wyczyszczenia tekstu
        if (gps.location.isValid()) {
          tft.drawFloat(gps.location.lat(), 6, 150, 10);  // Wyświetla szerokość geograficzną
        } else {
          tft.drawString("Invalid", 150, 10);  // Informacja, gdy brak ważnych danych GPS
        }

        tft.drawString("Longitude:", 10, 30);
        if (gps.location.isValid()) {
          tft.drawFloat(gps.location.lng(), 6, 150, 30);  // Wyświetla długość geograficzną
        } else {
          tft.drawString("Invalid", 150, 30);
        }

        tft.drawString("Altitude:", 10, 50);
        if (gps.altitude.isValid()) {
          tft.drawFloat(gps.altitude.meters(), 0, 150, 50);  // Wyświetla wysokość
        } else {
          tft.drawString("Invalid", 150, 50);
        }

        tft.drawString("Course:", 10, 70);
        if (gps.course.isValid()) {
          tft.drawFloat(gps.course.deg(), 2, 150, 70);  // Wyświetla kierunek (kurs)
        } else {
          tft.drawString("Invalid", 150, 70);
        }

        tft.drawString("LUX light:", 10, 90);
        tft.setTextPadding(100);
        tft.drawFloat(globalLux, 2, 150, 90);

        tft.drawString("LCD light:", 10, 110);
        tft.setTextPadding(100);
        tft.drawFloat(currentBrightness, 2, 150, 110);


        tft.drawString("Ava.Speed:", 10, 130);
        tft.setTextPadding(100);
        tft.drawFloat(averageSpeed, 2, 150, 130);
        tft.drawString("km/h", 210, 130);

        tft.drawString("Max.speed:", 10, 150);
        tft.setTextPadding(100);
        tft.drawFloat(maxSpeed, 2, 150, 150);
        tft.drawString("km/h", 210, 150);

        tft.drawString("ACCELERATION:", 10, 170);
        tft.setTextPadding(100);
        tft.drawFloat(calculateAcceleration(), 2, 190, 170);
        tft.drawString("m/s^2", 260, 170);

        tft.drawString("SLOPE:", 10, 190);
        tft.setTextPadding(100);
        tft.drawFloat(calculateSlope(), 2, 190, 190);
        tft.drawString("%", 260, 190);

        float destLat = 50.309525547747874;  // przykładowa szerokość geograficzna
        float destLon = 18.61577283875242;   // przykładowa długość geograficzna
        tft.drawString("ETA:", 10, 210);
        tft.drawString(calculateETA(destLat, destLon), 150, 210);

        break;
      }
    case 2:
      if (gps.location.isValid()) {
        float currentLat = gps.location.lat();
        float currentLon = gps.location.lng();
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(2);

        int lineSpacing = 20;   // Odstęp między liniami
        int startY = 10;        // Początkowa pozycja y
        int currentY = startY;  // Bieżąca pozycja y
        int fixedX = 170;       // Ustalona pozycja x dla odległości

        // Obliczenie odległości do każdej lokalizacji
        for (int i = 0; i < sizeof(locations) / sizeof(locations[0]); i++) {
          float distance = haversine(currentLat, currentLon, locations[i].latitude, locations[i].longitude);
          tft.drawString(locations[i].name, 5, currentY);                 // Nazwa lokalizacji na x = 5
          tft.drawString(String(distance, 2) + " km", fixedX, currentY);  // Odległość na x = 270
          currentY += lineSpacing;                                        // Przesunięcie w dół o odstęp
        }
      } else {
        tft.drawString("Brak Sygnalu GPS", 10, 10);
      }
      break;

    case 3:
      {
        const int SCALE_WIDTH = 40;  // Szerokość obszaru skali
        const int GRAPH_WIDTH = SCREEN_WIDTH - SCALE_WIDTH;
        const int GRAPH_HEIGHT = 200;
        const int GRAPH_Y = 80;
        const int NUM_BARS = GRAPH_WIDTH / BAR_WIDTH;
        const float LUX_THRESHOLD = 1000.0;  // Próg światła dla zmiany koloru
        const float MIN_LUX_CHANGE = 0.1;    // Minimalna zmiana światła do aktualizacji wykresu

        static float luxHistory[NUM_BARS] = { 0 };
        static int historyIndex = 0;
        static bool isInitialized = false;
        static float maxLux = 0;       // Zmienna do przechowywania maksymalnego poziomu światła
        static bool isPaused = false;  // Flaga informująca, czy wykres jest wstrzymany
        static float lastLux = 0;      // Ostatnio zapisana wartość światła

        tft.setTextSize(1);
        tft.setTextColor(TFT_BLACK);
        for (int i = 0; i <= 1500; i += 300) {
          int y = map(i, 0, 1500, GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y);
          tft.drawFastHLine(SCALE_WIDTH - 5, y, 5, TFT_BLACK);
          tft.setCursor(5, y - 4);
          tft.print(i);
        }
        tft.drawFastHLine(SCALE_WIDTH, GRAPH_Y + GRAPH_HEIGHT, GRAPH_WIDTH, TFT_BLACK);

        // Funkcja do rysowania pojedynczego słupka
        auto drawBar = [&](int x, float lux) {
          int barHeight = map(lux, 0, 1500, 0, GRAPH_HEIGHT);
          int y = GRAPH_Y + GRAPH_HEIGHT - barHeight;
          tft.fillRect(x + SCALE_WIDTH, GRAPH_Y, BAR_WIDTH - 1, GRAPH_HEIGHT, TFT_WHITE);  // Czyści cały słupek

          // Wybierz kolor słupka w zależności od poziomu światła
          uint16_t barColor = (lux > LUX_THRESHOLD) ? TFT_YELLOW : TFT_BLUE;
          tft.fillRect(x + SCALE_WIDTH, y, BAR_WIDTH - 1, barHeight, barColor);
        };

        // Funkcja do rysowania całego wykresu
        auto drawFullGraph = [&]() {
          tft.fillRect(SCALE_WIDTH, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_WHITE);

          for (int i = 0; i < NUM_BARS; i++) {
            int index = (historyIndex - i + NUM_BARS) % NUM_BARS;
            int x = GRAPH_WIDTH - (i + 1) * BAR_WIDTH;
            drawBar(x, luxHistory[index]);
          }
        };

        // Funkcja do aktualizacji wykresu
        auto updateLuxGraph = [&](float lux) {
          int oldestIndex = historyIndex;
          luxHistory[historyIndex] = lux;
          historyIndex = (historyIndex + 1) % NUM_BARS;

          // Aktualizacja maksymalnego poziomu światła
          if (lux > maxLux) {
            maxLux = lux;
          }

          // Sprawdź, czy maksymalny poziom światła nadal jest na wykresie
          bool maxLuxOnGraph = false;
          for (int i = 0; i < NUM_BARS; i++) {
            if (luxHistory[i] == maxLux) {
              maxLuxOnGraph = true;
              break;
            }
          }

          // Jeśli maksymalny poziom światła nie jest już na wykresie, znajdź nowy maksymalny
          if (!maxLuxOnGraph) {
            maxLux = 0;
            for (int i = 0; i < NUM_BARS; i++) {
              if (luxHistory[i] > maxLux) {
                maxLux = luxHistory[i];
              }
            }
          }

          // Przesuń wykres w lewo
          for (int i = 0; i < NUM_BARS; i++) {
            int index = (oldestIndex - i + NUM_BARS) % NUM_BARS;
            int x = GRAPH_WIDTH - (i + 1) * BAR_WIDTH;
            drawBar(x, luxHistory[index]);
          }
        };

        // Pełne przerysowanie wykresu tylko przy wejściu na ekran
        static int lastScreen = -1;
        if (currentScreen != lastScreen) {
          drawFullGraph();
          lastScreen = currentScreen;
        }

        // Inicjalizacja przy pierwszym uruchomieniu
        if (!isInitialized) {
          for (int i = 0; i < NUM_BARS; i++) {
            luxHistory[i] = 0;
          }
          isInitialized = true;
          maxLux = 0;
          lastLux = globalLux;
        }

        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(2);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.drawString("Wykres swiatla:", 10, 10);

        // Pobierz aktualny poziom światła
        float currentLux = globalLux;

        // Aktualizuj wykres tylko jeśli zmiana światła przekracza MIN_LUX_CHANGE
        if (abs(currentLux - lastLux) > MIN_LUX_CHANGE) {
          updateLuxGraph(currentLux);
          isPaused = false;
          lastLux = currentLux;
        } else {
          isPaused = true;
        }

        // Wyświetl aktualny poziom światła jako tekst
        tft.setTextSize(2);
        tft.setCursor(10, 30);
        tft.print("Swiatlo: ");
        tft.print(currentLux, 1);
        tft.print(" lux");

        // Wyświetl maksymalny poziom światła
        tft.setTextSize(2);
        tft.setCursor(10, 50);
        tft.print("Max: ");
        tft.print(maxLux, 1);
        tft.print(" lux");

        // Wyświetl status pauzy jako kolorową kropkę w prawym górnym rogu
        const int DOT_RADIUS = 5;
        const int DOT_X = SCREEN_WIDTH - DOT_RADIUS - 5;  // 5 pikseli od prawej krawędzi
        const int DOT_Y = DOT_RADIUS + 5;                 // 5 pikseli od górnej krawędzi

        if (isPaused) {
          tft.fillCircle(DOT_X, DOT_Y, DOT_RADIUS, TFT_RED);
        } else {
          tft.fillCircle(DOT_X, DOT_Y, DOT_RADIUS, TFT_GREEN);
        }
      }
      break;

    case 4:
      {
        const int SCALE_WIDTH = 40;  // Szerokość obszaru skali
        const int GRAPH_WIDTH = SCREEN_WIDTH - SCALE_WIDTH;
        const int GRAPH_HEIGHT = 200;
        const int GRAPH_Y = 80;
        const int NUM_BARS = GRAPH_WIDTH / BAR_WIDTH;
        const float SPEED_THRESHOLD = 60.0;  // Próg prędkości dla zmiany koloru
        const float MIN_SPEED = 3.0;         // Minimalna prędkość do aktualizacji wykresu (km/h)

        static float speedHistory[NUM_BARS] = { 0 };
        static int historyIndex = 0;
        static bool isInitialized = false;
        static float maxSpeed = 0;     // Zmienna do przechowywania maksymalnej prędkości
        static bool isPaused = false;  // Flaga informująca, czy wykres jest wstrzymany

        tft.setTextSize(1);
        tft.setTextColor(TFT_BLACK);
        for (int i = 0; i <= 100; i += 20) {
          int y = map(i, 0, 100, GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y);
          tft.drawFastHLine(SCALE_WIDTH - 5, y, 5, TFT_BLACK);
          tft.setCursor(5, y - 4);
          tft.print(i);
        }
        tft.drawFastHLine(SCALE_WIDTH, GRAPH_Y + GRAPH_HEIGHT, GRAPH_WIDTH, TFT_BLACK);

        // Funkcja do rysowania pojedynczego słupka
        auto drawBar = [&](int x, float speed) {
          int barHeight = map(speed, 0, 100, 0, GRAPH_HEIGHT);
          int y = GRAPH_Y + GRAPH_HEIGHT - barHeight;
          tft.fillRect(x + SCALE_WIDTH, GRAPH_Y, BAR_WIDTH - 1, GRAPH_HEIGHT, TFT_WHITE);  // Czyści cały słupek

          // Wybierz kolor słupka w zależności od prędkości
          uint16_t barColor = (speed > SPEED_THRESHOLD) ? TFT_RED : TFT_BLUE;
          tft.fillRect(x + SCALE_WIDTH, y, BAR_WIDTH - 1, barHeight, barColor);
        };

        // Funkcja do rysowania całego wykresu
        auto drawFullGraph = [&]() {
          tft.fillRect(SCALE_WIDTH, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_WHITE);

          for (int i = 0; i < NUM_BARS; i++) {
            int index = (historyIndex - i + NUM_BARS) % NUM_BARS;
            int x = GRAPH_WIDTH - (i + 1) * BAR_WIDTH;
            drawBar(x, speedHistory[index]);
          }
        };

        // Funkcja do aktualizacji wykresu
        auto updateSpeedGraph = [&](float speed) {
          int oldestIndex = historyIndex;
          speedHistory[historyIndex] = speed;
          historyIndex = (historyIndex + 1) % NUM_BARS;

          // Aktualizacja maksymalnej prędkości
          if (speed > maxSpeed) {
            maxSpeed = speed;
          }

          // Sprawdź, czy maksymalna prędkość nadal jest na wykresie
          bool maxSpeedOnGraph = false;
          for (int i = 0; i < NUM_BARS; i++) {
            if (speedHistory[i] == maxSpeed) {
              maxSpeedOnGraph = true;
              break;
            }
          }

          // Jeśli maksymalna prędkość nie jest już na wykresie, znajdź nową maksymalną
          if (!maxSpeedOnGraph) {
            maxSpeed = 0;
            for (int i = 0; i < NUM_BARS; i++) {
              if (speedHistory[i] > maxSpeed) {
                maxSpeed = speedHistory[i];
              }
            }
          }

          // Przesuń wykres w lewo
          for (int i = 0; i < NUM_BARS; i++) {
            int index = (oldestIndex - i + NUM_BARS) % NUM_BARS;
            int x = GRAPH_WIDTH - (i + 1) * BAR_WIDTH;
            drawBar(x, speedHistory[index]);
          }
        };

        // Pełne przerysowanie wykresu tylko przy wejściu na ekran
        static int lastScreen = -1;
        if (currentScreen != lastScreen) {
          drawFullGraph();
          lastScreen = currentScreen;
        }

        // Inicjalizacja przy pierwszym uruchomieniu
        if (!isInitialized) {
          for (int i = 0; i < NUM_BARS; i++) {
            speedHistory[i] = 0;
          }
          isInitialized = true;
          maxSpeed = 0;
        }

        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(2);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.drawString("Wykres predkosci:", 10, 10);

        // Pobierz aktualną prędkość
        float currentSpeed = gps.speed.kmph();
        //float currentSpeed = random(0, 100);

        // Aktualizuj wykres tylko jeśli prędkość przekracza MIN_SPEED
        if (currentSpeed > MIN_SPEED) {
          updateSpeedGraph(currentSpeed);
          isPaused = false;
        } else {
          isPaused = true;
        }

        // Wyświetl aktualną prędkość jako tekst
        tft.setTextSize(2);
        tft.setCursor(10, 30);
        tft.print("Predkosc: ");
        tft.print(currentSpeed, 1);
        tft.print(" km/h");

        // Wyświetl maksymalną prędkość
        tft.setTextSize(2);
        tft.setCursor(10, 50);
        tft.print("Max: ");
        tft.print(maxSpeed, 1);
        tft.print(" km/h");

        // Wyświetl status pauzy jako kolorową kropkę w prawym górnym rogu
        const int DOT_RADIUS = 5;
        const int DOT_X = SCREEN_WIDTH - DOT_RADIUS - 5;  // 5 pikseli od prawej krawędzi
        const int DOT_Y = DOT_RADIUS + 5;                 // 5 pikseli od górnej krawędzi

        if (isPaused) {
          tft.fillCircle(DOT_X, DOT_Y, DOT_RADIUS, TFT_RED);
        } else {
          tft.fillCircle(DOT_X, DOT_Y, DOT_RADIUS, TFT_GREEN);
        }
      }
      break;




    case 5:
      {
        // Stałe i zmienne
        const int NUM_MARKINGS = 17;  // 17 znaczników, ale wyświetlimy tylko 16 etykiet
        const int MARKING_LENGTH = 15;
        const int TEXT_OFFSET = 9;
        const int FONT_SIZE = 2;
        const int NEEDLE_THICKNESS = 6;
        const int NEEDLE_RADIUS = RADIUS - TEXT_OFFSET - MARKING_LENGTH - 0;
        const float MAX_SPEED = 80.0;

        static float lastSpeed = -1;

        // Funkcja rysowania skali
        auto drawScale = [&]() {
          tft.drawCircle(CENTER_X, CENTER_Y, 160, TFT_BLACK);
          tft.setTextSize(FONT_SIZE);
          tft.setTextColor(TFT_BLACK);

          for (int i = 0; i < NUM_MARKINGS; i++) {
            float angle = i * 2 * PI / (NUM_MARKINGS - 1) - PI / 2;
            int x1 = CENTER_X + (160 - MARKING_LENGTH) * cos(angle);
            int y1 = CENTER_Y + (160 - MARKING_LENGTH) * sin(angle);
            int x2 = CENTER_X + 160 * cos(angle);
            int y2 = CENTER_Y + 160 * sin(angle);

            tft.drawLine(x1, y1, x2, y2, TFT_BLACK);

            // Rysuj etykiety tylko dla pierwszych 16 znaczników (0-75)
            if (i < 16) {
              int labelValue = i * 5;
              String label = String(labelValue);
              int textX = CENTER_X + (RADIUS - TEXT_OFFSET) * cos(angle) - label.length() * 3;
              int textY = CENTER_Y + (RADIUS - TEXT_OFFSET) * sin(angle) - 4;
              tft.setCursor(textX, textY);
              tft.print(label);
            }
          }
        };

        // Rysuj skalę za każdym razem
        drawScale();

        // Pobierz aktualną prędkość z GPS
        float currentSpeed = gps.speed.kmph();

        // Ograniczenie prędkości do MAX_SPEED
        currentSpeed = min(currentSpeed, MAX_SPEED);

        // Aktualizuj wyświetlacz tylko jeśli prędkość się zmieniła
        if (currentSpeed != lastSpeed) {
          tft.fillCircle(CENTER_X, CENTER_Y, NEEDLE_RADIUS + 1, TFT_WHITE);
          tft.fillRect(CENTER_X - 50, CENTER_Y + 20, 100, 40, TFT_WHITE);

          float needleAngle = (currentSpeed / MAX_SPEED) * 2 * PI - PI / 2;

          int needleX = CENTER_X + NEEDLE_RADIUS * cos(needleAngle);
          int needleY = CENTER_Y + NEEDLE_RADIUS * sin(needleAngle);

          for (int i = -NEEDLE_THICKNESS / 2; i <= NEEDLE_THICKNESS / 2; i++) {
            int offsetX = -i * sin(needleAngle);
            int offsetY = i * cos(needleAngle);
            tft.drawLine(CENTER_X + offsetX, CENTER_Y + offsetY, needleX + offsetX, needleY + offsetY, TFT_RED);
          }

          tft.setTextSize(3);
          tft.setTextColor(TFT_RED);
          String speedText = String(currentSpeed, 1) + " km/h";
          int speedTextWidth = speedText.length() * 18;
          tft.setCursor(CENTER_X - speedTextWidth / 2, CENTER_Y + 40);
          tft.print(speedText);

          lastSpeed = currentSpeed;
        }

        delay(200);  // Zmniejszono opóźnienie dla częstszych aktualizacji

        break;
      }

    case 6:
      {
        static float lastAcceleration = -999;
        static float lastSpeed = -1;

        float speed = gps.speed.kmph();
        float acceleration = calculateAcceleration();

        int screenWidth = tft.width();
        int screenHeight = tft.height();
        int centerX = screenWidth / 2;
        int centerY = screenHeight / 2;
        int scaleHeight = 200;
        int topY = centerY - (scaleHeight / 2);
        int bottomY = centerY + (scaleHeight / 2);

        // Funkcja rysowania skali
        auto drawScale = [&]() {
          tft.drawLine(centerX, topY, centerX, bottomY, TFT_BLACK);

          tft.setTextColor(TFT_BLACK);
          tft.setTextSize(2);

          for (int i = -20; i <= 20; i += 5) {
            int y = map(i, -20, 20, bottomY, topY);
            tft.setCursor(centerX + 10, y - 8);
            tft.print(i);
            tft.drawLine(centerX - 5, y, centerX + 5, y, TFT_BLACK);
          }
        };

        // Rysuj skalę za każdym razem
        drawScale();

        // Aktualizuj wyświetlacz tylko jeśli przyspieszenie lub prędkość się zmieniły
        if (acceleration != lastAcceleration || speed != lastSpeed) {
          acceleration = constrain(acceleration, -20, 20);
          int newDotY = map(acceleration, -20, 20, bottomY, topY);
          int dotX = centerX - 20;  // Przesunięcie o 10 pikseli w lewo (było centerX - 10)

          // Czyść tylko obszar kropki i prędkości
          tft.fillRect(dotX - 5, topY, 15, scaleHeight, TFT_WHITE);  // Zwiększono szerokość czyszczenia
          tft.fillRect(10, 10, 150, 20, TFT_WHITE);

          // Rysuj nową kropkę
          tft.fillCircle(dotX, newDotY, 5, TFT_RED);

          // Aktualizacja wyświetlania prędkości
          tft.setTextColor(TFT_BLUE);
          tft.setTextSize(2);
          tft.setCursor(10, 10);
          tft.print("Speed: ");
          tft.print(speed, 1);
          tft.print(" km/h");

          lastAcceleration = acceleration;
          lastSpeed = speed;
        }

        delay(100);  // Krótkie opóźnienie dla stabilności

        break;
      }

    case 7:
      {
        const int SCALE_WIDTH = 40;  // Szerokość obszaru skali
        const int GRAPH_WIDTH = SCREEN_WIDTH - SCALE_WIDTH;
        const int GRAPH_HEIGHT = 200;
        const int GRAPH_Y = 80;
        const int NUM_BARS = GRAPH_WIDTH / BAR_WIDTH;
        const float MIN_ALTITUDE = 200.0;  // Minimalna wysokość na wykresie
        const float MAX_ALTITUDE = 400.0;  // Maksymalna wysokość na wykresie
        const float MIN_SPEED = 3.0;       // Minimalna prędkość do aktualizacji wykresu (km/h)

        static float altitudeHistory[NUM_BARS] = { 0 };
        static int historyIndex = 0;
        static bool isInitialized = false;
        static float maxAltitude = MIN_ALTITUDE;  // Zmienna do przechowywania maksymalnej wysokości
        static bool isPaused = false;             // Flaga informująca, czy wykres jest wstrzymany

        tft.setTextSize(1);
        tft.setTextColor(TFT_BLACK);
        for (int i = MIN_ALTITUDE; i <= MAX_ALTITUDE; i += 50) {
          int y = map(i, MIN_ALTITUDE, MAX_ALTITUDE, GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y);
          tft.drawFastHLine(SCALE_WIDTH - 5, y, 5, TFT_BLACK);
          tft.setCursor(5, y - 4);
          tft.print(i);
        }
        tft.drawFastHLine(SCALE_WIDTH, GRAPH_Y + GRAPH_HEIGHT, GRAPH_WIDTH, TFT_BLACK);

        // Funkcja do rysowania pojedynczego słupka
        auto drawBar = [&](int x, float altitude) {
          int barHeight = map(altitude, MIN_ALTITUDE, MAX_ALTITUDE, 0, GRAPH_HEIGHT);
          int y = GRAPH_Y + GRAPH_HEIGHT - barHeight;
          tft.fillRect(x + SCALE_WIDTH, GRAPH_Y, BAR_WIDTH - 1, GRAPH_HEIGHT, TFT_WHITE);  // Czyści cały słupek
          tft.fillRect(x + SCALE_WIDTH, y, BAR_WIDTH - 1, barHeight, TFT_BLUE);            // Rysuje niebieski słupek
        };

        // Funkcja do rysowania całego wykresu
        auto drawFullGraph = [&]() {
          tft.fillRect(SCALE_WIDTH, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_WHITE);

          for (int i = 0; i < NUM_BARS; i++) {
            int index = (historyIndex - i + NUM_BARS) % NUM_BARS;
            int x = GRAPH_WIDTH - (i + 1) * BAR_WIDTH;
            drawBar(x, altitudeHistory[index]);
          }
        };

        // Funkcja do aktualizacji wykresu
        auto updateAltitudeGraph = [&](float altitude) {
          int oldestIndex = historyIndex;
          altitudeHistory[historyIndex] = constrain(altitude, MIN_ALTITUDE, MAX_ALTITUDE);
          historyIndex = (historyIndex + 1) % NUM_BARS;

          // Aktualizacja maksymalnej wysokości
          if (altitude > maxAltitude) {
            maxAltitude = altitude;
          }

          // Sprawdź, czy maksymalna wysokość nadal jest na wykresie
          bool maxAltitudeOnGraph = false;
          for (int i = 0; i < NUM_BARS; i++) {
            if (altitudeHistory[i] == maxAltitude) {
              maxAltitudeOnGraph = true;
              break;
            }
          }

          // Jeśli maksymalna wysokość nie jest już na wykresie, znajdź nową maksymalną
          if (!maxAltitudeOnGraph) {
            maxAltitude = MIN_ALTITUDE;
            for (int i = 0; i < NUM_BARS; i++) {
              if (altitudeHistory[i] > maxAltitude) {
                maxAltitude = altitudeHistory[i];
              }
            }
          }

          // Przesuń wykres w lewo
          for (int i = 0; i < NUM_BARS; i++) {
            int index = (oldestIndex - i + NUM_BARS) % NUM_BARS;
            int x = GRAPH_WIDTH - (i + 1) * BAR_WIDTH;
            drawBar(x, altitudeHistory[index]);
          }
        };

        // Pełne przerysowanie wykresu tylko przy wejściu na ekran
        static int lastScreen = -1;
        if (currentScreen != lastScreen) {
          drawFullGraph();
          lastScreen = currentScreen;
        }

        // Inicjalizacja przy pierwszym uruchomieniu
        if (!isInitialized) {
          for (int i = 0; i < NUM_BARS; i++) {
            altitudeHistory[i] = MIN_ALTITUDE;
          }
          isInitialized = true;
          maxAltitude = MIN_ALTITUDE;
        }

        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(2);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.drawString("Wykres wysokosci:", 10, 10);

        // Pobierz aktualną wysokość i prędkość
        float currentAltitude = gps.altitude.meters();
        float currentSpeed = gps.speed.kmph();

        // Aktualizuj wykres tylko jeśli prędkość przekracza 3 km/h
        if (currentSpeed > MIN_SPEED) {
          updateAltitudeGraph(currentAltitude);
          isPaused = false;
        } else {
          isPaused = true;
        }

        // Wyświetl aktualną wysokość jako tekst
        tft.setTextSize(2);
        tft.setCursor(10, 30);
        tft.print("Wysokosc: ");
        tft.print(currentAltitude, 1);
        tft.print(" m");

        // Wyświetl maksymalną wysokość
        tft.setTextSize(2);
        tft.setCursor(10, 50);
        tft.print("Max: ");
        tft.print(maxAltitude, 1);
        tft.print(" m");



        // Wyświetl status pauzy jako kolorową kropkę w prawym górnym rogu
        const int DOT_RADIUS = 5;
        const int DOT_X = SCREEN_WIDTH - DOT_RADIUS - 5;  // 5 pikseli od prawej krawędzi
        const int DOT_Y = DOT_RADIUS + 5;                 // 5 pikseli od górnej krawędzi

        if (isPaused) {
          tft.fillCircle(DOT_X, DOT_Y, DOT_RADIUS, TFT_RED);
        } else {
          tft.fillCircle(DOT_X, DOT_Y, DOT_RADIUS, TFT_GREEN);
        }
      }
      break;

    case 8:
      {
        currentScreen = 8;
        static int pixelX = 0;
        static int pixelY = 0;
        static uint32_t lastPixelUpdate = 0;
        static uint32_t pixelColor = TFT_RED;
        static const int pixelSize = 48;  // Rozmiar "piksela" (można dostosować)

        uint32_t currentMillis = millis();

        ledcWrite(ledPin, 1);
        // Aktualizacja pozycji piksela co 100ms (można dostosować)
        if (currentMillis - lastPixelUpdate >= 100) {

          // Czyść poprzedni piksel (rysuj go na czarno, aby wyglądało jakby zniknął)
          tft.fillRect(pixelX, pixelY, pixelSize, pixelSize, TFT_BLACK);

          // Przesuń piksel do następnej pozycji
          pixelX += pixelSize;
          if (pixelX >= SCREEN_WIDTH) {
            pixelX = 0;
            pixelY += pixelSize;
            if (pixelY >= SCREEN_HEIGHT) {
              pixelY = 0;
              pixelColor = random(0xFFFF);  // Nowy losowy kolor dla kolejnych ruchów
            }
          }

          // Narysuj nowy piksel
          tft.fillRect(pixelX, pixelY, pixelSize, pixelSize, pixelColor);

          // Zaktualizuj czas ostatniej zmiany
          lastPixelUpdate = currentMillis;
        }

        break;
      }

    case 9:
      {
        tft.setTextDatum(TL_DATUM);
        sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp);

        float temperature = temp.temperature;
        float humidityLevel = humidity.relative_humidity;
        tft.setTextSize(2);
        char dateStr[20];
        sprintf(dateStr, "%02d/%02d/%04d", gpsDay, gpsMonth, gpsYear);
        tft.drawString(dateStr, 5, 5);

        char timeStr[20];
        sprintf(timeStr, "%02d:%02d:%02d", gpsHour, gpsMinute, gpsSecond);
        tft.drawString(timeStr, 5, 25);

        tft.setTextSize(2);
        tft.drawString("Temperatura:", 285, 5);
        tft.setTextPadding(100);
        tft.drawFloat(temperature, 1, 430, 5);

        tft.drawString("Wilgotnosc:", 285, 25);
        tft.setTextPadding(100);
        tft.drawFloat(humidityLevel, 1, 430, 25);

        tft.drawString("---------- Liczniki Dzienne ----------", 5, 65);

        tft.drawString("Dzis Przejechano: ", 5, 125);
        char dailyDistanceStr[20];
        sprintf(dailyDistanceStr, "%.2f km", dailyDistance);
        tft.drawString(dailyDistanceStr, 250, 125);

        tft.drawString("Czas jazdy Dzien:", 5, 105);
        unsigned long drivingSeconds2 = drivingTime2 / 1000;
        char drivingTimeStr2[20];
        sprintf(drivingTimeStr2, "%02lu:%02lu:%02lu", drivingSeconds2 / 3600, (drivingSeconds2 % 3600) / 60, drivingSeconds2 % 60);
        tft.drawString(drivingTimeStr2, 250, 105);

        tft.drawString("Czas postoju Dzien:", 5, 85);
        unsigned long stopSeconds2 = stopTime2 / 1000;
        char stopTimeStr2[20];
        sprintf(stopTimeStr2, "%02lu:%02lu:%02lu", stopSeconds2 / 3600, (stopSeconds2 % 3600) / 60, stopSeconds2 % 60);
        tft.drawString(stopTimeStr2, 250, 85);

        tft.drawString("--------- Liczniki Sumaryczne --------", 5, 145);

        tft.drawString("Czas jazdy Suma:", 5, 185);
        unsigned long drivingSeconds = drivingTime / 1000;
        char drivingTimeStr[20];
        sprintf(drivingTimeStr, "%02lu:%02lu:%02lu", drivingSeconds / 3600, (drivingSeconds % 3600) / 60, drivingSeconds % 60);
        tft.drawString(drivingTimeStr, 250, 185);

        // Wyświetlanie czasu postoju
        tft.drawString("Czas postoju Suma:", 5, 165);
        unsigned long stopSeconds = stopTime / 1000;
        char stopTimeStr[20];
        sprintf(stopTimeStr, "%02lu:%02lu:%02lu", stopSeconds / 3600, (stopSeconds % 3600) / 60, stopSeconds % 60);
        tft.drawString(stopTimeStr, 250, 165);

        tft.drawString("Suma Przejechano :", 5, 205);
        char distanceStr[20];
        sprintf(distanceStr, "%.2f km", totalDistance);
        tft.drawString(distanceStr, 250, 205);

        // Wyświetlanie czasu postoju dla drugiego stopera


        break;
      }
    case 10:
      {
        static float destinationLat = 50.30954827360399;  //DOM
        static float destinationLon = 18.615807836661556;
        static bool isNavigationActive = true;  // Domyślnie aktywna
        static uint32_t lastUpdateTime = 0;
        static float lastBearing = 0;
        static float lastDistance = 0;
        static float lastSpeed = 0;
        static float lastLat = 0;
        static float lastLon = 0;
        static float lastHeading = 0;
        static float lastCourse = 0;

        auto calculateBearing = [](float lat1, float lon1, float lat2, float lon2) -> float {
          float dLon = (lon2 - lon1) * (PI / 180.0);
          lat1 *= PI / 180.0;
          lat2 *= PI / 180.0;
          float y = sin(dLon) * cos(lat2);
          float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
          return atan2(y, x);
        };

        auto drawCompass = [&](float bearing, float heading, int centerX, int centerY, int radius) {
          // Czyść tylko obszar kompasu
          tft.fillCircle(centerX, centerY, radius + 1, TFT_WHITE);
          tft.drawCircle(centerX, centerY, radius, TFT_BLACK);

          // Rysuj podziałkę
          for (int i = 0; i < 360; i += 15) {
            float angle = i * PI / 180 - heading;
            int innerRadius = (i % 45 == 0) ? radius - 15 : radius - 10;
            int outerX = centerX + radius * sin(angle);
            int outerY = centerY - radius * cos(angle);
            int innerX = centerX + innerRadius * sin(angle);
            int innerY = centerY - innerRadius * cos(angle);
            tft.drawLine(innerX, innerY, outerX, outerY, TFT_BLACK);
          }

          // Rysuj oznaczenia kierunków świata
          const char* directions[] = { "N", "E", "S", "W" };
          for (int i = 0; i < 4; i++) {
            float angle = i * PI / 2 - heading;
            int x = centerX + (radius - 25) * sin(angle);
            int y = centerY - (radius - 25) * cos(angle);
            tft.setTextDatum(MC_DATUM);
            if (i == 0) {  // Dla "N" (północy)
              tft.setTextColor(TFT_RED);
            } else {
              tft.setTextColor(TFT_BLACK);
            }
            tft.drawString(directions[i], x, y);
          }

          // Rysuj pogrubioną strzałkę wskazującą cel
          float arrowAngle = bearing - heading;
          int arrowLength = radius - 20;
          int arrowX = centerX + arrowLength * sin(arrowAngle);
          int arrowY = centerY - arrowLength * cos(arrowAngle);

          // Rysuj grubszą linię
          for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
              tft.drawLine(centerX + i, centerY + j, arrowX + i, arrowY + j, TFT_RED);
            }
          }

          // Rysuj pogrubiony grot strzałki
          int triangleSize = 12;  // Zwiększony rozmiar grotu
          int triangle[6] = {
            arrowX, arrowY,
            static_cast<int>(arrowX - triangleSize * sin(arrowAngle - PI / 6)),
            static_cast<int>(arrowY + triangleSize * cos(arrowAngle - PI / 6)),
            static_cast<int>(arrowX - triangleSize * sin(arrowAngle + PI / 6)),
            static_cast<int>(arrowY + triangleSize * cos(arrowAngle + PI / 6))
          };
          tft.fillTriangle(triangle[0], triangle[1], triangle[2], triangle[3], triangle[4], triangle[5], TFT_RED);
        };

        uint32_t currentTime = millis();

        // Aktualizuj ekran co 1 sekundę lub przy pełnej aktualizacji
        if (fullUpdate || (currentTime - lastUpdateTime >= 1000)) {
          if (isNavigationActive && gps.location.isValid()) {
            float currentLat = gps.location.lat();
            float currentLon = gps.location.lng();
            float currentHeading = gps.course.deg() * (PI / 180.0);  // Konwersja na radiany
            float currentSpeed = gps.speed.kmph();
            float currentCourse = gps.course.deg();

            float bearing = calculateBearing(currentLat, currentLon, destinationLat, destinationLon);
            float distance = TinyGPSPlus::distanceBetween(currentLat, currentLon, destinationLat, destinationLon);

            // Rysuj kompas tylko jeśli kierunek lub heading się zmieniły
            if (fullUpdate || abs(bearing - lastBearing) > 0.1 || abs(currentHeading - lastHeading) > 0.1) {
              drawCompass(bearing, currentHeading, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 - 40, 100);
              lastBearing = bearing;
              lastHeading = currentHeading;
            }

            tft.setTextDatum(TL_DATUM);
            tft.setTextSize(2);

            // Aktualizuj odległość tylko jeśli się zmieniła
            if (fullUpdate || abs(distance - lastDistance) > 10) {  // 10 metrów różnicy
              tft.setTextColor(TFT_BLACK, TFT_WHITE);
              tft.drawString("Odleglosc:", 10, 270);
              tft.setTextPadding(100);                      // Zapewnia czyszczenie poprzedniej wartości
              tft.drawFloat(distance / 1000, 3, 130, 270);  // Konwersja na kilometry
              tft.drawString("km", 200, 270);
              lastDistance = distance;
            }

            // Aktualizuj kurs
            if (fullUpdate || abs(currentCourse - lastCourse) > 1.0) {  // 1 stopień różnicy
              tft.setTextColor(TFT_BLACK, TFT_WHITE);
              tft.drawString("Kurs:", 10, 290);
              tft.setTextPadding(100);
              tft.drawFloat(currentCourse, 1, 130, 290);
              tft.drawString("deg", 200, 290);
              lastCourse = currentCourse;
            }

            // Aktualizuj prędkość
            if (fullUpdate || abs(currentSpeed - lastSpeed) > 0.5) {  // 0.5 km/h różnicy
              tft.setTextSize(2);
              tft.setTextColor(TFT_BLACK, TFT_WHITE);
              tft.setTextDatum(TL_DATUM);
              tft.setTextPadding(470);  // Zwiększona wartość, aby pokryć całą linię
              char speedBuffer[30];
              sprintf(speedBuffer, "Predkosc: %.1f km/h", currentSpeed);
              tft.drawString(speedBuffer, 10, 250);
              lastSpeed = currentSpeed;
            }
          } else if (fullUpdate) {
            tft.fillScreen(TFT_WHITE);
            tft.setTextDatum(MC_DATUM);
            tft.setTextSize(2);
            tft.setTextColor(TFT_BLACK);
            tft.drawString("Nawigacja nieaktywna", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
            tft.drawString("lub brak sygnalu GPS", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 30);
          }

          lastUpdateTime = currentTime;
        }
      }
      break;
    case 11:
      {
        const int SCALE_WIDTH = 50;
        const int GRAPH_WIDTH = SCREEN_WIDTH - SCALE_WIDTH;
        const int GRAPH_HEIGHT = 200;
        const int GRAPH_Y = 80;
        const int BAR_WIDTH = GRAPH_WIDTH / 24;  // Szerokość słupka dla 24 godzin
        const int VERTICAL_LINE_OFFSET = 5;
        static bool graphInitialized = false;

        // Inicjalizacja wykresu (osie i skala)
        if (!graphInitialized || fullUpdate) {
          tft.fillScreen(TFT_WHITE);

          // Rysowanie tytułu
          tft.setTextColor(TFT_BLACK, TFT_WHITE);
          tft.setTextSize(2);
          tft.setCursor(10, 10);
          tft.print("Dystans godzinowy");

          // Rysowanie osi Y (skala kilometrów)
          tft.setTextSize(1);
          tft.drawFastVLine(SCALE_WIDTH - VERTICAL_LINE_OFFSET, GRAPH_Y, GRAPH_HEIGHT, TFT_BLACK);

          // Rysowanie podziałki na osi Y
          for (int i = 0; i <= MAX_DISTANCE; i += 10) {
            int y = map(i, 0, MAX_DISTANCE, GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y);
            tft.drawFastHLine(SCALE_WIDTH - VERTICAL_LINE_OFFSET - 5, y, 5, TFT_BLACK);
            tft.setCursor(5, y - 4);
            tft.print(i);
          }

          // Rysowanie osi X (skala godzin)
          tft.drawFastHLine(SCALE_WIDTH - VERTICAL_LINE_OFFSET, GRAPH_Y + GRAPH_HEIGHT,
                            GRAPH_WIDTH + VERTICAL_LINE_OFFSET, TFT_BLACK);

          // Rysowanie podziałki na osi X
          for (int i = 0; i < 24; i++) {
            int x = SCALE_WIDTH + i * BAR_WIDTH;
            tft.drawFastVLine(x, GRAPH_Y + GRAPH_HEIGHT, 5, TFT_BLACK);
            tft.setCursor(x - 3, GRAPH_Y + GRAPH_HEIGHT + 10);
            tft.print(i);
          }

          graphInitialized = true;
        }

        // Rysowanie słupków wykresu
        for (int i = 0; i < 24; i++) {
          int x = SCALE_WIDTH + i * BAR_WIDTH;
          float distance = getHourlyDistance(i);
          int barHeight = map(distance, 0, MAX_DISTANCE, 0, GRAPH_HEIGHT);
          int y = GRAPH_Y + GRAPH_HEIGHT - barHeight;

          // Czyszczenie poprzedniego słupka
          tft.fillRect(x, GRAPH_Y, BAR_WIDTH - 1, GRAPH_HEIGHT, TFT_WHITE);

          // Rysowanie nowego słupka tylko jeśli jest wartość do pokazania
          if (distance > 0) {
            // Rysowanie słupka
            tft.fillRect(x, y, BAR_WIDTH - 1, barHeight, TFT_BLUE);

            // Przygotowanie tekstu
            tft.setTextColor(TFT_BLACK, TFT_WHITE);
            tft.setTextSize(1);
            String label = String(distance, 1);  // Pokazuj jedno miejsce po przecinku

            // Obliczanie szerokości tekstu (przy font size 1, każdy znak ma około 6 pikseli szerokości)
            int textWidth = label.length() * 6;

            // Dokładne centrowanie nad słupkiem
            int labelX = x + ((BAR_WIDTH - textWidth) / 2);
            // Wysokość tekstu przy font size 1 to około 8 pikseli
            int labelY = y - 13;  // 8 pikseli na wysokość tekstu + 5 pikseli odstępu

            // Wyświetlanie tekstu
            if (y > GRAPH_Y + 13) {  // Sprawdź czy jest miejsce nad słupkiem
              tft.setCursor(labelX, labelY);
              tft.print(label);
            }
          }
        }

        // Wyświetlanie aktualnego dystansu
        tft.fillRect(10, 40, 300, 30, TFT_WHITE);  // Czyszczenie obszaru
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(10, 40);
        tft.print("Dystans: ");
        tft.print(hourlyData.currentHourDistance, 2);
        tft.print(" km");

        // Wyświetlanie maksymalnego dystansu
        tft.setCursor(10, 60);
        tft.print("Max: ");
        tft.print(hourlyData.maxDistance, 2);
        tft.print(" km");

        // Wyświetlanie aktualnej godziny
        if (gps.time.isValid()) {
          tft.fillRect(SCREEN_WIDTH / 2, 10, 150, 20, TFT_WHITE);  // Zwiększony prostokąt do czyszczenia
          tft.setCursor(SCREEN_WIDTH / 2, 10);
          int adjustedHour = (gps.time.hour() + 2) % 24;
          int minutes = gps.time.minute();
          int seconds = gps.time.second();

          // Format: "Godz: HH:MM:SS"
          tft.print("Godz: ");
          if (adjustedHour < 10) tft.print("0");
          tft.print(adjustedHour);
          tft.print(":");
          if (minutes < 10) tft.print("0");
          tft.print(minutes);
          tft.print(":");
          if (seconds < 10) tft.print("0");
          tft.print(seconds);
        }

        break;
      }
    case 12:
      {
        static int lastCurrentZoneIndex = -2;

        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(10, 10);
        tft.print("Czas w strefach:");

        // Nazwy wszystkich stref
        int yPos = 40;
        for (int i = 0; i < NUM_ZONES; i++) {
          tft.setCursor(10, yPos);
          tft.print(zones[i].name);
          tft.print(": ");
          yPos += 30;
          if (yPos > SCREEN_HEIGHT - 90) break;
        }

        // Napis "Poza strefami"
        if (yPos <= SCREEN_HEIGHT - 90) {
          tft.setCursor(10, yPos);
          tft.print("Poza strefami: ");
        }

        // Reset yPos dla dynamicznych elementów
        yPos = 40;
        const int DYNAMIC_OFFSET = 90;

        // Zawsze aktualizuj wszystkie czasy
        for (int i = 0; i < NUM_ZONES; i++) {
          int timeX = 10 + 6 * strlen(zones[i].name) + 12 + DYNAMIC_OFFSET;

          // Czyścimy obszar i wyświetlamy czas zawsze
          tft.fillRect(timeX, yPos, SCREEN_WIDTH - timeX - 10, 25, TFT_WHITE);
          tft.setCursor(timeX, yPos);
          tft.print(formatTime(zoneTimeTrack[i].totalTime));

          // Gwiazdkę wyświetlamy dla aktualnej strefy
          if (i == currentZoneIndex) {
            tft.print(" <==--");
          }

          yPos += 30;
          if (yPos > SCREEN_HEIGHT - 90) break;
        }

        // Czas poza strefami - zawsze aktualizuj
        if (yPos <= SCREEN_HEIGHT - 90) {
          int timeX = 10 + 13 * 6 + 12 + DYNAMIC_OFFSET;
          tft.fillRect(timeX, yPos, SCREEN_WIDTH - timeX - 10, 25, TFT_WHITE);

          tft.setCursor(timeX, yPos);
          tft.print(formatTime(zoneTimeTrack[NUM_ZONES].totalTime));
          if (currentZoneIndex == -1) {
            tft.print(" <==--");
          }
        }

        // Zawsze aktualizuj informację o aktualnej strefie
        tft.fillRect(10, SCREEN_HEIGHT - 30, SCREEN_WIDTH - 20, 25, TFT_WHITE);
        tft.setCursor(10, SCREEN_HEIGHT - 30);
        if (currentZoneIndex != -1) {
          tft.print("Aktualna strefa: ");
          tft.print(zones[currentZoneIndex].name);
        } else {
          tft.print("Poza wszystkimi strefami");
        }

        break;
      }

    case 13:
      {
        const int SCALE_WIDTH = 40;  // Szerokość obszaru skali
        const int GRAPH_WIDTH = SCREEN_WIDTH - SCALE_WIDTH;
        const int GRAPH_HEIGHT = 200;
        const int GRAPH_Y = 80;
        const int NUM_BARS = GRAPH_WIDTH / BAR_WIDTH;
        const float TEMP_THRESHOLD = 30.0;         // Próg temperatury dla zmiany koloru (30°C)
        const float MIN_TEMP_CHANGE = 1.0;         // Minimalna zmiana temperatury do aktualizacji wykresu
        const float MIN_MAX_TEMP_PRECISION = 0.1;  // Dokładność dla rejestrowania min/max temperatury
        const float MIN_TEMP = -10.0;              // Minimalna temperatura na wykresie
        const float MAX_TEMP = 50.0;               // Maksymalna temperatura na wykresie

        static float tempHistory[NUM_BARS] = { 0 };
        static int historyIndex = 0;
        static bool isInitialized = false;
        static float maxTemp = MIN_TEMP;        // Zmienna do przechowywania maksymalnej temperatury na wykresie
        static float minTemp = MAX_TEMP;        // Zmienna do przechowywania minimalnej temperatury na wykresie
        static bool isPaused = false;           // Flaga informująca, czy wykres jest wstrzymany
        static float lastTemp = 0;              // Ostatnio zapisana wartość temperatury
        static float globalMaxTemp = MIN_TEMP;  // Globalna maksymalna temperatura
        static float globalMinTemp = MAX_TEMP;  // Globalna minimalna temperatura

        tft.setTextSize(1);
        tft.setTextColor(TFT_BLACK);
        for (float i = MIN_TEMP; i <= MAX_TEMP; i += 10) {
          int y = map(i, MIN_TEMP, MAX_TEMP, GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y);
          tft.drawFastHLine(SCALE_WIDTH - 5, y, 5, TFT_BLACK);
          tft.setCursor(5, y - 4);
          tft.print(i, 0);
        }
        tft.drawFastHLine(SCALE_WIDTH, GRAPH_Y + GRAPH_HEIGHT, GRAPH_WIDTH, TFT_BLACK);

        // Funkcja do rysowania pojedynczego słupka
        auto drawBar = [&](int x, float temp) {
          int barHeight = map(temp, MIN_TEMP, MAX_TEMP, 0, GRAPH_HEIGHT);
          int y = GRAPH_Y + GRAPH_HEIGHT - barHeight;
          tft.fillRect(x + SCALE_WIDTH, GRAPH_Y, BAR_WIDTH - 1, GRAPH_HEIGHT, TFT_WHITE);  // Czyści cały słupek

          // Wybierz kolor słupka w zależności od temperatury
          uint16_t barColor = (temp > TEMP_THRESHOLD) ? TFT_RED : TFT_BLUE;
          tft.fillRect(x + SCALE_WIDTH, y, BAR_WIDTH - 1, barHeight, barColor);
        };

        // Funkcja do rysowania całego wykresu
        auto drawFullGraph = [&]() {
          tft.fillRect(SCALE_WIDTH, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_WHITE);

          for (int i = 0; i < NUM_BARS; i++) {
            int index = (historyIndex - i + NUM_BARS) % NUM_BARS;
            int x = GRAPH_WIDTH - (i + 1) * BAR_WIDTH;
            drawBar(x, tempHistory[index]);
          }
        };

        // Funkcja do aktualizacji wykresu
        auto updateTempGraph = [&](float temp) {
          int oldestIndex = historyIndex;
          tempHistory[historyIndex] = temp;
          historyIndex = (historyIndex + 1) % NUM_BARS;

          // Znajdź lokalną maksymalną i minimalną temperaturę na aktualnym wykresie
          float localMaxTemp = MIN_TEMP;
          float localMinTemp = MAX_TEMP;
          for (int i = 0; i < NUM_BARS; i++) {
            if (tempHistory[i] > localMaxTemp) localMaxTemp = tempHistory[i];
            if (tempHistory[i] < localMinTemp) localMinTemp = tempHistory[i];
          }

          // Przesuń wykres w lewo
          for (int i = 0; i < NUM_BARS; i++) {
            int index = (oldestIndex - i + NUM_BARS) % NUM_BARS;
            int x = GRAPH_WIDTH - (i + 1) * BAR_WIDTH;
            drawBar(x, tempHistory[index]);
          }

          // Aktualizuj wyświetlane wartości min i max
          maxTemp = localMaxTemp;
          minTemp = localMinTemp;
        };

        // Pełne przerysowanie wykresu tylko przy wejściu na ekran
        static int lastScreen = -1;
        if (currentScreen != lastScreen) {
          drawFullGraph();
          lastScreen = currentScreen;
        }

        // Inicjalizacja przy pierwszym uruchomieniu
        if (!isInitialized) {
          for (int i = 0; i < NUM_BARS; i++) {
            tempHistory[i] = MIN_TEMP;
          }
          isInitialized = true;
          maxTemp = MIN_TEMP;
          minTemp = MAX_TEMP;
          lastTemp = MIN_TEMP;
          globalMaxTemp = MIN_TEMP;
          globalMinTemp = MAX_TEMP;
        }

        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(2);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.drawString("Wykres temperatury:", 10, 10);

        // Pobierz aktualną temperaturę
        sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp);
        float currentTemp = temp.temperature;

        // Aktualizuj globalne wartości min i max z dokładnością 0.1 stopnia
        if (abs(currentTemp - lastTemp) > MIN_MAX_TEMP_PRECISION) {
          if (currentTemp > globalMaxTemp) {
            globalMaxTemp = currentTemp;
          }
          if (currentTemp < globalMinTemp) {
            globalMinTemp = currentTemp;
          }
        }

        // Aktualizuj wykres tylko jeśli zmiana temperatury przekracza MIN_TEMP_CHANGE
        if (abs(currentTemp - lastTemp) > MIN_TEMP_CHANGE) {
          updateTempGraph(currentTemp);
          isPaused = false;
          lastTemp = currentTemp;
        } else {
          isPaused = true;
        }

        // Wyświetl aktualną temperaturę jako tekst
        tft.setTextSize(2);
        tft.setCursor(10, 30);
        tft.print("Temp: ");
        tft.print(currentTemp, 1);
        tft.print(" C");

        // Wyświetl globalną maksymalną i minimalną temperaturę
        tft.setTextSize(2);
        tft.setCursor(10, 50);
        tft.print("Max: ");
        tft.print(globalMaxTemp, 1);
        tft.print(" C");
        tft.setCursor(160, 50);
        tft.print("Min: ");
        tft.print(globalMinTemp, 1);
        tft.print(" C");

        // Wyświetl status pauzy jako kolorową kropkę w prawym górnym rogu
        const int DOT_RADIUS = 5;
        const int DOT_X = SCREEN_WIDTH - DOT_RADIUS - 5;  // 5 pikseli od prawej krawędzi
        const int DOT_Y = DOT_RADIUS + 5;                 // 5 pikseli od górnej krawędzi

        if (isPaused) {
          tft.fillCircle(DOT_X, DOT_Y, DOT_RADIUS, TFT_RED);
        } else {
          tft.fillCircle(DOT_X, DOT_Y, DOT_RADIUS, TFT_GREEN);
        }
      }
      break;

    case 14:
      {
        drawTripInterface(tft);
        break;
      }
  }
}





void updateBacklight(float lux) {
  if (currentScreen == 8) {
    // Jeśli ekran wygaszacza (8) jest aktywny, nie aktualizujemy jasności
    return;
  }

  // Normalna aktualizacja jasności na pozostałych ekranach
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= updateInterval) {
    // Oblicz docelową jasność na podstawie odczytu z czujnika
    int newTargetBrightness = map(lux, 0, 100, 100, 800);
    newTargetBrightness = constrain(newTargetBrightness, 150, 800);

    // Ustaw histerezę, np. 15, aby zminimalizować gwałtowne zmiany
    int brightnessThreshold = 15;

    // Aktualizacja targetBrightness tylko jeśli różnica jest znacząca
    if (abs(newTargetBrightness - targetBrightness) >= brightnessThreshold) {
      targetBrightness = newTargetBrightness;
    }

    // Płynna zmiana jasności z krokiem co 10, niezależnie od zmian czujnika
    if (targetBrightness > currentBrightness) {
      currentBrightness = min(currentBrightness + brightnessStep, targetBrightness);
    } else if (targetBrightness < currentBrightness) {
      currentBrightness = max(currentBrightness - brightnessStep, targetBrightness);
    }

    // Aktualizuj jasność LED
    ledcWrite(ledPin, currentBrightness);
    lastUpdateTime = currentTime;
  }
}


void beep() {
  tone(BUZZER_PIN, 1000, 100);
}