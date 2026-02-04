#include <Encoder.h>

float targetRPM = 135.0;
int motorSpeed = 0;

Encoder myEnc(2, 3);

const int IN1 = 12;
const int IN2 = 13;
const int ENA = 10;

const int E_COUNT = 11;
const int DE_COUNT = 7;

float e_points[E_COUNT];
float de_points[DE_COUNT];

float tabelaMamdani[E_COUNT * DE_COUNT];
bool tabelaInitializata = false;

bool init_in = false;
float ponderi_in[4] = {0, 0, 0, 0};
int indici_in[2] = {0, 0};

float gamma = 0.05;

long oldPosition = 0;
unsigned long lastTime = 0;
unsigned long startTime = 0;

const float encoderPulsesPerRev = 11.0;
const float gearRatio = 30.0;

void initPoints(float* points, int count) {
  for (int i = 0; i < count; i++) {
    points[i] = -1.0 + 2.0 * i / (count - 1);
  }
}

float TabelaDeDecizieMamdani(
    float epsilon, float e, float de,
    bool &init_in, float* tabela_in,
    float* ponderi_in, int* indici_in,
    int e_count, int de_count,
    float* e_points, float* de_points,
    float gamma,
    bool &init_out,
    float* tabela_out,
    float* ponderi_out,
    int* indici_out
  ) {

  if (!init_in) {
    init_out = true;
    for (int i = 0; i < e_count * de_count; i++) {
      tabela_out[i] = 0.0;
    }
  } else {
    init_out = init_in;
    for (int i = 0; i < e_count * de_count; i++) {
      tabela_out[i] = tabela_in[i];
    }
  }

  int i_a = indici_in[0];
  int j_a = indici_in[1];

  if (i_a < 0) i_a = 0;
  if (j_a < 0) j_a = 0;
  if (i_a >= e_count) i_a = e_count - 1;
  if (j_a >= de_count) j_a = de_count - 1;

  int index = i_a * de_count + j_a;

  if (i_a < e_count - 1 && j_a < de_count - 1) {
    tabela_out[index] += gamma * ponderi_in[0] * epsilon;
    tabela_out[index + de_count] += gamma * ponderi_in[1] * epsilon;
    tabela_out[index + 1] += gamma * ponderi_in[2] * epsilon;
    tabela_out[index + de_count + 1] += gamma * ponderi_in[3] * epsilon;
  } else if (i_a == e_count - 1 && j_a == de_count - 1) {
    tabela_out[index] += gamma * epsilon;
  } else if (i_a == e_count - 1) {
    tabela_out[index] += gamma * 0.5 * epsilon;
    tabela_out[index + 1] += gamma * 0.5 * epsilon;
  } else if (j_a == de_count - 1) {
    tabela_out[index] += gamma * 0.5 * epsilon;
    tabela_out[index + de_count] += gamma * 0.5 * epsilon;
  } else {
    tabela_out[index] += gamma * epsilon;
  }

  int i = 0;
  while (i < e_count - 1 && e_points[i + 1] <= e) i++;
  int j = 0;
  while (j < de_count - 1 && de_points[j + 1] <= de) j++;

  indici_out[0] = i;
  indici_out[1] = j;

  index = i * de_count + j;

  float w_ij, w_i1j, w_ij1, w_i1j1;
  float u = 0.0;

  if (i < e_count - 1 && j < de_count - 1) {
    float e_range = e_points[i + 1] - e_points[i];
    float de_range = de_points[j + 1] - de_points[j];
    w_ij = (e_points[i + 1] - e) * (de_points[j + 1] - de) / (e_range * de_range);
    w_i1j = (e - e_points[i]) * (de_points[j + 1] - de) / (e_range * de_range);
    w_ij1 = (e_points[i + 1] - e) * (de - de_points[j]) / (e_range * de_range);
    w_i1j1 = (e - e_points[i]) * (de - de_points[j]) / (e_range * de_range);

    ponderi_out[0] = w_ij;
    ponderi_out[1] = w_i1j;
    ponderi_out[2] = w_ij1;
    ponderi_out[3] = w_i1j1;

    u = w_ij * tabela_out[index]
      + w_i1j * tabela_out[index + de_count]
      + w_ij1 * tabela_out[index + 1]
      + w_i1j1 * tabela_out[index + de_count + 1];
  } else if (i == e_count - 1 && j == de_count - 1) {
    ponderi_out[0] = 1;
    ponderi_out[1] = 0;
    ponderi_out[2] = 0;
    ponderi_out[3] = 0;
    u = tabela_out[index];
  } else if (i == e_count - 1) {
    float w1 = (de_points[j + 1] - de) / (de_points[j + 1] - de_points[j]);
    float w2 = 1 - w1;
    ponderi_out[0] = w1;
    ponderi_out[1] = w2;
    ponderi_out[2] = 0;
    ponderi_out[3] = 0;
    u = w1 * tabela_out[index] + w2 * tabela_out[index + 1];
  } else if (j == de_count - 1) {
    float w1 = (e_points[i + 1] - e) / (e_points[i + 1] - e_points[i]);
    float w2 = 1 - w1;
    ponderi_out[0] = w1;
    ponderi_out[1] = 0;
    ponderi_out[2] = w2;
    ponderi_out[3] = 0;
    u = w1 * tabela_out[index] + w2 * tabela_out[index + de_count];
  } else {
    ponderi_out[0] = 1;
    ponderi_out[1] = 0;
    ponderi_out[2] = 0;
    ponderi_out[3] = 0;
    u = tabela_out[index];
  }

  return u;
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENA, 0);
  oldPosition = 0;
  lastTime = millis();
  startTime = millis();

  initPoints(e_points, E_COUNT);
  initPoints(de_points, DE_COUNT);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - startTime >= 10000) {
    analogWrite(ENA, 0);
    return;
  }

  long newPosition = myEnc.read();

  if (currentTime - lastTime >= 100) {
    long deltaTicks = newPosition - oldPosition;
    float ticksPerSec = (deltaTicks * 1000.0) / (currentTime - lastTime);
    float rpm = (ticksPerSec / (encoderPulsesPerRev * gearRatio)) * 60.0;

    static float lastError = 0;
    float error = targetRPM - rpm;

    float e_norm = constrain(error / targetRPM, -1, 1);
    float de_norm = constrain((error - lastError) / 0.1 / targetRPM, -1, 1);
    float epsilon = error;

    float u = TabelaDeDecizieMamdani(
      epsilon, e_norm, de_norm,
      init_in, tabelaMamdani,
      ponderi_in, indici_in,
      E_COUNT, DE_COUNT,
      e_points, de_points,
      gamma,
      init_in,
      tabelaMamdani,
      ponderi_in,
      indici_in
    );

    motorSpeed += (int)u;
    motorSpeed = constrain(motorSpeed, 0, 255);

    analogWrite(ENA, motorSpeed);

    lastError = error;
    oldPosition = newPosition;
    lastTime = currentTime;

    Serial.print(" "); Serial.print(currentTime / 1000.0);
    Serial.print(" "); Serial.print(targetRPM);
    Serial.print(" "); Serial.print(rpm);
    Serial.print(" "); Serial.println(motorSpeed);
  }
}
