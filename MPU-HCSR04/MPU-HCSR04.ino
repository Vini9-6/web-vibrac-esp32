#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- Access Point ---
const char* ssid = "ESP32-Rede";
const char* password = "12345678";
WiFiServer server(80);

// --- HC-SR04 ---
#define trigPin 12
#define echoPin 13
long duration;
float cm = 0;

// --- MPU6050 ---
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
bool mpuOk = false;
float accelX = 0, accelY = 0, accelZ = 0;

// --- Filtro Passa-Alta ---
float alpha = 0.95;
float filteredX = 0, filteredY = 0, filteredZ = 0;

// --- Filtro de Kalman Simples ---
struct KalmanFilter {
  float Q, R, x, P, K;
  KalmanFilter(float q, float r, float initial) : Q(q), R(r), x(initial), P(1), K(0) {}
  float update(float measurement) {
    P = P + Q;
    K = P / (P + R);
    x = x + K * (measurement - x);
    P = (1 - K) * P;
    return x;
  }
};

KalmanFilter kalmanX(0.001, 0.05, 0);
KalmanFilter kalmanY(0.001, 0.05, 0);
KalmanFilter kalmanZ(0.001, 0.05, 0);

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  if (mpu.begin()) {
    mpuOk = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    calibrateMPU6050();
    Serial.println("MPU6050 configurado para alta sensibilidade!");
  } else {
    Serial.println("MPU6050 n\u00e3o encontrado!");
  }

  WiFi.softAP(ssid, password);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  server.begin();
}

void calibrateMPU6050() {
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 100; i++) {
    mpu.getEvent(&a, &g, &temp);
    sumX += a.acceleration.x;
    sumY += a.acceleration.y;
    sumZ += a.acceleration.z;
    delay(10);
  }
  accelX = sumX / 100;
  accelY = sumY / 100;
  accelZ = sumZ / 100;
  kalmanX.x = 0; kalmanY.x = 0; kalmanZ.x = 0;
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    client.setTimeout(1000);
    String req = client.readStringUntil('\r');
    client.flush();

    if (mpuOk) {
      mpu.getEvent(&a, &g, &temp);
      filteredX = alpha * (filteredX + a.acceleration.x - accelX);
      filteredY = alpha * (filteredY + a.acceleration.y - accelY);
      filteredZ = alpha * (filteredZ + a.acceleration.z - accelZ);
      accelX = a.acceleration.x;
      accelY = a.acceleration.y;
      accelZ = a.acceleration.z;
      filteredX = kalmanX.update(filteredX);
      filteredY = kalmanY.update(filteredY);
      filteredZ = kalmanZ.update(filteredZ);
    }

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH, 30000);
    cm = (duration / 2.0) / 29.1;

    if (req.indexOf("GET /data") >= 0) {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Connection: close");
      client.println();
      client.printf("{\"vibX\": %.4f, \"vibY\": %.4f, \"vibZ\": %.4f, \"dist\": %.2f}", filteredX, filteredY, filteredZ, cm);
      client.stop();
      return;
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println(R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'>
<title>ESP32 - Vibração</title>
<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>
<style>
  body { font-family: sans-serif; margin: 20px; }
  .chart-container { width: 100%; max-width: 800px; margin: 0 auto; }
  canvas { background: #fff; border: 1px solid #ddd; margin-bottom: 20px; }
  #contador { font-weight: bold; margin-top: 10px; text-align: center; }
  #btnDownload { display: block; margin: 10px auto 30px auto; padding: 10px 20px; font-size: 16px; }
</style>
</head><body>
<h2 style="text-align:center;">Monitoramento de Vibração</h2>
<div class="chart-container">
  <canvas id="vibChartX"></canvas>
  <canvas id="vibChartY"></canvas>
  <canvas id="vibChartZ"></canvas>
  <canvas id="distChart"></canvas>
</div>
<button id="btnDownload">Iniciar contagem e exportar dados (10s)</button>
<div id="contador"></div>
<script>
const chartOptions = {
  responsive: true,
  animation: { duration: 0 },
  scales: {
    y: { min: -0.5, max: 0.5, ticks: { stepSize: 0.1 } },
    x: { display: false }
  }
};
const ctxX = document.getElementById('vibChartX').getContext('2d');
const ctxY = document.getElementById('vibChartY').getContext('2d');
const ctxZ = document.getElementById('vibChartZ').getContext('2d');
const ctxDist = document.getElementById('distChart').getContext('2d');
const chartX = new Chart(ctxX, { type: 'line', data: { datasets: [{ label: 'Vibração X', borderColor: 'red', data: [] }] }, options: chartOptions });
const chartY = new Chart(ctxY, { type: 'line', data: { datasets: [{ label: 'Vibração Y', borderColor: 'green', data: [] }] }, options: chartOptions });
const chartZ = new Chart(ctxZ, { type: 'line', data: { datasets: [{ label: 'Vibração Z', borderColor: 'blue', data: [] }] }, options: chartOptions });
const chartDist = new Chart(ctxDist, {
  type: 'line',
  data: { datasets: [{ label: 'Distância (cm)', borderColor: 'orange', data: [] }] },
  options: {
    responsive: true,
    animation: { duration: 0 },
    scales: {
      y: { min: 0, max: 100, ticks: { stepSize: 10 } },
      x: { display: false }
    }
  }
});
function updateChart(chart, value, time) {
  chart.data.labels.push(time);
  chart.data.datasets[0].data.push(value);
  if (chart.data.labels.length > 30) {
    chart.data.labels.shift();
    chart.data.datasets[0].data.shift();
  }
  chart.update();
}
async function updateCharts() {
  try {
    const res = await fetch('/data');
    const data = await res.json();
    const time = new Date().toLocaleTimeString();
    updateChart(chartX, data.vibX, time);
    updateChart(chartY, data.vibY, time);
    updateChart(chartZ, data.vibZ, time);
    updateChart(chartDist, data.dist, time);
  } catch (err) {
    console.error("Erro:", err);
    setTimeout(updateCharts, 1000);
  }
}
setInterval(updateCharts, 100);
</script>
</body></html>
)rawliteral");
    client.stop();
  }
}
