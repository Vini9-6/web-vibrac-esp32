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
float alpha = 0.95; // Fator do filtro (0.9-0.99)
float filteredX = 0, filteredY = 0, filteredZ = 0;

// --- Filtro de Kalman Simples ---
struct KalmanFilter {
  float Q;   // Processo variância (quanto o filtro confia no modelo)
  float R;   // Medida variância (ruído do sensor)
  float x;   // Estimativa inicial
  float P;   // Estimativa de erro
  float K;   // Ganho de Kalman

  KalmanFilter(float q, float r, float initial) : Q(q), R(r), x(initial), P(1), K(0) {}

  float update(float measurement) {
    // Previsão
    P = P + Q;

    // Atualização do ganho
    K = P / (P + R);

    // Atualiza estimativa com medida
    x = x + K * (measurement - x);

    // Atualiza estimativa do erro
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
    Serial.println("MPU6050 não encontrado!");
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
  Serial.printf("Offsets: X=%.4f, Y=%.4f, Z=%.4f\n", accelX, accelY, accelZ);

  // Inicializa filtros Kalman com o valor do offset para estabilidade inicial
  kalmanX.x = 0;
  kalmanY.x = 0;
  kalmanZ.x = 0;
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    client.setTimeout(1000);
    String req = client.readStringUntil('\r');
    client.flush();

    if (mpuOk) {
      mpu.getEvent(&a, &g, &temp);

      // Filtro passa-alta
      filteredX = alpha * (filteredX + a.acceleration.x - accelX);
      filteredY = alpha * (filteredY + a.acceleration.y - accelY);
      filteredZ = alpha * (filteredZ + a.acceleration.z - accelZ);

      accelX = a.acceleration.x;
      accelY = a.acceleration.y;
      accelZ = a.acceleration.z;

      // Aplica filtro de Kalman para suavizar os dados
      filteredX = kalmanX.update(filteredX);
      filteredY = kalmanY.update(filteredY);
     // filteredZ = kalmanZ.update(filteredZ);
    }

    // Medição HC-SR04
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
      client.printf("{\"vibX\": %.4f, \"vibY\": %.4f, \"vibZ\": %.4f, \"dist\": %.2f}",
        filteredX, filteredY, filteredZ, cm);
      client.stop();
      return;
    }

    // Página HTML com gráficos + botão e contador (igual seu código anterior)
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

  const chartX = new Chart(ctxX, { type: 'line', data: { datasets: [{ label: 'Vibração X', borderColor: 'red', data: [] }] }, options: chartOptions });
  const chartY = new Chart(ctxY, { type: 'line', data: { datasets: [{ label: 'Vibração Y', borderColor: 'green', data: [] }] }, options: chartOptions });
  const chartZ = new Chart(ctxZ, { type: 'line', data: { datasets: [{ label: 'Vibração Z', borderColor: 'blue', data: [] }] }, options: chartOptions });

  async function updateCharts() {
    try {
      const res = await fetch('/data');
      const data = await res.json();
      const time = new Date().toLocaleTimeString();

      updateChart(chartX, data.vibX, time);
      updateChart(chartY, data.vibY, time);
      updateChart(chartZ, data.vibZ, time);
    } catch (err) {
      console.error("Erro:", err);
      setTimeout(updateCharts, 1000);
    }
  }

  function updateChart(chart, value, time) {
    chart.data.labels.push(time);
    chart.data.datasets[0].data.push(value);
    if (chart.data.labels.length > 30) {
      chart.data.labels.shift();
      chart.data.datasets[0].data.shift();
    }
    chart.update();
  }

  setInterval(updateCharts, 100);

  let collecting = false;
  let collectedData = [];

  document.getElementById('btnDownload').onclick = () => {
    if (collecting) return;
    collectedData = [];
    document.getElementById('contador').innerText = 'Contagem: 10s';
    collecting = true;
    let secondsLeft = 10;

    const collectInterval = setInterval(async () => {
      try {
        const res = await fetch('/data');
        const data = await res.json();
        const time = new Date().toLocaleTimeString();
        collectedData.push([time, data.vibX.toFixed(4), data.vibY.toFixed(4), data.vibZ.toFixed(4)]);
      } catch (err) {
        console.error('Erro ao coletar dados:', err);
      }
    }, 100);

    const countdown = setInterval(() => {
      secondsLeft--;
      document.getElementById('contador').innerText = `Contagem: ${secondsLeft}s`;
      if (secondsLeft <= 0) {
        clearInterval(countdown);
        clearInterval(collectInterval);
        document.getElementById('contador').innerText = 'Download iniciando...';

        let csvContent = 'data:text/csv;charset=utf-8,Hora,VibX,VibY,VibZ\n';
        collectedData.forEach(row => {
          csvContent += row.join(',') + '\n';
        });

        const link = document.createElement('a');
        link.setAttribute('href', encodeURI(csvContent));
        link.setAttribute('download', 'dados_vibracao_10s.csv');
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);

        collecting = false;
        document.getElementById('contador').innerText = '';
      }
    }, 1000);
  };
</script>
</body></html>
)rawliteral");
    client.stop();
  }
}
 
