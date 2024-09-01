#include <../include/secrets.h>
#include <Adafruit_SGP30.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <M5Unified.h>
#include <M5UnitEnv.h>
#include <WiFi.h>

InfluxDBClient influxDbClient(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET,
                              INFLUXDB_TOKEN);

SHT4X sht4x;
BMP280 bmp280;
Adafruit_SGP30 sgp30;

Point sht4xPoint("sht4x");
Point bmp280Point("bmp280");
Point sgp30Point("sgp30");

hw_timer_t *timer = NULL;
SemaphoreHandle_t syncSemaphore;

void IRAM_ATTR onTimer() {
  /* release semaphore */
  xSemaphoreGiveFromISR(syncSemaphore, NULL);
}

boolean connectWiFi() {
  Serial.print("WiFi Connecting");

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int count = 1;
  while (WiFi.status() != WL_CONNECTED) {
    if (count >= 100) { // 10sec
      WiFi.disconnect(true, true);
      Serial.println("");
      Serial.println("WiFi Connection Error.");
      Serial.println(WiFi.status());
      return false;
    }

    Serial.print(".");
    count++;
    delay(100);
  }

  Serial.println("");
  Serial.println("WiFi Connected.");

  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("Subnet: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("DNS: ");
  Serial.println(WiFi.dnsIP());
  Serial.println("");

  return true;
}

void initializeSensors() {
  // fetch I2C pins
  auto pin_sda = M5.getPin(m5::port_a_scl);
  auto pin_scl = M5.getPin(m5::port_a_sda);

  // initialize bmp280
  if (!bmp280.begin(&Wire, BMP280_I2C_ADDR, pin_scl, pin_sda, 400000U)) {
    Serial.println("[!] couldn't find BMP280");
  }

  // NOTE: default settings
  bmp280.setSampling(BMP280::MODE_NORMAL,     /* Operating Mode. */
                     BMP280::SAMPLING_X2,     /* Temp. oversampling */
                     BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     BMP280::FILTER_X16,      /* Filtering. */
                     BMP280::STANDBY_MS_500); /* Standby time. */

  // initialize sht4x
  if (!sht4x.begin(&Wire, SHT40_I2C_ADDR_44, pin_scl, pin_sda, 400000U)) {
    Serial.println("[!] couldn't find SHT4X");
  }

  // NOTE: 精度が上がる代わりに計測時間が長くなる
  sht4x.setPrecision(SHT4X_HIGH_PRECISION);
  // NOTE: センサ結露時用のヒーターは無効化
  sht4x.setHeater(SHT4X_NO_HEATER);

  // initialize sht4x
  if (!sgp30.begin(&Wire, true)) {
    Serial.println("[!] couldn't find SGP30");
  }

  sgp30.softReset();
  sgp30.IAQinit();
  // NOTE: 定義すると前回のキャリブレーション値を流用できるが、
  //       1週間以内のキャリブレ値が使えない場合は明示的に指定しない方が良い
  //       指定しない場合は自動キャリブレーションに12時間かかり、
  //       その間はTVOC: 0, eCO2: 400が返る
  // sgp.setIAQBaseline(0, 0);
}

// initialize timer and create semaphore
void initializeTimer() {
  syncSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true); // 80 clock = 1us
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000 * 1000 * 60,
                  true); // 1000us * 1000 * 60 = 1min
  timerAlarmEnable(timer);
}

void setup() {
  // initialize M5
  auto cfg = M5.config();
  M5.begin(cfg);

  // NOTE: influxDBにSSL検証で失敗して接続できないので一次的に無効化
  //       多分SSLをCloudflareでやってるせいでcertが違う
  influxDbClient.setInsecure();

  sht4xPoint.addTag("sensor", "ENV.IV");
  bmp280Point.addTag("sensor", "ENV.IV");
  sgp30Point.addTag("sensor", "TVOC/eCO2");

  initializeSensors();
  initializeTimer();
}

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity =
      216.7f * ((humidity / 100.0f) * 6.112f *
                exp((17.62f * temperature) / (243.12f + temperature)) /
                (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled =
      static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
}

void loop() {
  xSemaphoreTake(syncSemaphore, portMAX_DELAY);

  Serial.println("initialize point fields.");
  sht4xPoint.clearFields();
  bmp280Point.clearFields();
  sgp30Point.clearFields();

  connectWiFi();

  timeSync(TZ_INFO, "ntp.nict.jp");

  float temp, hum;
  if (sht4x.update()) {
    temp = sht4x.cTemp;
    hum = sht4x.humidity;
    sht4xPoint.addField("temperature", temp);
    sht4xPoint.addField("humidity", hum);
  }

  if (bmp280.update()) {
    bmp280Point.addField("temperature", bmp280.cTemp);
    bmp280Point.addField("pressure", bmp280.pressure);
    bmp280Point.addField("altitude", bmp280.altitude);
  }

  // NOTE: 絶対湿度でTVOCを補正する
  sgp30.setHumidity(getAbsoluteHumidity(temp, hum));

  if (sgp30.IAQmeasure()) {
    sgp30Point.addField("tvoc", sgp30.TVOC);
    sgp30Point.addField("eco2", sgp30.eCO2);

    uint16_t eco2_base, tvoc_base;
    sgp30.getIAQBaseline(&eco2_base, &tvoc_base);
    sgp30Point.addField("eco2_base", eco2_base);
    sgp30Point.addField("tvoc_base", tvoc_base);
  }

  Serial.println("Write Point");
  Serial.println(sht4xPoint.toLineProtocol());
  Serial.println(bmp280Point.toLineProtocol());
  Serial.println(sgp30Point.toLineProtocol());
  Serial.println(influxDbClient.writePoint(sht4xPoint));
  Serial.println(influxDbClient.writePoint(bmp280Point));
  Serial.println(influxDbClient.writePoint(sgp30Point));
  Serial.println("");

  Serial.println("Disconnect WiFi");
  WiFi.disconnect(true, true);
  Serial.println("");

  Serial.println("waiting for next loop...");
  Serial.println("");
}
