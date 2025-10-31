#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include "time.h" // Para sincronizar hora NTP (Network Time Protocol)

// Certificado CA en formato cadena
const char* ca_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"MIID9zCCAt+gAwIBAgIUMUftpGqBKQK8tAHGTB0uql62jIEwDQYJKoZIhvcNAQEL\n" \
"BQAwgYoxCzAJBgNVBAYTAkNPMRIwEAYDVQQIDAlBbnRpb3F1aWExETAPBgNVBAcM\n" \
"CE1lZGVsbGluMQ0wCwYDVQQKDARVZGVBMQwwCgYDVQQLDANJb1QxDjAMBgNVBAMM\n" \
"BWtldmluMScwJQYJKoZIhvcNAQkBFhhrZXZpbi5sb3BlemdAdWRlYS5lZHUuY28w\n" \
"HhcNMjUxMDEwMTYzMDA5WhcNMjYxMDEwMTYzMDA5WjCBijELMAkGA1UEBhMCQ08x\n" \
"EjAQBgNVBAgMCUFudGlvcXVpYTERMA8GA1UEBwwITWVkZWxsaW4xDTALBgNVBAoM\n" \
"BFVkZUExDDAKBgNVBAsMA0lvVDEOMAwGA1UEAwwFa2V2aW4xJzAlBgkqhkiG9w0B\n" \
"CQEWGGtldmluLmxvcGV6Z0B1ZGVhLmVkdS5jbzCCASIwDQYJKoZIhvcNAQEBBQAD\n" \
"ggEPADCCAQoCggEBAK+n1X5pa+uWtt8Mf11GCDjdDG/VGfJUMFynIuuuOZeWkb6Y\n" \
"pQFrVG4eUsFcbN8CN5uXV5hcojoEx6KpRrO42TF2a9wYeGBlkJVj9+mVGFgPQZth\n" \
"6kypCfQOLBwKjsNugIGIiFZhuIVEywMQvZ+XotbpZa+IvIKPCzqvFMCDI6LRr4yR\n" \
"e6CqnAhSyFEhOcacAmadgACMcD1SdBaQkUCcuG8Tftym8D/PAPGrEevC/RuWwI7c\n" \
"tTuhUY8RC8Qmrzm8g3S5YpELo1RGN4fiyRalgnRTjnY+3hpTLJhzJ5NVB4NqJ8Yv\n" \
"2IhrzrLaWeLtrTwirp9B6gbjZUbhgA9+yk+syU0CAwEAAaNTMFEwHQYDVR0OBBYE\n" \
"FAnJhR4xGrCGn5yVM2lPZv6d3XnhMB8GA1UdIwQYMBaAFAnJhR4xGrCGn5yVM2lP\n" \
"Zv6d3XnhMA8GA1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQELBQADggEBAF/0xLKN\n" \
"5jAPZ30/Cyj8Y9o+mVjNiP/VyJFlpqYyeLtVR4WYCKSdMRyLU/w0RstKz6dIwGoe\n" \
"j5JZCcpkY7zzTi/Um0Zg/1NfiVkKZVqmFp0ya6T93A42Q/O17VyfLURphJvVmvcj\n" \
"YTC0n5mIpm4XfJIfx8Qz4c1j3WwN+OmgEJ3JoO7OMbDrLjszSSEZIiCOsyFhDmfS\n" \
"Q/XuETEPzYzDOFZXXNNGm5F+AbLONU+N3uvol/3LjVS7Cd5/Nl5P0aFXT7kJ1aii\n" \
"BnrlwjlNKE1yLMYTtCTg5oqkpmLD5lcLNzYJd+bnLUXHOssmESEvl8M6h5ev8xNx\n" \
"9f4xyB4ew1bh6iw=\n" \
"-----END CERTIFICATE-----\n";

const char* client_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"MIID8jCCAtqgAwIBAgIUfNYDV/0JVenf94KhyjmoTmINT2IwDQYJKoZIhvcNAQEL\n" \
"BQAwgYoxCzAJBgNVBAYTAkNPMRIwEAYDVQQIDAlBbnRpb3F1aWExETAPBgNVBAcM\n" \
"CE1lZGVsbGluMQ0wCwYDVQQKDARVZGVBMQwwCgYDVQQLDANJb1QxDjAMBgNVBAMM\n" \
"BWtldmluMScwJQYJKoZIhvcNAQkBFhhrZXZpbi5sb3BlemdAdWRlYS5lZHUuY28w\n" \
"HhcNMjUxMDEwMjIzOTI5WhcNMjYxMDEwMjIzOTI5WjCBljELMAkGA1UEBhMCQ08x\n" \
"EjAQBgNVBAgMCUFudGlvcXVpYTERMA8GA1UEBwwITWVkZWxsaW4xDTALBgNVBAoM\n" \
"BFVkZUExDDAKBgNVBAsMA0lvVDEZMBcGA1UEAwwQU21hcnRGaXNoLUNsaWVudDEo\n" \
"MCYGCSqGSIb3DQEJARYZbWlndWVsLnB1ZXJ0YUB1ZGVhLmVkdS5jbzCCASIwDQYJ\n" \
"KoZIhvcNAQEBBQADggEPADCCAQoCggEBALcETb9kLIrCtzdpoxbTIcd/lKa6r142\n" \
"uQluGweotINeZNjI6Xk/QsAj2aXu36MKnS6ZbyvrwZl2ZOtPpy+5e7pORzubLfJf\n" \
"xMrKYY2ZNO7kdgWwMl5QQsvx/A+7MWRQMtOKx0zgDH830MSwWWjrpDEDYV738WbL\n" \
"WfJWB5dPez+R7E493xpgqSkpueCfA1Tm0CKpy9JNfEbGxd3lWqAZmsGCLmS01LJ2\n" \
"KSsEKLngaYc0vrb8dBIdZsQ3RDkln1DLPomuI0Lw2ownWmiJyP2kPcWp3iwKSiJ6\n" \
"FnpmU4hOXT5VGrabh2GruDr0yEnLvIFBnArvXvOGiqXZRLK3YKuJvRECAwEAAaNC\n" \
"MEAwHQYDVR0OBBYEFPEax8LyaobA8j3eT6r6pNxGdrjCMB8GA1UdIwQYMBaAFAnJ\n" \
"hR4xGrCGn5yVM2lPZv6d3XnhMA0GCSqGSIb3DQEBCwUAA4IBAQCQtwjhfWW/ZA2r\n" \
"FSl95s5SnwdvNVhnuLbVDD7gznIjaLoXNPe5meMbGGgA6qVO9RM7scXKU26vbAwz\n" \
"6KXrg35qFo6fxBw+bYBI8mX0liWTQ/tgPm6I8pCmpKZMtoxyb9UsafHKGt1AiaXA\n" \
"VVs7MPLk6XCmxfcuJcANLCDc28vdOn0LNuyLVX5SVbXTkHL6EgUM1AdDp46IfBR3\n" \
"2WlwK5VeMawaF+aUnv4lygrGynthEk5D+yzE9WBZpmNf6kq3umzi969U2M+sW8qW\n" \
"dtlexOlgf8nnrMKDhg6LFypfryZkFJbJx9a09MHeNDr4Lb+4rCxUQTiWQpqpbt+m\n" \
"hzUu5AED\n" \
"-----END CERTIFICATE-----\n";

const char* client_key = \
"-----BEGIN PRIVATE KEY-----\n"
"MIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQC3BE2/ZCyKwrc3\n"
"aaMW0yHHf5Smuq9eNrkJbhsHqLSDXmTYyOl5P0LAI9ml7t+jCp0umW8r68GZdmTr\n"
"T6cvuXu6Tkc7my3yX8TKymGNmTTu5HYFsDJeUELL8fwPuzFkUDLTisdM4Ax/N9DE\n"
"sFlo66QxA2Fe9/Fmy1nyVgeXT3s/kexOPd8aYKkpKbngnwNU5tAiqcvSTXxGxsXd\n"
"5VqgGZrBgi5ktNSydikrBCi54GmHNL62/HQSHWbEN0Q5JZ9Qyz6JriNC8NqMJ1po\n"
"icj9pD3Fqd4sCkoiehZ6ZlOITl0+VRq2m4dhq7g69MhJy7yBQZwK717zhoql2USy\n"
"t2Crib0RAgMBAAECggEABevxOg6bNFgthnD4Em59as+Rdz2WHAXIQepanIpUYOXG\n"
"fI8vorktxzrBuZE7kjJvOIlFZBXqOzVt+YEnxy8ItlP/EQop1+xKWUdoFaVN5abl\n"
"vLRhRNTy/FdVkQq9A+JmaprUbQPY06ryKdMM6YyqUN2rol+CY5GT1Pf8oeQz+DAJ\n"
"7259drkW6iGi39eX/S0o/qnxyNbCo857yOEO2+JHsm4PzsvBBdD0uRkscCqOWvKi\n"
"stDVUIC+6RfbOZSYqvm9C99tLRWHkN4KrKGyMH1qgLqT5QV2Y2tws4RV32c4lzbE\n"
"h9jSMg25TCEsXaW/e5cbMzbpSa2m1C0eb/m93J0vMwKBgQDw4/snefmJRfiFTKR+\n"
"TZMmDel1Sy3vum9vL6pB3PbrNFI7OP2P7wFsfBLXDxPJGMUNaC0av7yo6s/fJV9I\n"
"JkDv8lQG5l5tM0o9NsqD0ln6oIRO7Oizu6QnZY9xSNigiJBZmtQeL1Sfe5QcDf16\n"
"3IQzb1zD45bh5Ldz2T2V2Tyj7wKBgQDCfwjXePqyHuKcpYxRJOMBkYd7b5OCVrlM\n"
"A00tSu+MZRMwNw6D5RSxz7t4gACoN7+YgSYipw9h74ZiERR7EtQwVftlKIOevJIT\n"
"4KJ/o3XYZ9aW9gx4qCtVSAXcD+HomhQ5GLfkeSfpI4YJwfqq/ycwrDgudUsMUGFN\n"
"2aKeDKqu/wKBgQCVs5ipd8vz22AOtwqi02LXC15CXm41xQ7rchvEwqVQljtkQo6R\n"
"nF6uJI0wyVXqa1JKeqtNLEr6TrLQYlFMbTl95IMDpraX9n/0etBwC5GPh6Gxytjy\n"
"mXHyadMy+6pkWXobtXSNLazRT+NDbA5TBCfELzjfI9jomVLI1ADJ1y8wUQKBgGUW\n"
"//J2h96ynNgMV40JXg+/oypuCL5S8wZmJOPose9HzZxe0WWOWJ4+uVLdPPox15yd\n"
"a8PRDva5NyCbfgil9bGVzw956kdD/azlMbCjtimfdvKJuluwFK3DvH/vpVxvURmI\n"
"FnRi4HKVqyJBN1dPPQBYUu8aucIRuTFsb9A0BzbjAoGBALxQT4O2bTI+XzdrNnTC\n"
"XdgL9oh0L5fDD3KU7EdosUZT0p1boV6/OsxH76fywIK49uNrZS2OM+M2LnFiPd/d\n"
"zSEuMCOfNmmI5tVM3ho2IMTzXYazECVRm0zgC2JA4D+IDEq36iLo7SPFDeG7VBH1\n"
"g2LWglzKZ4f/y14byNDqCKvO\n"
"-----END PRIVATE KEY-----\n";

//Configurar WiFi
const char* ssid = "iPhone";
const char* password = "Sebas1234";

//Configurar Broker MQTT (IP de la rapsberry Pi)
const char* mqtt_server = "kelogi.local"; //IP local de la rapsberry Pi
const int mqtt_port = 8883; //Puerto mqtt con TLS
//const int mqtt_port = 1883; //Puerto mqtt sin TLS
const char* topic1 = "smartfishtank/237592164/temperature";
const char* topic2 = "smartfishtank/237592164/caudal";
const char* topic3 = "smartfishtank/237592164/turbidez";

//Clientes WiFi y MQTT
WiFiClientSecure wifiClient;
//WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Macro y libería para el sensor de temperatura
#define ONE_WIRE_BUS 4 //Pin de conexión sensor de temperatura
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
float sumTemperatura = 0.0;   // suma acumulada de temperaturas
int countTemperatura = 0;     // cantidad de muestras
float promTemperatura = 0.0;   // promedio calculado cada minuto

// Macro para el sensor de turbidez
#define TURBIDITY_SENSOR_PIN 13 //Pin de conexión sensor de turbidez
int turbidez = 0;

// Macros de actualización para el sensor de flujo
#define FLOW_SENSOR_PIN 21  //Pin de conexión sensor de flujo
unsigned long pulseCount = 0;
unsigned long lastFlowCalcTime = 0;
float flowRate = 0.0;  // en L/min
float MinCaudal = 9999.0;  // Caudal Minimo detectado en un periodo de 1 Min.

// Macros de actualización para el motor paso a paso
// Pines conectados al driver ULN2003 IN1-IN4
#define IN1 19 //Pin de conexión IN1 control de motor paso a paso.
#define IN2 18 //Pin de conexión IN2 control de motor paso a paso.
#define IN3 5 //Pin de conexión IN3 control de motor paso a paso.
#define IN4 17 //Pin de conexión IN4 control de motor paso a paso.

// 2048 pasos por revolución según Datasheet.
const int stepsXRevolution = 2048;

// Crear objeto AccelStepper para el control del motor
AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

// Variable auxiliar para llevar la cuenta del tiempo de cada minuto y actualizar datos a enviar.
unsigned long lastMinuteCheck = 0;
const unsigned long SendPeriod = 60000; // Tiempo de envío de los datos.
bool sendTemp = false;
bool sendTurbidez = false;
bool sendCaudal = false;

// -------- Macro FreeRTOS --------
#define RTOS_delay(ms) vTaskDelay((ms) / portTICK_PERIOD_MS)

TaskHandle_t TaskTempHandle = NULL;
TaskHandle_t TaskTurbidezHandle = NULL;
TaskHandle_t TaskCaudalHandle = NULL;
TaskHandle_t TaskStepperHandle = NULL;
TaskHandle_t TaskComunicationHandle = NULL;

// ISR de flujo
void IRAM_ATTR flowPulseISR() {
  pulseCount++;
}

// Task Lectura sensor de temperatura.
//Con la librería asociada, tomamos la temperatura actual.
void TaskTemperatura(void * parameter) {
  tempSensor.begin();
  for (;;) {
    tempSensor.requestTemperatures();
    float t = tempSensor.getTempCByIndex(0);

    // Acumulamos la temperatura y contamos muestras.
    sumTemperatura += t;
    countTemperatura++;

    // Verificamos si ya pasó el minuto global.
    unsigned long current = millis();
    if ((current - lastMinuteCheck >= SendPeriod) && sendTemp == false) {
      if (countTemperatura > 0) {
        promTemperatura = sumTemperatura / countTemperatura;
        Serial.print("*********************Promedio de temperatura en 1 min: ");
        Serial.print(promTemperatura, 2);
        Serial.println(" °C*********************");
      }

      // Reiniciamos acumuladores para el siguiente minuto.
      sumTemperatura = 0.0;
      countTemperatura = 0;

      sendTemp = true;
    }

    /*Serial.print("Valor de temperatura en el Agua: ");
    Serial.print(t);
    Serial.println(" °C");*/
    RTOS_delay(2000);
  }
}

// Task Lectura sensor de turbidez.
//Realizamos la lectura del conversor analogo a digital.
void TaskTurbidez(void * parameter) {
  for (;;) {
    int raw = analogRead(TURBIDITY_SENSOR_PIN);
    float voltage = raw * (3.3 / 4095.0);
    /*Serial.print("Valor de turbidez en ADC: ");
    Serial.print(raw);
    Serial.print(" Voltaje sensor: ");
    Serial.println(voltage, 2);*/
    turbidez = raw;
    RTOS_delay(2000);
  }
}

// Task Lectura sensor de caudal.
//Con el método de interrupciones contamos los pulsos que genera el sensor.
//Cada pulso representa una fracción de volumen de agua. Con base en los pulsos
//medidos en un intervalo de tiempo, estimamos el caudal en litros por minuto.
void TaskCaudal(void * parameter) {
  // Configuramos la interrupción en el pin del sensor de flujo.
  // Se activa en cada flanco de bajada de la señal (FALLING).
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowPulseISR, FALLING);

  // Guardamos el instante inicial para el cálculo por intervalos de tiempo.
  lastFlowCalcTime = millis();
  lastMinuteCheck = millis();

  for (;;) {
    unsigned long current = millis();

    // Verificamos si ha pasado 1 segundo desde la última medición.
    if (current - lastFlowCalcTime >= 1000) {  // cada segundo
      // Desactivamos la interrupción para leer el conteo de manera segura.
      detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));

      // Guardamos los pulsos acumulados y reiniciamos el contador.
      unsigned long pulses = pulseCount;
      pulseCount = 0;
      lastFlowCalcTime = current;

      // Fórmula de caudal:
      // El sensor POW110D3B no especifica cuántos pulsos equivalen a 1 litro.
      // Se debe calibrar experimentalmente. (se ajustará después).
      float calibrationFactor = 450.0; // ajustar este valor

      // Cálculo del caudal:
      // (pulsos / calibrationFactor) = volumen en litros por segundo.
      // Multiplicamos por 60 para convertirlo a litros por minuto.
      flowRate = (pulses / calibrationFactor) * 60.0; // L/min

      // Actualizamos el mínimo si es menor.
      if (flowRate < MinCaudal) {
        MinCaudal = flowRate;
      }

      /*Serial.print("Valor de pulsos en 1s: ");
      Serial.print(pulses);
      Serial.print(" Flujo estimado: ");
      Serial.print(flowRate, 2);
      Serial.println(" L/min");*/
      // Reactivamos la interrupción para seguir contando.
      attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowPulseISR, FALLING);
    }

    // Actualización variable global minimo caudal en 1 minuto.
    if ((current - lastMinuteCheck >= SendPeriod) && sendCaudal == false) {
      Serial.print("*********************Caudal mínimo en el último minuto: ");
      Serial.print(MinCaudal, 2);
      Serial.println(" L/min*********************");

      // Reiniciamos para el próximo minuto.
      MinCaudal = 9999.0;
      sendCaudal = true;
    }

    if(sendTemp && sendCaudal){
      lastMinuteCheck = current;
      sendTemp = false;
      sendCaudal = false;
      sendTurbidez = false;
    }

    RTOS_delay(200);
  }
}

// Task: Motor Paso a Paso
//Utilizamos objeto creado para utilizar la librería.
void TaskStepper(void * parameter) {
  // configuración inicial del motor
  const int stepsXRevolution = 2048;   // 2048 pasos por revolución según Datasheet.
  const int SetSteps = stepsXRevolution * 2; // 2 vueltas
  const unsigned long FoodCycle = 30000; // 30 s de pausa (Mas adelante según instrucciones de comunicación)

  stepper.setMaxSpeed(800.0);     // definimos velocidad máxima de giro
  stepper.setAcceleration(400.0); // definimos parametro de aceleración            

  for (;;) {
    Serial.println("Inició ciclo de alimentación");

    stepper.setCurrentPosition(0);
    stepper.moveTo(SetSteps); //

    //Ejecutar ciclo de movimiento de alimentación
    while (stepper.distanceToGo() != 0) {
      stepper.run();
      RTOS_delay(1); 
    }

    // Apagar salidas para que quede quieto
    stepper.disableOutputs();
    Serial.println("Fin ciclo de alimentación");

    // Pausa usando RTOS_delay hasta nueva orden de ciclo
    RTOS_delay(FoodCycle);

    // Reactivar salidas antes del siguiente ciclo
    stepper.enableOutputs();
  }
}

//Configuración de la conexión WiFi
void setup_wifi() {
  delay(10);
  Serial.println("Conectando a Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado");
  Serial.println("IP: ");
  Serial.println(WiFi.localIP());
}

//Reconección a WiFi si se perdió la conexión.
void reconnect() {
    Serial.print("vamos a reconectar");
    while (!client.connected()) {
        Serial.print("Conectando a MQTT...");
        if (client.connect("ESP32Client")) {
          Serial.println("Conectado al broker");
        } else {
        Serial.print("Fallo, rc=");
        Serial.print(client.state());
        delay(5000);
      }
    }
}

void syncTime(){
  //Sincronizar hora para TLS
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Esperando sincronización NTP");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }

  Serial.println("\nHora sincronizada");
}

void TLS() {
  //Configurar certificado CA
  wifiClient.setCACert(ca_cert);
  wifiClient.setCertificate(client_cert);
  wifiClient.setPrivateKey(client_key);
  //Verificar conexión TLS directa
  Serial.print("Probando conexión TLS directa...");
  wifiClient.connect(mqtt_server, mqtt_port);
  if (wifiClient.connect(mqtt_server, mqtt_port)) {
    Serial.println("Conexión TLS abierta correctamente.");
    wifiClient.stop(); // cerrarla antes de usar PubSubClient
  } 
  else {
    Serial.println("Fallo al abrir conexión TLS (wifiClient.connect).");
  }
}


// Task: Comunicación MQTT
//Utilizamos objeto creado para utilizar la librería.
void TaskComunication(void * parameter) {

  for (;;) {
    if (!client.connected()) reconnect();
    client.loop();
    char tempStr[10];
    dtostrf(promTemperatura, 4, 2, tempStr);
    client.publish(topic1, tempStr);
    Serial.print("Temperatura enviada: ");
    Serial.println(tempStr);
    char caudStr[10];
    dtostrf(MinCaudal, 4, 2, caudStr);
    client.publish(topic2, caudStr);
    Serial.print("Caudal enviado: ");
    Serial.println(caudStr);
    char turbStr[10];
    dtostrf(turbidez, 4, 2, turbStr);
    client.publish(topic3, turbStr);
    Serial.print("Turbidez enviada: ");
    Serial.println(turbStr);

    RTOS_delay(60000);
  }  
}

void setup() {
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  Serial.begin(115200);

  setup_wifi();
  syncTime();   
  TLS(); //Se abre la conexión con TLS, se prueba y se cierra nuevamente.
  client.setServer(mqtt_server, mqtt_port);
  reconnect();

  //Ejecución de tareas de Hardware en el nucleo 2.
  xTaskCreatePinnedToCore(TaskTemperatura, "TaskTemp", 4096, NULL, 1, &TaskTempHandle, 0);
  xTaskCreatePinnedToCore(TaskTurbidez, "TaskTurb", 4096, NULL, 1, &TaskTurbidezHandle, 0);
  xTaskCreatePinnedToCore(TaskCaudal, "TaskCaudal", 4096, NULL, 1, &TaskCaudalHandle, 0);
  xTaskCreatePinnedToCore(TaskStepper, "TaskStepper", 4096, NULL, 1, &TaskStepperHandle, 0);

  //Ejecución de la comunicación en el nucleo 1.
  xTaskCreatePinnedToCore(TaskComunication, "TaskComunication", 8192, NULL, 1, &TaskComunicationHandle, 1);
}

void loop() {
  // Vacío, todo lo maneja FreeRTOS
}
