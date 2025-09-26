#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <AccelStepper.h>

// Macro y libería para el sensor de temperatura
#define ONE_WIRE_BUS 4 //Pin de conexión sensor de temperatura
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
float sumTemperatura = 0.0;   // suma acumulada de temperaturas
int countTemperatura = 0;     // cantidad de muestras
float promTemperatura = 0.0;   // promedio calculado cada minuto

// Macro para el sensor de turbidez
#define TURBIDITY_SENSOR_PIN 13 //Pin de conexión sensor de turbidez

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

    Serial.print("Valor de temperatura en el Agua: ");
    Serial.print(t);
    Serial.println(" °C");
    RTOS_delay(2000);
  }
}

// Task Lectura sensor de turbidez.
//Realizamos la lectura del conversor analogo a digital.
void TaskTurbidez(void * parameter) {
  for (;;) {
    int raw = analogRead(TURBIDITY_SENSOR_PIN);
    float voltage = raw * (3.3 / 4095.0);
    Serial.print("Valor de turbidez en ADC: ");
    Serial.print(raw);
    Serial.print(" Voltaje sensor: ");
    Serial.println(voltage, 2);
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

      Serial.print("Valor de pulsos en 1s: ");
      Serial.print(pulses);
      Serial.print(" Flujo estimado: ");
      Serial.print(flowRate, 2);
      Serial.println(" L/min");
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

void setup() {
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  
  //Vamos a resrvar el nucleo 2 para la comunicación.
  xTaskCreatePinnedToCore(TaskTemperatura, "TaskTemp", 4096, NULL, 1, &TaskTempHandle, 1);
xTaskCreatePinnedToCore(TaskTurbidez, "TaskTurb", 4096, NULL, 1, &TaskTurbidezHandle, 1);
  xTaskCreatePinnedToCore(TaskCaudal, "TaskCaudal", 4096, NULL, 1, &TaskCaudalHandle, 1);
  xTaskCreatePinnedToCore(TaskStepper, "TaskStepper", 4096, NULL, 1, &TaskStepperHandle, 1);
}

void loop() {
  // Vacío, todo lo maneja FreeRTOS
}
