#define LED_PIN 2

//Decare task handle
TaskHandle_t BlinkTaskHandle=NULL;

void BlinkTask(void *parameter){
 for(;;){//Infinite loop
  digitalWrite(LED_PIN,HIGH);
  Serial.println("BlinkTask: LED ON");
  vTaskDelay(1000/portTICK_PERIOD_MS);
  digitalWrite(LED_PIN,LOW);
  Serial.println("BlinkTask: LED OFF");
  vTaskDelay(1000/portTICK_PERIOD_MS);
  Serial.println("BlinkTask running on core");
  Serial.println(xPortGetCoreID());

}
}

void setup() {
 Serial.begin(115200);
 pinMode(LED_PIN, OUTPUT);  
 xTaskCreatePinnedToCore(
  BlinkTask,
  "BlinkTask",
  10000, // Stack size (bytes)
  NULL,
  1,
  &BlinkTaskHandle,
  1
 );
}

void loop() {
  // put your main code here, to run repeatedly:

}
