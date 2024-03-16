#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#include <driver/adc.h>
#include <driver/ledc.h>

rcl_subscription_t subscription_pwm_cycle;
rcl_publisher_t publisher_setpoint;
std_msgs__msg__Float32 msg_sp;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_1;  // Changed timer name to timer_1
rcl_timer_t timer_2;  // Changed timer name to timer_2
std_msgs__msg__Float32 pwm_msg;

//#define ADC_RESOLUTION 12
//#define ADC_MAX_VALUE (1 << ADC_RESOLUTION)
//#define VOLTAGE_MAX 3.3

//#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8
#define PWM_FREQUENCY 5000

const int encoderPinA = 21;
//const int encoderPinB = 18;
#define LED_PIN 15
int ena = 14;
int in1 = 27;
int in2 = 26;
int signo = 0;

//Variables PID, Ganancias en tiempo discreto
double cv, cv1, error, error1, error2, rpm, w; //_1 tiempo pasado

/*double Kp = 0.507;
double Ki = 0.644;
double Kd = 1.478;*/

double Kp = 0.1;
double Ki = 0.1;
double Kd = 0.0;
double Tm = 0.05;
volatile int pulsos = 0;
unsigned long previousMillis = 0;
long interval = 50;
float sp;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(100);
  }
}

void interrupcion() {
  pulsos++;
}

//Callback to listen to
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 * msg_in = (const std_msgs__msg__Float32 *)msgin;
  sp = msg_in->data;                                          //Volvemos pwm_cycle y lo ponemos en una variable global381976730705846e
  //RCSOFTCHECK(rcl_publish(&publisher_setpoint, &sp, NULL));

  /*if (pwm_cycle >= 0) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  signo = 0;                                                  //Signo es 0 cuando la senal de entrada es positiva
  } else {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  signo = 1;                                                  //Signo es 1 cuando la senal de entrada es negativa
  }*/

  //sp = abs(sp);
}

  
//Timer para calcular PID
void timer_callback_1(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  unsigned long currentMillis = millis();

  //Para contar los pulsos
  if ((currentMillis - previousMillis) >= interval) {
  previousMillis = currentMillis;
  rpm = pulsos * 176 / 98;            //RPM (dv)
  w = (rpm * 2 * PI) / 60;            //Velocidad Angular (pv)
  pulsos = 0;
  }


  //Calculo de error y ganancia
  error = ((sp * 18.5)) - w;
  //error = ((sp * 18.5) - w)/ 1.75;


  //Ecuacion de diferencias
  //cv = (Kp + Kd / Tm) * error + (-Kp + Ki * Tm - 2 * Kd / Tm) * error1 + (Kd / Tm) * error2;  //Ecuacion de diferencias del video
  //cv =  ((Kp * error) + (Kd / Tm) * (error - error1) + Ki * Tm * (error + error1)); //Ecuacion de proporcionada por Manchester PID
  cv =  cv1 + ((Kp * error) + Ki * Tm * (error + error1)); // PI
  //cv =  cv1 + ((Kp * error) + (Kd / Tm) * (error - error1)); //PD
  //cv = Kp * error; //P
  cv1 = cv;
  error2 = error1;
  error1 = error;

  /*//Saturar la salida PID
  if (cv > 500.0) {
  cv = 500.0;
  } else if (cv < 30.0) {
  cv = 30.0;
  }
  ledcWrite(PWM_CHANNEL, cv * (255 / 500));
  */


  if (sp >= 0.0f) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);//publisher_setpoint;
  signo = 0;                                                  //Signo es 0 cuando la senal de entrada es positiva
  } else {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  signo = 1;                                                  //Signo es 1 cuando la senal de entrada es negativa
  }

  if (cv >= 1.0){
    cv = 1.0;
  } else if (cv <= -1.0){
    cv = -1.0;
  }
  double pwm_sal = cv;



  
  
  //msg_sp.data = sp;
  ledcWrite(PWM_CHANNEL, (int)(abs((pwm_sal)/1.85f)*255.0f));
  //delay(50);
  //RCSOFTCHECK(rcl_publish(&publisher_setpoint, &cv, NULL));
  
}

void timer_callback_2(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  float cvf = (cv/1.85f)*255.0f;
  RCSOFTCHECK(rcl_publish(&publisher_setpoint, &cvf, NULL));
  
}

void setup() {
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(ena, PWM_CHANNEL);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(21), interrupcion, RISING);
  set_microros_transports();
  //pinMode(encoderPinB, INPUT);
  //pinMode(ena, OUTPUT);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "controller_node", "", &support));

  RCCHECK(rclc_publisher_init_default(&publisher_setpoint, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "micro_ros_esp32/sp"))

  RCCHECK(rclc_subscription_init_default(&subscription_pwm_cycle, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/micro_ros_esp32/setpoint"));                                        //Nombre del topico

  //const unsigned int timer_1_timeout = 10;
  RCCHECK(rclc_timer_init_default(&timer_1, &support, RCL_MS_TO_NS(50), timer_callback_1));
  RCCHECK(rclc_timer_init_default(&timer_2, &support, RCL_MS_TO_NS(10), timer_callback_2));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscription_pwm_cycle, &pwm_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_2));

  pwm_msg.data = 0.0;                                             //Se utiliza para publicar
  msg_sp.data = 0.0f;
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
