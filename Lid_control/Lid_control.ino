
#include <Wire.h>
#include "DFRobot_INA219.h"
#include <SimpleKalmanFilter.h>
#include <EEPROM.h>
#include <CAN.h>
#include <stdint.h>
#include <time.h>
SimpleKalmanFilter filter(5, 5, 0.001);

/*======================== Pin denfine ====================*/
#define L_PWM 17
#define R_PWM 16
#define LR_EN 18

#define BUTTON_CLOSE 19      // The button performs the opening operation on-site (NC)

#define SCL 33            // INA219 sensor
#define SDA 32

#define LIMIT_SWICTH_OPEN 34       // Limit switch OPEN 
#define LIMIT_SWICTH_CLOSE 35      // Limit switch CLOSE

#define TX_GPIO_NUM   21  // Connects to CAN_TX
#define RX_GPIO_NUM   22  // Connects to CAN_RX

/*=========================================================*/

#define STEP 9569 // 4900  // 9580
#define U_STEP 2

#define STOP_CMD 0
#define OPEN_CMD 1
#define CLOSE_CMD -1


typedef enum LidState
{
  CLOSED = 0,
  CLOSING = 1,
  OPENED = 2,
  OPENING = 3,
  STUCK_WHILE_CLOSING = 102,
  STUCK_WHILE_OPENING = 103
} LidState;

LidState lid_state = CLOSED;

// Current sensor
DFRobot_INA219_IIC     ina219(&Wire, INA219_I2C_ADDRESS1);
// Revise the following two paramters according to actual reading of the INA219 and the multimeter

// For linearly calibration
float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;
float previous_yn = 200.0;  
float abs_previous;

bool limit_sw_open = 0;
bool limit_sw_close = 0;
bool button_close = 0;

unsigned long time_ms = 0;

int16_t duty=0;       // Duty PWM (0-255)
int16_t duty_defauct=200;

uint8_t open_speed=0, close_speed=0;
uint8_t new_open_speed=0, new_close_speed=0;

bool flag = false;
float xn1 = 0;
float yn1 = 0;
float pre = 0.0;

int current,voltage;
int lid_cmd = STOP_CMD;
int overload = 0;

uint16_t new_threshold;
uint16_t threshold_current;
uint16_t threshold_current_defauct=650;

/*-------------------------------------------- Config list status feedback Frame 1 
    ID: 0x111A
             ID Package 0x11 (status feedback Frame1)
             ID Device  0x1A (LID)
    byte[0]  uint8_t status_data
                  uint8_t opened = 0
                  uint8_t running = 1
                  uint8_t closed = 2
                  uint8_t stuck = 101
                  uint8_t Initializing=3
    byte[1]  uint16_t current_threshold_upper
    byte[2]  uint16_t current_threshold_lower
    byte[3]  byte button_status
                  bit[2] Lock Switch
                  bit[3] Lid Close Button
    byte[4]  uint16_t current_upper
    byte[5]  uint16_t current_lower
    byte[6]  uint16_t voltage_upper
    byte[7]  uint16_t voltage_lower
--------------------------------------------*/



/*--------------------------------------------Config list status feedback Frame 2
    ID: 0x121A
            ID Package 0x12 (status feedback Frame 2)
            ID Device  0x1A (LID)
    byte[0] byte error
            bit[0] error CURRENT
            bit[1] error EEPROM
            bit[2] error Lock Switch
            bit[3] error Lock Control
--------------------------------------------*/


/*=================== Frame 1 ===================*/
uint32_t Start_Feedback1 = 0x111A; 
uint8_t status_data;
uint8_t button_status=0;
/*===============================================*/


/*=================== Frame 2 ===================*/
uint32_t Start_Feedback2 = 0x121A;
uint8_t error=0;
/*===============================================*/

void run(LidState state)
{
  switch (state)
  {
  case CLOSED:
    duty = STOP_CMD;
    break;
  case CLOSING:
    //duty = close_speed * CLOSE_CMD;
    duty = soft_start(close_speed * CLOSE_CMD,duty);
    break;
  case OPENED:
    duty = STOP_CMD;
    break;
  case OPENING:
    //duty = open_speed * OPEN_CMD;
    duty = soft_start(open_speed * OPEN_CMD,duty);
    break;
  case STUCK_WHILE_CLOSING:
    duty = STOP_CMD;
    break;
  case STUCK_WHILE_OPENING:
    duty = STOP_CMD;
    break;
  default:
    break;
  }
}

LidState check_new_state()
{
  LidState new_lid_state = lid_state;
  limit_sw_open = digitalRead(LIMIT_SWICTH_OPEN);
  limit_sw_close = digitalRead(LIMIT_SWICTH_CLOSE);
  button_close = digitalRead(BUTTON_CLOSE);       // 1: Close lid    0: Open lid

  switch (lid_state)
  {
    case CLOSED:
      if (lid_cmd == OPEN_CMD) new_lid_state = OPENING;
      break;
    case CLOSING:
      if (limit_sw_close) new_lid_state = CLOSED;
      if (overload) new_lid_state = STUCK_WHILE_CLOSING; 
      break;
    case OPENED:
      if (lid_cmd == CLOSE_CMD || button_close == 1) new_lid_state = CLOSING;
      break;
    case OPENING:
      if (limit_sw_open) new_lid_state = OPENED;
      if (overload) new_lid_state = STUCK_WHILE_OPENING;
      break;
    case STUCK_WHILE_CLOSING:
      new_lid_state = OPENING;
      break;
    case STUCK_WHILE_OPENING:
      if (lid_cmd == OPEN_CMD) new_lid_state = OPENING;
      if (lid_cmd == CLOSE_CMD || button_close == 1) new_lid_state = CLOSING;   
      break;
    default:
      break;
  }
  return new_lid_state;
}

/*================= Set Bit to High level i position =================*/
void setBitIndex_High(uint8_t* data_Byte, int index)
{
  if(index >=0 && index <8)
  {
    *data_Byte |= (1 << index);
  }
}
/*====================================================================*/

/*================= Set Bit to Low level i position ==================*/
void setBitIndex_Low(uint8_t* data_Byte, int index)
{
  if(index >=0 && index <8)
  {
    *data_Byte &= ~(1 << index);
  }
}
/*====================================================================*/

/*===================== Get Bit level i position =====================*/
int getBitIndex(uint8_t get_error, int index)
{
  if(index >=0 && index <8)
  {
    return (get_error >> index) & 0x01;
  }
  return -1;
}
/*====================================================================*/

static inline int8_t sign(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

int soft_start(int u_set, float u)
{
  float d_u = u_set - u;

  int k = d_u / U_STEP;

  if (k == 0) return u_set;
  else
  {
    pre = sign(d_u) * U_STEP + u;
    return (int)(pre);
  }
}

int clip(int value, int min_, int max_)
{
  if ( value < min_ ) return min_;
  if ( value > max_ ) return max_;
  return value;
}

void set_motor(int pwm)
{
  if (pwm > 0)
  {
    digitalWrite(R_PWM, 1);
    digitalWrite(L_PWM, 0);
    ledcWrite(0, clip(pwm, 40, 255));
  }
  else if (pwm < 0)
  {
    digitalWrite(R_PWM, 0);
    digitalWrite(L_PWM, 1);
    ledcWrite(0, clip(abs(pwm), 40, 255));
  }
  else
  {
    digitalWrite(R_PWM, 0);
    digitalWrite(L_PWM, 0);
    ledcWrite(0, pwm);
  }
}
float calc_angle(long pos)
{
  return 2 * PI * pos / STEP;
}

/*======================= Check status limit switch, button ======================*/
void check_button()
{
  if(limit_sw_open == 1)
  {
    setBitIndex_High(&button_status,0);
    setBitIndex_Low(&button_status,1);
  }
  else if (limit_sw_close==1)
  {
    setBitIndex_High(&button_status,1);
    setBitIndex_Low(&button_status,0);
  }
  else if (button_close==1)
  {
    setBitIndex_High(&button_status,3);
  }
  else if (button_close==0)
  {
    setBitIndex_Low(&button_status,3);
  }
}
/*=================================================================================*/

/*========================== Lid status feedback Frame 1 ==========================*/
void send_status_frame1(uint32_t Start_Feedback1_tem, uint8_t status_data_tem, uint16_t threshold_current_tem, uint8_t button_status_tem, uint16_t current_tem, uint16_t voltage_tem)
{
  CAN.beginExtendedPacket(Start_Feedback1_tem);
  CAN.write(status_data_tem);
  byte thresholdByte[2];
  thresholdByte[0]=(byte)(threshold_current_tem >>8 ) & 0xFF;  //// threshold_upper
  thresholdByte[1]=(byte)(threshold_current_tem & 0xFF);       //// threshold_lower
  for(int i=0;i<sizeof(thresholdByte);i++)
  {
    CAN.write(thresholdByte[i]);
  }

  CAN.write(button_status_tem);

  byte currentByte[2];
  currentByte[0]=(byte)(current_tem >>8 ) & 0xFF;      //// current_upper
  currentByte[1]=(byte)(current_tem & 0xFF);           //// current_lower
  for(int i=0;i<sizeof(currentByte);i++)
  {
    CAN.write(currentByte[i]);
  }

  byte voltageByte[2];
  voltageByte[0]=(byte)(voltage_tem >> 8 ) & 0xFF;     //// voltage_upper
  voltageByte[1]=(byte)(voltage_tem & 0xFF);           //// voltage_lower
  for(int i=0;i<sizeof(voltageByte);i++)
  {
    CAN.write(voltageByte[i]);
  }
  CAN.endPacket();
  delay(10);
}
/*=================================================================================*/

/*========================== Lid status feedback Frame 2 ==========================*/
void send_status_frame2(uint32_t Start_Feedback2_tem, uint8_t error_tem)
{
  CAN.beginExtendedPacket(Start_Feedback2);
  CAN.write(error_tem);
  CAN.endPacket();
  delay(10);
}
/*=================================================================================*/

void setup() 
{
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(LR_EN, OUTPUT);
  pinMode(LIMIT_SWICTH_OPEN, INPUT_PULLUP);
  pinMode(LIMIT_SWICTH_CLOSE, INPUT_PULLUP);
  pinMode(BUTTON_CLOSE, INPUT_PULLUP);
  pinMode(2,OUTPUT);

  ledcSetup(0, 500, 8);
  ledcAttachPin(LR_EN, 0);
  delay(10);
  digitalWrite(L_PWM, 0);
  digitalWrite(R_PWM, 0);

  time_ms = millis();
  Serial.begin(115200);
  static unsigned long startTime = millis();
  EEPROM.begin(512);
  while(EEPROM.begin(512) == false) 
  {
    setBitIndex_High(&error,1);
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;
    if(elapsedTime >= 2000)
    {
      break;
    }
  }

  /*-------------- Read config from EEPROM --------------*/
  if((EEPROM.read(0)^EEPROM.read(1)^EEPROM.read(2)^EEPROM.read(3))==(EEPROM.read(4)))
  {
    close_speed=EEPROM.read(0);
    open_speed=EEPROM.read(1);
    threshold_current=(EEPROM.read(2)<<8) | EEPROM.read(3);
  }
  if((EEPROM.read(0)^EEPROM.read(1)^EEPROM.read(2)^EEPROM.read(3))!=(EEPROM.read(4)))
  {
    EEPROM.write(0,duty_defauct);
    EEPROM.write(1,duty_defauct);
    EEPROM.write(2,(threshold_current_defauct >> 8) & 0xFF);
    EEPROM.write(3,(threshold_current_defauct & 0xFF));
    if (!EEPROM.commit()) 
    {
      setBitIndex_High(&error,1);
    }
    close_speed = EEPROM.read(0);
    open_speed = EEPROM.read(1);
    threshold_current = (EEPROM.read(2)<<8) | EEPROM.read(3);
  }
  /*-----------------------------------------------------*/

  /*------------------ INA219 setup ------------------*/
  Wire.setPins(SDA, SCL);
  static unsigned long startTime1 = millis();
  ina219.begin();
  while(ina219.begin() == false) {
    setBitIndex_High(&error,0);
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime1;
    if(elapsedTime >=3000)
    {
      break;
    }
  }
  //Linear calibration
  ina219.linearCalibrate(/*The measured current before calibration*/ina219Reading_mA, /*The current measured by other current testers*/extMeterReading_mA);
  /*--------------------------------------------------*/

  /*------------------ CAN setup ------------------*/
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  if (!CAN.begin (500E3)) {
    Serial.println ("Starting CAN failed!");
    while (1);
  }
  else {
    Serial.println ("CAN Initialized");
  }
  /*-----------------------------------------------*/

  /*------------------ RESET MOTOR ------------------*/

  if(EEPROM.begin(512) == true && ina219.begin() == true) lid_state = OPENING;
  else  lid_state = CLOSED;
  /*-------------------------------------------------*/
}

void loop() {
  /*------------------------------- Check CAN Receive-------------------------------*/
  int size_data = CAN.parsePacket();
  if(size_data)
  {
    if(CAN.available())
    {
      int package_ID=(CAN.packetId() >> 8) & 0xFF;
      int device_ID=CAN.packetId() & 0xFF;
      if(device_ID==0x1A)
      {
        if(package_ID == 0x01)
        {
          byte data_input[8]; 
          CAN.readBytes(data_input, size_data);
          if(data_input[0] == 0)  lid_cmd = CLOSE_CMD;
          else if(data_input[0] == 1) lid_cmd = OPEN_CMD;
        }
        else if(package_ID == 0x02)
        {
          byte data_input[8]; 
          CAN.readBytes(data_input, size_data);
          new_close_speed = data_input[0];
          new_open_speed = data_input[1];
          new_threshold=(data_input[2] << 8) | data_input[3];
          if((threshold_current != new_threshold) || (close_speed != new_close_speed) || (open_speed != new_open_speed))
          {
            threshold_current=new_threshold;
            close_speed=new_close_speed;
            open_speed=new_open_speed;

            setBitIndex_Low(&error,1);
            EEPROM.write(0,data_input[0]);       //data close speed
            EEPROM.write(1,data_input[1]);       //data open speed
            EEPROM.write(2,data_input[2]);       //data threshold_lower
            EEPROM.write(3,data_input[3]);       //data threshold_upper
            EEPROM.write(4,data_input[4]);       //data checksum
                      
            if (!EEPROM.commit()) setBitIndex_High(&error,1);

          }    
        }
      }
    }
    else lid_cmd = STOP_CMD; 
  }
  /*-------------------------------------------------------------------------*/

  /*--------------------- Check current ---------------------*/
  if (millis() - time_ms  > 10)
  {
    // float  xn = ina219.getCurrent_mA();
    if(ina219.getCurrent_mA()==0.0)
    {
      setBitIndex_High(&error,0);
      ina219.begin();
    }
    else
    {
      setBitIndex_Low(&error,0);
    }
    float  yn = filter.updateEstimate(ina219.getCurrent_mA());
    float  vol = ina219.getBusVoltage_V();
    current=(int)(yn);
    voltage=(int)(vol*10);
    if(yn>=threshold_current)
    {
      overload = 1;
      status_data=101;
    } 
    time_ms = millis();
  }
  /*---------------------------------------------------------*/

  lid_state = check_new_state();
  check_button();
  run(lid_state);
  set_motor(duty);

  /*------------------------- Send Lid status feedback -------------------------*/ 
  send_status_frame1(Start_Feedback1,status_data,threshold_current,button_status,current,voltage);   
  send_status_frame2(Start_Feedback2,error);
  delay(1);
  /*----------------------------------------------------------------------------*/ 

}
