#include <AccelStepper.h>
#include <MultiStepper.h>


#define limit1 24
#define limit2 22
#define limit3 23
#define relay 25

/* Cấu trúc mô tả trạng thái biến khớp của robot tại vị trí hiện tại*/
struct JointInfo {
  long J1;
  long J2;
  long J3;
};

boolean debug = false;
const int BUFFER_SIZE = 100;
char buf[BUFFER_SIZE];

// Số xung để động cơ quay từ vị trí home đến vị trí hiện tại
static int J1;
static int J2;
static int J3;

// robot geometry
static float e = 5;     // end effector triangle 
static float f = 16;    // base triangle  
static float re = 40;   // length of long arm 
static float rf = 22;   // length of short arm f
 
// trigonometric constants
static float sqrt3 = sqrt(3.0);
static float pi = 3.141592653;    // PI
static float sin120 = sqrt3/2.0;   
static float cos120 = -0.5;        
static float tan60 = sqrt3;
static float sin30 = 0.5;
static float tan30 = 1/sqrt3;

// servo alignment
static int s1offset = -19;
static int s2offset = -14;
static int s3offset = -22;

// envelope boundaries
static float min_x = -15;
static float max_x = 15;
static float min_y = -15;
static float max_y = 15;
static float min_z = -55;
static float max_z = -24;
static float max_t = 80;
static float min_t = -45;

static int precision = 100; //round to 2 decimal places

float xp = 0;
float yp = 0;
float zp =-45;
float t1 = 0;  // servo angle t for 'theta', 1 for servo 1
float t2 = 0;
float t3 = 0;

// backups of previous positions in case we try to make an illegal move
float last_x;
float last_y;
float last_z;
float last_t1;
float last_t2;
float last_t3;

boolean validPosition;
boolean servosEnabled = false;


// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 6, 7); // (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper2(1, 8, 9);
AccelStepper stepper3(1, 10, 11);

MultiStepper steppersControl;  // Create instance of MultiStepper

long gotoposition[3]; // An array to store the target positions for each stepper motor
long initial_homing=-1;
JointInfo PR[15];

void setup() {
  Serial.begin(9600);
  pinMode(limit1, INPUT_PULLUP);
  pinMode(limit2, INPUT_PULLUP);
  pinMode(limit3, INPUT_PULLUP);
  pinMode(relay, OUTPUT);
  Serial.println("Homing...");
  
  HomeMachineHigh();
  


  // Adding the 3 steppers to the steppersControl instance for multi stepper control
  steppersControl.addStepper(stepper1);
  steppersControl.addStepper(stepper2);
  steppersControl.addStepper(stepper3);  
  stepper1.setMaxSpeed(100); // Set maximum speed value for the stepper
  stepper2.setMaxSpeed(100);
  stepper3.setMaxSpeed(100);
  stepper1.setAcceleration(30.0);
  stepper2.setAcceleration(30.0);
  stepper3.setAcceleration(30.0);
  stepper1.setSpeed(30); 
  stepper2.setSpeed(30);
  stepper3.setSpeed(30);
  
}

void loop() {
  // xp = 0;
  // yp = 0;
  // zp = -55;

//   gotoposition[0] = 200;  // 800 steps - full rotation with quater-step resolution
//   gotoposition[1] = -200;
//   gotoposition[2] = -200;

//   steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
//   steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position
//   delay(2000);
//   gotoposition[0] = 0;  // 800 steps - full rotation with quater-step resolution
//   gotoposition[1] = 0;
//   gotoposition[2] = 0;

//   steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
//   steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position
//   delay(2000);
  
  // -------------Nếu có dữ liệu trên serial buffer nhận--------------
  if (Serial.available() > 0) {
    String data = "";
    String code = "";
    int len = Serial.readBytesUntil('\r', buf, BUFFER_SIZE);
    // Chuyển đổi mảng byte thành chuỗi string
    for (int i = 0; i < len ; i++) {
      // Đọc mã code điều khiển
      if (i < 2){
        code += (char)buf[i];
      }
      // Đọc dữ liệu
      else{
        data += (char)buf[i];
      }
    }
    int value = data.toInt();
    Serial.print("Code:");
    Serial.print(code);

    // 1. Nếu đọc mã code teaching điểm: Lưu lại vị trí hiện tại của các động cơ so với vị trí gốc 0
    if (code == "TE"){
      int step1 = stepper1.currentPosition();
      int step2 = stepper2.currentPosition();
      int step3 = stepper3.currentPosition();
      PR[value].J1 = step1;
      PR[value].J2 = step2;
      PR[value].J3 = step3;
    }
    // 2. Nếu đọc mã code di chuyển điểm: Lấy vị trí step động cơ tại các vị trí cần đến - vị trí step hiện tại -> move()
    if (code == "MO"){

      PR[1].J1 = 70;
      PR[1].J2 = -70;
      PR[1].J3 = -70;

      PR[2].J1 = 230;
      PR[2].J2 = -230;
      PR[2].J3 = -380;

      PR[3].J1 = 390;
      PR[3].J2 = -240;
      PR[3].J3 = -240;

      PR[4].J1 = 370;
      PR[4].J2 = -370;
      PR[4].J3 = -170;

      int step1 = PR[value].J1;
      int step2 = PR[value].J2;
      int step3 = PR[value].J3;
      gotoposition[0] = step1;  // 800 steps - full rotation with quater-step resolution
      gotoposition[1] = step2;
      gotoposition[2] = step3;
      steppersControl.moveTo(gotoposition);
      steppersControl.runSpeedToPosition();
           
    }
    // 3. Nếu đọc mã code jog khớp J1: Di chuyển động cơ theo số bước từ vị trí hiện tại của nó
    if (code == "J1"){     
      gotoposition[0] = stepper1.currentPosition() + value;  // 800 steps - full rotation with quater-step resolution
      gotoposition[1] = stepper2.currentPosition();
      gotoposition[2] = stepper3.currentPosition();
      steppersControl.moveTo(gotoposition);
      steppersControl.runSpeedToPosition();
    }
    // 4. Nếu đọc mã code jog khớp J2 : Di chuyển động cơ theo số bước từ vị trí hiện tại của nó
    if (code == "J2"){
      gotoposition[0] = stepper1.currentPosition();  // 800 steps - full rotation with quater-step resolution
      gotoposition[1] = stepper2.currentPosition() + value;
      gotoposition[2] = stepper3.currentPosition();
      steppersControl.moveTo(gotoposition);
      steppersControl.runSpeedToPosition();
    }
    // 5. Nếu đọc mã code jog khớp J3 : Di chuyển động cơ theo số bước từ vị trí hiện tại của nó
    if (code == "J3"){
      gotoposition[0] = stepper1.currentPosition();  // 800 steps - full rotation with quater-step resolution
      gotoposition[1] = stepper2.currentPosition();
      gotoposition[2] = stepper3.currentPosition() + value;
      steppersControl.moveTo(gotoposition);
      steppersControl.runSpeedToPosition();
    }
    // 6. Nếu đọc mã code bật van hút
    if (code == "ON"){
      digitalWrite(relay, HIGH);
    }
    // 7. Nếu đọc mã code tắt van hút
    if (code == "OF"){
      digitalWrite(relay,LOW);
      
    }
  }
}

/* */
void StepperControl() {  
  gotoposition[0] = t1;  // 800 steps - full rotation with quater-step resolution
  gotoposition[1] = t2;
  gotoposition[2] = t3;

  steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
  steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position
}

/* Động học thuận robot delta */
float delta_calcAngleYZ(float x0, float y0, float z0) {
  float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
  y0 -= 0.5 * 0.57735    * e;    // shift center to edge
  // z = a + b*y
  float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
  float b = (y1-y0)/z0;
  // discriminant
  float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
  if (d < 0) return 999; // non-existing point
  float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
  float zj = a + b*yj;
  return 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
} 

/* Động học ngược robot delta */
void setThetasfromXYZ() {
  if ( debug ) { Serial.println("Entering: setThetasfromXYZ()"); }
  //first bounds-check the input
  if (xp < min_x) { xp = min_x; }
  if (xp > max_x) { xp = max_x; }
  if (yp < min_y) { yp = min_y; }
  if (yp > max_y) { yp = max_y; }
  if (zp < min_z) { zp = min_z; }
  if (zp > max_z) { zp = max_z; }
  
  validPosition = true;
  //set the first angle
  float theta1 = delta_calcAngleYZ(xp, yp, zp);
  if (theta1 != 999) {
    float theta2 = delta_calcAngleYZ(xp*cos120 + yp*sin120, yp*cos120-xp*sin120, zp);  // rotate coords to +120 deg
    if (theta2 != 999) {
      float theta3 = delta_calcAngleYZ(xp*cos120 - yp*sin120, yp*cos120+xp*sin120, zp);  // rotate coords to -120 deg
      if (theta3 != 999) {
        //we succeeded - point exists
        if (theta1 <= max_t && theta2 <= max_t && theta3 <= max_t && theta1 >= min_t && theta2 >= min_t && theta3 >= min_t ) { //bounds check
          t1 = round(theta1*4/1.8*3);
          t2 = round(theta2*4/1.8*3);
          t3 = round(theta3*4/1.8*3);
          Serial.println(t1);
          Serial.println(t2);
          Serial.println(t3);
          StepperControl();
        } else {
          validPosition = false;
        }

      } else {
        validPosition = false;
      }
    } else {
      validPosition = false;
    }
  } else {
    validPosition = false;
  } 
  //uh oh, we failed, revert to our last known good positions
  if ( !validPosition ) {
    xp = last_x;
    yp = last_y;
    zp = last_z;
  }      
  if ( debug ) { Serial.println("Exiting: setThetasfromXYZ()"); }
}

/* Hàm di chuyển robot về vị trí gốc */
void Limit(){
    stepper1.setSpeed(-50); 
    stepper2.setSpeed(50);
    stepper3.setSpeed(50);

    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
    while(digitalRead(limit1) == HIGH || digitalRead(limit2) == HIGH || digitalRead(limit3) == HIGH)
  {

    if(digitalRead(limit1) == LOW)
    {
      stepper1.setSpeed(0);
      stepper1.stop();

    }
    if(digitalRead(limit2) == LOW)
    {
      stepper2.setSpeed(0);
      stepper2.stop();
    }
    if(digitalRead(limit3) == LOW)
    {
      stepper3.setSpeed(0);
      stepper3.stop();
      
    }    
  } 
  // 
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
}


void HomeMachine()
{
  while(digitalRead(limit1) == LOW || digitalRead(limit2) == LOW || digitalRead(limit3) == LOW)
  {
    Serial.println("CT1: ");
    Serial.println(digitalRead(limit1));
    Serial.println("CT2: ");
    Serial.println(digitalRead(limit2));
    Serial.println("CT3: ");
    Serial.println(digitalRead(limit3));
    delay(5000);
    if(digitalRead(limit1) == LOW)
    {
      stepper1.move(-10);
      stepper1.run();
    }
    if(digitalRead(limit2) == LOW)
    {
      stepper2.move(10);
      stepper2.run();
    }
    if(digitalRead(limit3) == LOW)
    {
      stepper3.move(10);
      stepper3.run();
    }    
  } 
  // 
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  J1 = 0;
  J2 = 0;
  J3 = 0;
}
void HomeMachineHigh()
{

  stepper1.setMaxSpeed(1000); 
  stepper2.setMaxSpeed(1000);
  stepper3.setMaxSpeed(1000);
  stepper1.setAcceleration(500.0);
  stepper2.setAcceleration(500.0);
  stepper3.setAcceleration(500.0);
  stepper1.setSpeed(-5000); 
  stepper2.setSpeed(5000);
  stepper3.setSpeed(5000);
  
  while(digitalRead(limit1) == HIGH || digitalRead(limit2) == HIGH || digitalRead(limit3) == HIGH)
  {
    Serial.println("CT1: ");
    Serial.println(digitalRead(limit1));
    Serial.println("CT2: ");
    Serial.println(digitalRead(limit2));
    Serial.println("CT3: ");
    Serial.println(digitalRead(limit3));
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
    if(digitalRead(limit1) == LOW)
    {
      stepper1.setSpeed(0);
      stepper1.stop();

    }
    if(digitalRead(limit2) == LOW)
    {
      stepper2.setSpeed(0);
      stepper2.stop();
    }
    if(digitalRead(limit3) == LOW)
    {
      stepper3.setSpeed(0);
      stepper3.stop();
      
    }    
  } 
  // 
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  J1 = 0;
  J2 = 0;
  J3 = 0;
}
