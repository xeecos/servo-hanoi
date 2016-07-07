#include <MeOrion.h>

int value = 0;
String buffer = "";

double angle_rad = PI / 180.0;
double angle_deg = 180.0 / PI;

MePort port(3);
MePort tiltPort(4);

Servo servo1;//inside
Servo servo2;//outside
Servo servoTilt;
Servo servoClaw;

const double L1 = 80.0; //armLength,inside
const double L2 = 106.0; //armLength,outside

struct Angle
{
  double x;//inside
  double y; //outside
  double z;
  double c;
};
struct Queue{
    int array[5];
    int length;
};

void pushPoint(Angle p);
Angle pointToAngles(float x,float y,float z,float c);
void addPoint(float x, float y,int pen);
void step();

void parseBuffer();
float prevX;
float prevY;
float prevZ;
float prevC;
long lastTime = 0;
long runTime = 0;
int len = 0;
boolean isRunning = false;

Angle currentAngle;
Angle lastAngle;

Queue hanoi[3];
float degs = 180.0/PI;

int openClaw = 90;
int closeClaw[3] = {150,135,125};
int tilt[3] = {60,95,120};

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("order:/pos/x/y/z/c");
  servo1.attach(port.pin1());
  servo2.attach(port.pin2());
  servoTilt.attach(tiltPort.pin1());
  servoClaw.attach(tiltPort.pin2());
  hanoi[0].length = 0;
  hanoi[2].length = 0;
  hanoi[1].array[0] = 3;
  hanoi[1].array[1] = 2;
  hanoi[1].array[2] = 1;
  hanoi[1].length = 3;
  addPoints(0,150,150,100);
  delay(5000);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parseBuffer();
    } else {
      buffer += c;
    }
  }
  translate(1,2,0);
  delay(2000);
  translate(2,0,1);
  delay(2000);
  translate(0,1,2);
  delay(2000);
}

void parseBuffer() {
  buffer = buffer + "/";
  int count = 0;
  int startIndex = 0;
  int endIndex = 0;
  int len = buffer.length();
  if (len < 1) {
    return;
  }
  String tmp;
  String values[6];
  while (true) {
    startIndex = buffer.indexOf("/", endIndex);
    endIndex = buffer.indexOf("/", startIndex + 1);
    tmp = buffer.substring(startIndex + 1, endIndex);
    values[count] = tmp;
    count++;
    if (endIndex == len - 1) break;
  }
  if (values[0].equals("pos")) {
    addPoint(values[1].toFloat(), values[2].toFloat(),values[3].toFloat(),values[4].toFloat());
  }
  buffer = "";
}

void translate(int from,int to,int other){
  moveTo(from,to);
  moveTo(from,other);
  moveTo(to,other);
  moveTo(from,to);
  moveTo(other,from);
  moveTo(other,to);
  moveTo(from,to);
}
void moveTo(int fromPos,int toPos){
  float x1,y1,z1,x2,y2,z2,z3,c1,c2;
  y1 = 150;
  y2 = 150;
  x1 = (fromPos-1)*90;
  x2 = (toPos-1)*90;
  z1 = 160;
  z2 = tilt[hanoi[fromPos].length-1];
  z3 = tilt[hanoi[toPos].length];
  c1 = openClaw;
  c2 = closeClaw[hanoi[fromPos].array[hanoi[fromPos].length-1]-1];
  addPoints(x1,y1,z1,c1);
  addPoints(x1,y1,z2,c1);
  addPoints(x1,y1,z2,c2);
  addPoints(x1,y1,z1,c2);
  addPoints(x2,y2,z1,c2);
  addPoints(x2,y2,z3,c2);
  addPoints(x2,y2,z3,c1);
  addPoints(x2,y2,z1,c1);
  push(&hanoi[toPos],pop(&hanoi[fromPos]));
}
void addPoints(float x,float y,float z,float c){
  int len = 10;
  for(int i=0;i<len;i++){
    float tx = prevX+(x-prevX)/len*i;
    float ty = prevY+(y-prevY)/len*i;
    float tz = prevZ+(z-prevZ)/len*i;
    float tc = prevC+(c-prevC)/len*i;
    pushPoint(pointToAngles(tx,ty,tz,tc));
  }
  prevX = x;
  prevY = y;
  prevZ = z;
  prevC = c;
}
void addPoint(float x, float y,float tilt,float claw)
{
  servo1.write(x);
  servo2.write(y);
  servoTilt.write(tilt);
  servoClaw.write(claw);
  delay(50);
  //Serial.println("ok");
}
void pushPoint(Angle p){
  if(lastAngle.x!=p.x||lastAngle.y!=p.y||lastAngle.z!=p.z||lastAngle.c!=p.c){
      addPoint(p.x,p.y,p.z,p.c);
  } 
  lastAngle = p;
}
Angle pointToAngles(float x,float y,float z,float c){
  long pow1 = L1*L1;
  long pow2 = L2*L2;
  long powR = x*x+y*y;
  double R = sqrt(powR);
  Angle a;
  a.z = z;
  a.c = c;
  if(powR<pow1+pow2||R>L1+L2){
    a.x = 90;
    a.y = 90;
    return a;
  }
  int t = 1;
  a.x = floor(((atan2(x,y)-acos((powR+pow1-pow2)/(2*R*L1)))*degs+90.0)*t)/t;
  a.y = floor((acos((pow1+pow2-powR)/(2*L1*L2))-PI/2)*degs*t)/t;
  return a;
}
void push(Queue* queue, int value){
    if (queue->length >= 5)
        return;
    queue->array[queue->length] = value;
    queue->length++;
}
  
int pop(Queue* queue){
    if (queue->length <= 0)
        return 0;
    queue->length--;
    int v = queue->array[queue->length];
    queue->array[queue->length] = 0;
    return v;
}
