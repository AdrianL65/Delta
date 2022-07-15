/***
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp8266-input-data-html-form/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
***/

#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <SPIFFS.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
  #include <Hash.h>
  #include <FS.h>
#endif
#include <ESPAsyncWebServer.h>

AsyncWebServer server(8000);

// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "Phoenix Airport21";
const char* password = "IcePhoenix21-20";

const char* PARAM_FLOAT_1 = "inputFloat_1";
const char* PARAM_FLOAT = "inputFloat";

// HTML web page to handle 3 input fields (inputString, inputInt, inputFloat)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>

  <title>ESP Input Form</title>
  <meta charset="utf-8", name="viewport", content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="estilos.css">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script></head><body>
    <h1>Delta</h1>
  <form action="/get" target="hidden-form">
    X_Pos (current value %inputFloat_1%): <input type="number " name="inputFloat_1">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    Y_Pos (current value %inputFloat%): <input type="number " name="inputFloat">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form>
  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if(!file || file.isDirectory()){
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  file.close();
  Serial.println(fileContent);
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

// Replaces placeholder with stored values
String processor(const String& var){
  //Serial.println(var);
  if(var == "inputFloat_1"){
    return readFile(SPIFFS, "/inputFloat_1.txt");
  }

  else if(var == "inputFloat"){
    return readFile(SPIFFS, "/inputFloat.txt");
  }
  return String();
}

int motor_speed=10;//grados por segundo

//General settings motor
const float max_angleG=30;
const float min_angleG=80;
const float tolerance_P=5; //Tolerance limit positions
float motor_resolution=25000;
long int delay_micro=1000000*360/motor_resolution/motor_speed;
//Hardware PINOUT
int pulse_left=13;
int dir_left=12;
int pulse_right=5;
int dir_right=10;
const float pi=3.14150965;


float read_character();
float cInv(float x, float y,float l1, float l2,float l5 ,float l6,float end_efector,int sol);
class Motor{
  public:
  float max_angle;
  float min_angle;
  long int max_index;
  long int current_index_pos;
  float home_angle;
  float motor_speed;
  int pin_pulse;
  int pin_dir;
  Motor (float min_angle,float max_angle, float home_position,float motor_speed,int pin_pulse,int pin_dir){
    this->max_angle=max_angle;
    this->min_angle=min_angle;
    this->max_index=abs(max_angle-min_angle)*motor_resolution/360;
    this->current_index_pos=0;
    this->motor_speed=motor_speed;
    this->home_angle=home_position;
    this->pin_pulse=pin_pulse;
    this->pin_dir=pin_dir;
  }
  int sature_angles(float angle);
  long int get_delta_pulses(float angle);
  void change_direction(bool flag,int delta);
};
class Delta_robot{
  public:
  Motor* right_motor;
  Motor* left_motor;
  float x_pos;
  float y_pos;
  float l1=270;
  float l2=540;
  float l5=170;
  float l6=100;
  float end_efector=30;

  bool reacheble_pos;

  Delta_robot (){
    this->right_motor=new Motor(-min_angleG,max_angleG,max_angleG,motor_speed,pulse_right,dir_right); //MIN   MAX  HOME AND MOTOR SPEED PIN_PULSE PIN_DIR
    this->left_motor=new Motor((-max_angleG+180),180+min_angleG,180-max_angleG,motor_speed,pulse_left,dir_left);
  }
  float cInv(float x, float y,float l1, float l2,float l5 ,float l6,float end_efector,int sol);
  void move_same_velocity(float angle_right, float angle_left);  
  void move_same_time();
  
};
 Delta_robot delta1;
void setup() {
 

  Serial.begin(115200);
  // Initialize SPIFFS
  #ifdef ESP32
    if(!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  #else
    if(!SPIFFS.begin()){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  #endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET inputString value on <ESP_IP>/get?inputString=<inputMessage>
    if (request->hasParam(PARAM_FLOAT_1)) {
      inputMessage = request->getParam(PARAM_FLOAT_1)->value();
      writeFile(SPIFFS, "/inputFloat_1.txt", inputMessage.c_str());
    }
    
    // GET inputFloat value on <ESP_IP>/get?inputFloat=<inputMessage>
    else if (request->hasParam(PARAM_FLOAT)) {
      inputMessage = request->getParam(PARAM_FLOAT)->value();
      writeFile(SPIFFS, "/inputFloat.txt", inputMessage.c_str());
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });
  server.onNotFound(notFound);
  server.begin();

    // put your setup code here, to run once:
  pinMode(pulse_left,OUTPUT);
  pinMode(pulse_right,OUTPUT);
  pinMode(dir_left,OUTPUT);
  pinMode(dir_right,OUTPUT);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Testing motor configuration");
  Serial.println(delta1.left_motor->max_angle);
}

void loop() {
  // To access your stored values on inputString, inputInt, inputFloat
  float yourInputFloat_1 = readFile(SPIFFS, "/inputFloat_1.txt").toFloat();
  Serial.print("* Your X_Pos: ");
  Serial.println(yourInputFloat_1);
    
  float yourInputFloat = readFile(SPIFFS, "/inputFloat.txt").toFloat();
  Serial.print("* Your Y_Pos: ");
  Serial.println(yourInputFloat);

 delay(5000);
  

  // put your main code here, to run repeatedly:
  //Serial.print("Ingrese las coordeanadas del motor derecho: ");
 
  float desired_xpos=yourInputFloat_1;
  //float angle_right=read_character();
  //Serial.print("Ingrese las coordeanadas del motor izquierdo: ");

  float desired_ypos=yourInputFloat;
  //float angle_left=read_character();//
  float angle_right=delta1.cInv( desired_xpos,  desired_ypos, delta1.l1,  delta1.l2, delta1.l5 , delta1.l6, delta1.end_efector,-1);
  float angle_left=delta1.cInv( desired_xpos,  desired_ypos, delta1.l1,  delta1.l2, delta1.l5 , delta1.l6, delta1.end_efector,1);
  Serial.println("PRUEBA..............................................................");
  if(delta1.reacheble_pos){
  Serial.println("Angle left");Serial.println(angle_left);
  Serial.println("Angle right");Serial.println(angle_right);
  delta1.move_same_velocity(angle_right,angle_left);
  }
  Serial.println("Position reached");
  Serial.println(" ");
 
}


void Delta_robot:: move_same_velocity(float angle_right, float angle_left){
   long int delta_right= this->right_motor->get_delta_pulses(angle_right);    
   long int delta_left=this->left_motor->get_delta_pulses(angle_left);
   Serial.print("Delta right: ");Serial.println(delta_right);
   Serial.print("Delta left: ");Serial.println(delta_left);
   int delta=abs(delta_right);
   if(abs(delta_right)>=abs(delta_left)){
      delta=abs(delta_right);
   }
   else{
      delta=abs(delta_left);
   }
   this->left_motor->change_direction(false,delta_left);
   this->right_motor->change_direction(true,delta_right);
   int cont_right=0;int cont_left=0;
   for (int angle_step=0;angle_step<delta;angle_step++){
    //Move left and right motors
    if(cont_right<abs(delta_right)){
        digitalWrite(this->right_motor->pin_pulse,HIGH);
        cont_right++; 
    }
    if(cont_left<abs(delta_left)) {
        digitalWrite(this->left_motor->pin_pulse,HIGH);
        cont_left++; 
    }
    delayMicroseconds(delay_micro/2);
    digitalWrite(this->right_motor->pin_pulse,LOW); 
    digitalWrite(this->left_motor->pin_pulse,LOW); 
    delayMicroseconds(delay_micro/2);
    
  }
    Serial.println(cont_left);
    Serial.println(cont_right);
    this->right_motor->current_index_pos=this->right_motor->current_index_pos+delta_right;
    this->left_motor->current_index_pos=this->left_motor->current_index_pos+delta_left; 
    Serial.print("Pos right: ");Serial.println(this->right_motor->current_index_pos);
   Serial.print("Pos left: ");Serial.println( this->left_motor->current_index_pos);
}
    
long int Motor:: get_delta_pulses(float angle){
  long int desired_index=this->current_index_pos;
  if(this->sature_angles(angle)){//Si el ángulo puede ser alcanzado
       
       desired_index=abs(angle-this->home_angle)/360*motor_resolution;
       Serial.print(desired_index); Serial.print("yeah");Serial.println(this->pin_dir);
       
  }
  long int delta_t=desired_index-(this->current_index_pos);
  return (delta_t);
}
int Motor:: sature_angles(float angle){
   Serial.println(angle);
  if ((this-> max_angle>angle)&(angle>(this->min_angle))){
    Serial.println("angle in range");    return 1;
  }
  else {
    return 0;
  }

}
void Motor::change_direction(bool flag,int delta){
    if(flag){//Motor derecho
        if(delta>0){
            digitalWrite(this->pin_dir,HIGH);
        }
        else{
            digitalWrite(this->pin_dir,LOW);
        }
    }
    else{//Motor izquierdo
        if(delta>0){
            digitalWrite(this->pin_dir,LOW);
        }
        else{
            digitalWrite(this->pin_dir,HIGH);
        }

    }
}
float read_character(){
  int var;  float data=0;
  int sign=1;
  while (Serial.available()){
    var=Serial.read();
    if(var==45){
      sign=-1;
      var=48;
    }
    if (var!=10){
      data=data*10+(var-48);
    }  
    else{
      Serial.println(data*sign);
      return (data*sign);
    } 
  }
}
float Delta_robot:: cInv(float x, float y,float l1, float l2,float l5 ,float l6,float end_efector,int sol){//Posición deseada en el plano [mm]
    float D,q2,phi,beta,q1;
    y=y+end_efector;
    if(sol==1){
      x=x-(l6/2)+(l5/2);
    }
    if(sol==-1){
      x=x+(l6/2)-(l5/2);
    }
    D= (pow(x,2)+pow(y,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2);
    if(pow(D,2)<=1){
          q2=atan2(sol*sqrt(1-(pow(D,2))),D);
          phi=atan2(y,x);
          beta=atan2(l2*sin(q2),l1+l2*cos(q2));
          q1=phi-beta;
          Serial.print("q1: "); Serial.println(q1);
          Serial.print("q2: "); Serial.println(q2);
          this->reacheble_pos=true;
          if(sol==1){
            if (q1<=0){
              q1=q1+2*pi;
            }
          }
          q1=q1/pi*180;
          return q1;
  
    }
    else{
        this->reacheble_pos=false;
        Serial.println("Point selected out of workspace");
    }
    
}