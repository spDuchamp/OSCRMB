#include <FS.h>
#include <ArduinoOSC.h>
#include <iq_module_communication.hpp>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

const bool DEBUG = true; // true to send OSC message to log host
const char* log_host = "SOMEIPADDR"; // (debugging using https://hexler.net/products/protokol)
const int log_port = 58888;

const char* configPortalPass = "SOMEPASSWORD";
const int recv_port = 54321;
const float kSpeed = 300.0f; // WARNING! Keep within safe operating speed

float vel_control = 0.0f;
float vel_offset = 0.0f;
float vel_offset_attenuation = 0.0f;
float vel_offset_atten_curve = 0.0f;
float vel_to_set = 0.0f;
float prev_vel_to_set = 0.0f;
float obs_vel = 0.0f;
float obs_motor_redline;
float min_vel_control = 0.06f;
float accX = 0.0f;
float accY = 0.0f;
float accZ = 0.0f;

IqSerial ser(Serial);
PropellerMotorControlClient prop(0);
BrushlessDriveClient mot(0);
AnticoggingClient cog(0);

void setup() {
  ser.begin(115200);
  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false); // output messes the motor comm
  if(!wifiManager.autoConnect("",configPortalPass)) {
    ESP.reset();
    delay(1000);
  }
  if (DEBUG) {
    OscWiFi.send(log_host,log_port,"/log/wifi_connected");
    OscWiFi.send(log_host,log_port,"/log/localIP",WiFi.localIP().toString());
  }

  OscWiFi.subscribe(recv_port,"/rmb/velocity",[](const OscMessage& m) {
    vel_control = m.arg<float>(0);
    vel_control = constrain(vel_control,-1,1);
    if (sq(vel_control)<sq(min_vel_control)) {
      vel_control=0.0;
    }
    if (DEBUG) OscWiFi.send(log_host,log_port,"/log/velocity_control",vel_control);
  });

  OscWiFi.subscribe(recv_port,"/rmb/veloffset",[](const OscMessage& m) {
    vel_offset = m.arg<float>(0);
    vel_offset = constrain(vel_offset,-1,1);
    if (DEBUG) OscWiFi.send(log_host,log_port,"/log/velocity_offset",vel_offset);
  });

  OscWiFi.subscribe(recv_port,"/rmb/veloffsetattenuation",[](const OscMessage& m) {
    vel_offset_attenuation = m.arg<float>(0);
    vel_offset_attenuation = constrain(vel_offset_attenuation,0,1);
    if (DEBUG) OscWiFi.send(log_host,log_port,"/log/velocity_offset_attenuation",vel_offset_attenuation);

  });

  OscWiFi.subscribe(recv_port,"/rmb/veloffsetattencurve",[](const OscMessage& m) {
    vel_offset_atten_curve = m.arg<float>(0);
    vel_offset_atten_curve = constrain(vel_offset_atten_curve,0,1);
    if (DEBUG) OscWiFi.send(log_host,log_port,"/log/velocity_offset_atten_curve",vel_offset_atten_curve);
  });

  OscWiFi.subscribe(recv_port, "/accxyz", accX,accY,accZ);

  ser.set(cog.is_enabled_,(uint8_t)1);
  ser.set(mot.motor_redline_start_,kSpeed);
  ser.set(mot.motor_redline_end_,kSpeed+50.0f);
  if (DEBUG) OscWiFi.send(log_host,log_port,"/log/obs_motor_redline",obs_motor_redline);
}

void loop() {
  OscWiFi.parse();
  vel_to_set = constrain(kSpeed*(vel_control+pow(vel_offset,3.0f)*pow(vel_offset_attenuation,1+5.0f*vel_offset_atten_curve))+accX/10.0f+accY,-kSpeed,kSpeed);
  ser.set(prop.ctrl_velocity_,vel_to_set);
  OscWiFi.post();
}
