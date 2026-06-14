// OSR-Alpha4_ESP32 (V2) - No Temperature Control Version
// by TempestMAx 5-3-22, Modified: No NTC protection
// 无温控输出版本，适用于没有NTC温控PCB的老用户

// ----------------------------
//   Settings
// ----------------------------

// Device IDs, for external reference
#define FIRMWARE_ID "SR6-OSR2-CraftyHandy-NoNTC v2.0"
#define TCODE_VER "TCode v0.3"

#define OSR2_MODE false // (true/false) Switch servo outputs to OSR2 mode

// Servo microseconds per radian
// (Standard: 637 μs/rad)
// (LW-20: 700 μs/rad)
#define ms_per_rad 637  // (μs/rad)

// Pin assignments
// T-wist feedback goes on digital pin 2
#define LowerLeftServo_PIN 15    // Lower Left Servo (OSR2 Left Servo)
#define UpperLeftServo_PIN 2     // Upper Left Servo
#define LowerRightServo_PIN 13   // Lower Right Servo (OSR2 Right Servo)
#define UpperRightServo_PIN 12   // Upper Right Servo
#define LeftPitchServo_PIN 4     // Left Pitch Servo (OSR2 Pitch Servo)
#define RightPitchServo_PIN 14   // Right Pitch Servo
#define TwistServo_PIN 27        // Twist Servo
#define ValveServo_PIN 25        // Valve Servo
#define TwistFeedback_PIN 26     // Twist Servo Feedback
#define Vibe0_PIN 18             // Vibration motor 1
#define Vibe1_PIN 19             // Vibration motor 2

// Arm servo zeros - EEPROM-backed (can be modified via serial)
// Change these to adjust arm positions
// (1500 = centre, 1 complete step = 160)
#define DEFAULT_LowerLeft_ZERO 1500
#define DEFAULT_UpperLeft_ZERO 1500
#define DEFAULT_LowerRight_ZERO 1500
#define DEFAULT_UpperRight_ZERO 1500
#define DEFAULT_LeftPitch_ZERO 1500
#define DEFAULT_RightPitch_ZERO 1500
#define DEFAULT_TwistServo_ZERO 1500
#define DEFAULT_ValveServo_ZERO 1500

int LowerLeftServo_ZERO = DEFAULT_LowerLeft_ZERO;
int UpperLeftServo_ZERO = DEFAULT_UpperLeft_ZERO;
int LowerRightServo_ZERO = DEFAULT_LowerRight_ZERO;
int UpperRightServo_ZERO = DEFAULT_UpperRight_ZERO;
int LeftPitchServo_ZERO = DEFAULT_LeftPitch_ZERO;
int RightPitchServo_ZERO = DEFAULT_RightPitch_ZERO;
int TwistServo_ZERO = DEFAULT_TwistServo_ZERO;
int ValveServo_ZERO = DEFAULT_ValveServo_ZERO;

// Servo operating frequencies
#define PitchServo_Freq 330 // Pitch Servos
#define MainServo_Freq 330  // Main Servos
#define TwistServo_Freq 50  // Twist Servo
#define ValveServo_Freq 50  // Valve Servo
#define VibePWM_Freq 8000   // Vibe motor control PWM frequency

// Other functions
#define TWIST_PARALLAX false      // (true/false) Parallax 360 feedback servo on twist (t-wist3)
#define REVERSE_TWIST_SERVO false // (true/false) Reverse twist servo direction
#define VALVE_DEFAULT 5000        // Auto-valve default suction level (low-high, 0-9999)
#define REVERSE_VALVE_SERVO true  // (true/false) Reverse T-Valve direction
#define VIBE_TIMEOUT 2000         // Timeout for vibration channels (milliseconds).
#define LUBE_V1 false             // (true/false) Lube pump installed instead of vibration channel 1
#define Lube_PIN 23               // Lube manual input button pin (Connect pin to +5V for ON)
#define Lube_SPEED 255            // Lube pump speed (0-255)
#define MIN_SMOOTH_INTERVAL 3     // Minimum auto-smooth ramp interval for live commands (ms)
#define MAX_SMOOTH_INTERVAL 100   // Maximum auto-smooth ramp interval for live commands (ms)

// T-Code Channels
#define CHANNELS 10                // Number of channels of each type (LRVA)

// ----------------------------
//  Auto Settings
// ----------------------------
// Do not change

// Servo operating intervals
#define MainServo_Int 1000000/MainServo_Freq
#define PitchServo_Int 1000000/PitchServo_Freq
#define TwistServo_Int 1000000/TwistServo_Freq
#define ValveServo_Int 1000000/ValveServo_Freq

// Servo PWM channels
#define LowerLeftServo_PWM 0
#define UpperLeftServo_PWM 1
#define LowerRightServo_PWM 2
#define UpperRightServo_PWM 3
#define LeftPitchServo_PWM 4
#define RightPitchServo_PWM 5
#define TwistServo_PWM 6
#define ValveServo_PWM 7
#define TwistFeedback_PWM 8
#define Vibe0_PWM 9
#define Vibe1_PWM 10

// Libraries used
#include <EEPROM.h> // Permanent memory
#include "BluetoothSerial.h"
#include <math.h>
#include <WiFi.h>  // WiFi功能
#include <WebServer.h> // Web服务器
#include <WebSocketsServer.h> // WebSocket服务器

BluetoothSerial SerialBT;

// PWM通道号到GPIO引脚号的映射表（ESP32 Core 3.x ledcWrite 使用引脚号而非通道号）
const int channel_to_pin[11] = {
    LowerLeftServo_PIN,   // 0
    UpperLeftServo_PIN,   // 1
    LowerRightServo_PIN,  // 2
    UpperRightServo_PIN,  // 3
    LeftPitchServo_PIN,   // 4
    RightPitchServo_PIN,  // 5
    TwistServo_PIN,       // 6
    ValveServo_PIN,       // 7
    TwistFeedback_PIN,    // 8
    Vibe0_PIN,            // 9
    Vibe1_PIN             // 10
};

// -----------------------------
// Class to handle each axis
// -----------------------------
class Axis {

  public:
  // Setup function
  Axis() {

    // Set default dynamic parameters
    rampStartTime = 0;
    rampStart = 5000;
    rampStopTime = rampStart;
    rampStop = rampStart;

    // Set Empty Name
    Name = "";
    lastT = 0;

    // Live command auto-smooth
    minInterval = MAX_SMOOTH_INTERVAL;

  }

  // Function to set the axis dynamic parameters
  void Set(int x, char ext, long y) {
    unsigned long t = millis(); // This is the time now
    x = constrain(x,0,9999);
    y = constrain(y,0,9999999);
    // Set ramp parameters, based on inputs
    // Live command
    if ( y == 0 || ( ext != 'S' && ext != 'I' ) ) {
      // update auto-smooth regulator
      int lastInterval = t - rampStartTime;
      if ( lastInterval > minInterval && minInterval < MAX_SMOOTH_INTERVAL ) { minInterval += 1; }
      else if ( lastInterval < minInterval && minInterval > MIN_SMOOTH_INTERVAL ) { minInterval -= 1; }
      // Set ramp parameters
      rampStart = GetPosition();
      rampStopTime = t + minInterval;
    }
    // Speed command
    else if ( ext == 'S' ) {
      rampStart = GetPosition();  // Start from current position
      int d = x - rampStart;  // Distance to move
      if (d<0) { d = -d; }
      long dt = d;  // Time interval (time = dist/speed)
      dt *= 100;
      dt /= y;
      rampStopTime = t + dt;  // Time to arrive at new position
    }
    // Interval command
    else if ( ext == 'I' ) {
      rampStart = GetPosition();  // Start from current position
      rampStopTime = t + y;  // Time to arrive at new position
    }
    rampStartTime = t;
    rampStop = x;
    lastT = t;
  }

  // Function to return the current position of this axis
  int GetPosition() {
    int x; // This is the current axis position, 0-9999
    unsigned long t = millis();
    if (t > rampStopTime) {
      x = rampStop;
    } else if (t > rampStartTime) {
      x = map(t,rampStartTime,rampStopTime,rampStart,rampStop);
    } else {
      x = rampStart;
    }
    x = constrain(x,0,9999);
    return x;
  }

  // Function to stop axis movement at current position
  void Stop() {
    unsigned long t = millis(); // This is the time now
    rampStart = GetPosition();
    rampStartTime = t;
    rampStop = rampStart;
    rampStopTime = t;
  }

  // Public variables
  String Name;  // Function name of this axis
  unsigned long lastT;  //

  private:

  // Movement positions
  int rampStart;
  unsigned long rampStartTime;
  int rampStop;
  unsigned long rampStopTime;

  // Live command auto-smooth regulator
  int minInterval;

};

// 舵机速度系数和当前位置（需在TCode类前声明）
float servo_speed[6] = {1.0,1.0,1.0,1.0,1.0,1.0};
float servo_current[6] = {1500,1500,1500,1500,1500,1500};
char bt_device_name[32] = "craftyhandy"; // 蓝牙名称（需在TCode类前声明）
// WiFi配置
char wifi_ssid[48] = "";
char wifi_pass[48] = "";
bool wifi_connected = false;
WebServer wifiServer(80);
WebSocketsServer webSocket(81); // WebSocket端口81

// -----------------------------
// Class to manage Toy Comms
// -----------------------------
class TCode {

  public:
  // Setup function
  TCode(String firmware, String tcode) {
    firmwareID = firmware;
    tcodeID = tcode;

    // Vibe channels start at 0
    for (int i = 0; i < CHANNELS; i++) { Vibration[i].Set(0,' ',0); }

  }

  // Function to name and activate axis
  void RegisterAxis(String ID, String axisName) {
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': Linear[channel].Name = axisName; break;
        case 'R': Rotation[channel].Name = axisName; break;
        case 'V': Vibration[channel].Name = axisName; break;
        case 'A': Auxiliary[channel].Name = axisName; break;
      }
    }
  }

  // Function to read off individual bytes as input
  void ByteInput(byte inByte) {
    bufferString += (char)inByte;  // Add new character to string

    if (inByte=='\n') {  // Execute string on newline
      bufferString.trim();  // Remove spaces, etc, from buffer
      executeString(bufferString); // Execute string
      bufferString = ""; // Clear input string
    }
  }

  // Function to read off whole strings as input
  void StringInput(String inString) {
    bufferString = inString;  // Replace existing buffer with input string
    bufferString.trim();  // Remove spaces, etc, from buffer
    executeString(bufferString); // Execute string
    bufferString = ""; // Clear input string
  }

  // Function to set an axis
  void AxisInput(String ID, int magnitude, char extension, long extMagnitude) {
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': Linear[channel].Set(magnitude,extension,extMagnitude); break;
        case 'R': Rotation[channel].Set(magnitude,extension,extMagnitude); break;
        case 'V': Vibration[channel].Set(magnitude,extension,extMagnitude); break;
        case 'A': Auxiliary[channel].Set(magnitude,extension,extMagnitude); break;
      }
    }
  }

  // Function to read the current position of an axis
  int AxisRead(String ID) {
    int x = 5000; // This is the return variable
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': x = Linear[channel].GetPosition(); break;
        case 'R': x = Rotation[channel].GetPosition(); break;
        case 'V': x = Vibration[channel].GetPosition(); break;
        case 'A': x = Auxiliary[channel].GetPosition(); break;
      }
    }
    return x;
  }

  // Function to query when an axis was last commanded
  unsigned long AxisLast(String ID) {
    unsigned long t = 0; // Return time
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': t = Linear[channel].lastT; break;
        case 'R': t = Rotation[channel].lastT; break;
        case 'V': t = Vibration[channel].lastT; break;
        case 'A': t = Auxiliary[channel].lastT; break;
      }
    }
    return t;
  }

  private:
  // Strings
  String firmwareID;
  String tcodeID;
  String bufferString; // String to hold incomming commands

  // Declare axes
  Axis Linear[CHANNELS];
  Axis Rotation[CHANNELS];
  Axis Vibration[CHANNELS];
  Axis Auxiliary[CHANNELS];

  // Function to divide up and execute input string
  void executeString(String bufferString) {
    int index = bufferString.indexOf(' ');  // Look for spaces in string
    while (index > 0) {
      readCmd(bufferString.substring(0,index));  // Read off first command
      bufferString = bufferString.substring(index+1);  // Remove first command from string
      bufferString.trim();
      index = bufferString.indexOf(' ');  // Look for next space
    }
    readCmd(bufferString);  // Read off last command
  }

  // Function to process the individual commands
  void readCmd(String command) {
    command.toUpperCase();

    // Switch between command types
    switch( command.charAt(0) ) {
      // Axis commands
      case 'L':
      case 'R':
      case 'V':
      case 'A':
        axisCmd(command);
      break;

      // Device commands
      case 'D':
        deviceCmd(command);
      break;

      // Setup commands
      case '$':
        setupCmd(command);
      break;
    }
  }


  // Function to read and interpret axis commands
  void axisCmd(String command) {

    char type = command.charAt(0);  // Type of command - LRVA
    boolean valid = true;  // Command validity flag, valid by default

    // Check for channel number
    int channel = command.charAt(1) - '0';
    if (channel < 0 || channel >= CHANNELS) {valid = false;}
    channel = constrain(channel,0,CHANNELS);

    // Check for an extension
    char extension = ' ';
    int index = command.indexOf('S',2);
    if (index > 0) {
      extension = 'S';
    } else {
      index = command.indexOf('I',2);
      if (index > 0) {
        extension = 'I';
      }
    }
    if (index < 0) { index = command.length(); }

    // Get command magnitude
    String magString = command.substring(2,index);
    magString = magString.substring(0,4);
    while (magString.length() < 4) { magString += '0'; }
    int magnitude = magString.toInt();
    if (magnitude == 0 && magString.charAt(0) != '0') { valid = false; } // Invalidate if zero returned, but not a number

    // Get extension magnitude
    long extMagnitude = 0;
    if ( extension != ' ') {
      magString = command.substring(index+1);
      magString = magString.substring(0,8);
      extMagnitude = magString.toInt();
    }
    if (extMagnitude == 0) { extension = ' '; }

    // Switch between command types
    if (valid) {
      switch(type) {
        // Axis commands
        case 'L': Linear[channel].Set(magnitude,extension,extMagnitude); break;
        case 'R': Rotation[channel].Set(magnitude,extension,extMagnitude); break;
        case 'V': Vibration[channel].Set(magnitude,extension,extMagnitude); break;
        case 'A': Auxiliary[channel].Set(magnitude,extension,extMagnitude); break;
      }
    }

  }

  // Function to identify and execute device commands
  void deviceCmd(String command) {
    int i;
    // Remove "D"
    command = command.substring(1);

    // Look for device stop command
    if (command.substring(0,4) == "STOP") {
        for (i = 0; i < 10; i++) { Linear[i].Stop(); }
        for (i = 0; i < 10; i++) { Rotation[i].Stop(); }
        for (i = 0; i < 10; i++) { Vibration[i].Set(0,' ',0); }
        for (i = 0; i < 10; i++) { Auxiliary[i].Stop(); }
    } else {
      // Look for numbered device commands
      int commandNumber = command.toInt();
      if (commandNumber==0 && command.charAt(0)!='0' ) { command = -1; }
      switch( commandNumber ) {
        case 0:
          Serial.println(firmwareID);
        break;

        case 1:
          Serial.println(tcodeID);
        break;

        case 2:
          for (i = 0; i < 10; i++) { axisRow("L" + String(i), 8*i, Linear[i].Name); }
          for (i = 0; i < 10; i++) { axisRow("R" + String(i), 8*i+80, Rotation[i].Name); }
          for (i = 0; i < 10; i++) { axisRow("V" + String(i), 8*i+160, Vibration[i].Name); }
          for (i = 0; i < 10; i++) { axisRow("A" + String(i), 8*i+240, Auxiliary[i].Name); }
        break;

        case 3:
          // Read ZERO values with GPIO pin numbers
          Serial.print("GPIO15:"); Serial.print(LowerLeftServo_ZERO);
          Serial.print(" GPIO2:"); Serial.print(UpperLeftServo_ZERO);
          Serial.print(" GPIO13:"); Serial.print(LowerRightServo_ZERO);
          Serial.print(" GPIO12:"); Serial.print(UpperRightServo_ZERO);
          Serial.print(" GPIO4:"); Serial.print(LeftPitchServo_ZERO);
          Serial.print(" GPIO14:"); Serial.println(RightPitchServo_ZERO);
        break;

        case 4:
          // 无温控版本：返回未启用信息
          Serial.println("TEMP: N/A (无温控版本)");
        break;

        case 5:
          // Read servo speed settings
          Serial.print("SPD:");
          for (int i = 0; i < 6; i++) {
            Serial.print(servo_speed[i]);
            if (i < 5) Serial.print(",");
          }
          Serial.println();
        break;

        case 6:
          // Read Bluetooth name
          Serial.print("BT:");
          Serial.println(bt_device_name);
        break;

        case 7:
          // Read WiFi status
          if (wifi_connected) {
            Serial.print("WiFi:");
            Serial.print(WiFi.SSID());
            Serial.print(" IP:");
            Serial.println(WiFi.localIP().toString());
          } else {
            Serial.println("WiFi:OFF");
          }
        break;
      }
    }


  }

  // Function to modify axis preference values
  void setupCmd(String command) {
    // ---- ZERO command: $Z1500,1500,1500,1500,1500,1500 ----
    if (command.substring(0,2) == "$Z") {
      String data = command.substring(2);
      int v[8], idx = 0;
      while (data.length() > 0 && idx < 8) {
        int comma = data.indexOf(',');
        String token = (comma > 0) ? data.substring(0, comma) : data;
        data = (comma > 0) ? data.substring(comma + 1) : "";
        token.trim();
        v[idx] = token.toInt();
        if (v[idx] == 0 && token != "0") break;
        v[idx] = constrain(v[idx], 800, 2200);
        idx++;
      }
      if (idx >= 6) {
        int addr = 321;
        EEPROM.write(320, 0x5A);
        EEPROM.put(addr, v[0]); addr += 4;
        EEPROM.put(addr, v[1]); addr += 4;
        EEPROM.put(addr, v[2]); addr += 4;
        EEPROM.put(addr, v[3]); addr += 4;
        EEPROM.put(addr, v[4]); addr += 4;
        EEPROM.put(addr, v[5]); addr += 4;
        EEPROM.put(addr, (idx > 6 ? constrain(v[6],800,2200) : constrain(v[5],800,2200))); addr += 4;
        EEPROM.put(addr, (idx > 7 ? constrain(v[7],800,2200) : constrain(v[5],800,2200)));
        // Update runtime variables
        LowerLeftServo_ZERO = v[0];
        UpperLeftServo_ZERO = v[1];
        LowerRightServo_ZERO = v[2];
        UpperRightServo_ZERO = v[3];
        LeftPitchServo_ZERO = v[4];
        RightPitchServo_ZERO = v[5];
        if (idx > 6) TwistServo_ZERO = constrain(v[6],800,2200);
        if (idx > 7) ValveServo_ZERO = constrain(v[7],800,2200);
        EEPROM.commit();
        Serial.print("ZERO saved: ");
        for (int i = 0; i < 6; i++) { Serial.print(v[i]); if (i < 5) Serial.print(","); }
        Serial.println(" (已生效)");
      } else {
        Serial.println("ZERO format: $Z1500,1500,1500,1500,1500,1500");
      }
      return;
    }

    // ---- MOVE command: $M0-1500 (set PWM channel 0 to pulse 1500) ----
    if (command.substring(0,2) == "$M") {
      int dashIdx = command.indexOf('-');
      if (dashIdx == 3) {
        int ch = command.charAt(2) - '0';
        int val = command.substring(4).toInt();
        if (ch >= 0 && ch <= 10 && val > 0) {
          val = constrain(val, 500, 2500);
          int duty = map(val, 0, MainServo_Int, 0, 65535);
          if (ch >= 0 && ch <= 10) {
            ledcWrite(channel_to_pin[ch], duty);
          }
          Serial.print("MOVE ch"); Serial.print(ch); Serial.print("="); Serial.println(val);
        }
      }
      return;
    }

    // ---- SPEED command: $S1.0,1.0,1.0,1.0,1.0,1.0 ----
    if (command.substring(0,2) == "$S") {
      String data = command.substring(2);
      float v[6]; int idx = 0;
      while (data.length() > 0 && idx < 6) {
        int comma = data.indexOf(',');
        String token = (comma > 0) ? data.substring(0, comma) : data;
        data = (comma > 0) ? data.substring(comma + 1) : "";
        token.trim();
        v[idx] = token.toFloat();
        v[idx] = constrain(v[idx], 0.1, 5.0);
        idx++;
      }
      if (idx == 6) {
        for (int i = 0; i < 6; i++) servo_speed[i] = v[i];
        EEPROM.put(361, servo_speed);
        EEPROM.commit();
        Serial.print("SPEED saved: ");
        for (int i = 0; i < 6; i++) { Serial.print(servo_speed[i]); if (i < 5) Serial.print(","); }
        Serial.println();
      } else {
        Serial.println("SPEED format: $S1.0,1.0,1.0,1.0,1.0,1.0");
      }
      return;
    }

    // ---- BLUETOOTH name command: $Bcraftyhandy ----
    if (command.substring(0,2) == "$B") {
      String name = command.substring(2);
      name.trim();
      if (name.length() >= 1 && name.length() <= 30) {
        // Check for special characters
        bool valid = true;
        for (int i = 0; i < name.length(); i++) {
          char c = name.charAt(i);
          if (!isAlphaNumeric(c) && c != '-' && c != '_') { valid = false; break; }
        }
        if (valid) {
          name.toCharArray(bt_device_name, 32);
          // Save to EEPROM
          for (int i = 0; i < 32; i++) {
            EEPROM.write(386 + i, (i < name.length()) ? name.charAt(i) : 0);
          }
          EEPROM.write(385, 0x5B);
          EEPROM.commit();
          // Restart Bluetooth with new name
          SerialBT.end();
          SerialBT.begin(bt_device_name);
          Serial.print("BT name set to: ");
          Serial.println(bt_device_name);
        } else {
          Serial.println("BT name: 仅支持字母/数字/连字符/下划线");
        }
      } else {
        Serial.println("BT name: 长度1-30个字符");
      }
      return;
    }

    // ---- WiFi command: $Wssid,password ----
    if (command.substring(0,2) == "$W") {
      String data = command.substring(2);
      int comma = data.indexOf(',');
      if (comma > 0 && comma < 48) {
        String ssid = data.substring(0, comma);
        String pass = data.substring(comma + 1);
        ssid.trim(); pass.trim();
        if (ssid.length() > 0) {
          ssid.toCharArray(wifi_ssid, 48);
          pass.toCharArray(wifi_pass, 48);
          EEPROM.put(419, wifi_ssid);
          EEPROM.put(467, wifi_pass);
          EEPROM.write(418, 0x5C);
          EEPROM.commit();
          Serial.print("WiFi saved: ");
          Serial.println(ssid);
          Serial.println("Reboot to connect WiFi");
        }
      } else {
        Serial.println("WiFi format: $Wssid,password");
      }
      return;
    }

    int minVal,maxVal;
    String minValString,maxValString;
    boolean valid;
    // Axis type
    char type = command.charAt(1);
    switch (type) {
      case 'L':
      case 'R':
      case 'V':
      case 'A':
      valid = true;
      break;

      default:
      type = ' ';
      valid = false;
      break;
    }
    // Axis channel number
    int channel = (command.substring(2,3)).toInt();
    if (channel == 0 && command.charAt(2) != '0') {
      valid = false;
    }
    // Input numbers
    int index1 = command.indexOf('-');
    if (index1 !=3) { valid = false; }
    int index2 = command.indexOf('-',index1+1);  // Look for spaces in string
    if (index2 <=3) { valid = false; }
    if (valid) {
      // Min value
      minValString = command.substring(4,index2);
      minValString = minValString.substring(0,4);
      while (minValString.length() < 4) { minValString += '0'; }
      minVal = minValString.toInt();
      if ( minVal == 0 && minValString.charAt(0)!='0' ) { valid = false; }
      // Max value
      maxValString = command.substring(index2+1);
      maxValString = maxValString.substring(0,4);
      while (maxValString.length() < 4) { maxValString += '0'; }
      maxVal = maxValString.toInt();
      if ( maxVal == 0 && maxValString.charAt(0)!='0' ) { valid = false; }
    }
    // If a valid command, save axis preferences to EEPROM
    if (valid) {
      int memIndex = 0;
      switch (type) {
        case 'L': memIndex = 0; break;
        case 'R': memIndex = 80; break;
        case 'V': memIndex = 160; break;
        case 'A': memIndex = 240; break;
      }
      memIndex += 8*channel;
      minVal = constrain(minVal,0,9999);
      EEPROM.put(memIndex, minVal-1);
      minVal = constrain(maxVal,0,9999);
      EEPROM.put(memIndex+4, maxVal-10000);
      // Output that axis changed successfully
      switch (type) {
        case 'L': axisRow("L" + String(channel), memIndex, Linear[channel].Name); break;
        case 'R': axisRow("R" + String(channel), memIndex, Rotation[channel].Name); break;
        case 'V': axisRow("V" + String(channel), memIndex, Vibration[channel].Name); break;
        case 'A': axisRow("A" + String(channel), memIndex, Auxiliary[channel].Name); break;
      }
    }
  }

  // Function to print the details of an axis
  void axisRow(String axisID, int memIndex, String axisName) {
    int low, high;
    if (axisName != "") {
      EEPROM.get(memIndex,low);
      low = constrain(low,-1,9998);
      EEPROM.get(memIndex + 4,high);
      high = constrain(high,-10000,-1);
      Serial.print(axisID);
      Serial.print(" ");
      Serial.print(low + 1);
      Serial.print(" ");
      Serial.print(high + 10000);
      Serial.print(" ");
      Serial.println(axisName);
    }
  }

};

// 全局变量声明
TCode tcode(FIRMWARE_ID, TCODE_VER);

// 操作变量
int xLin,yLin,zLin;

const int servo_pwm_ch[6] = {0,1,2,3,4,5}; // PWM通道映射
int xRot,yRot,zRot;
int vibe0,vibe1;
int lube;
int valveCmd,suckCmd;
int xLast;
unsigned long tLast;
float upVel,valvePos;
volatile int twistPulseLength = 0;
volatile int twistPulseCycle = 1099;
volatile int twistPulseStart = 0;
float twistServoAngPos = 0.5;
int twistTurns = 0;
float twistPos;

// 舵机速度插值输出函数（ESP32 Core 3.x：用引脚号代替通道号）
void servoWriteSpeed(int ch, int target) {
  if (ch < 0 || ch > 5) { ledcWrite(channel_to_pin[ch], target); return; }
  float speed = servo_speed[ch];
  float cur = servo_current[ch];
  float diff = target - cur;
  float maxStep = 50.0 * speed; // 基础步长50，乘以速度系数
  if (diff > 0) {
    cur += min(diff, maxStep);
  } else {
    cur += max(diff, -maxStep);
  }
  if (abs(diff) < maxStep) cur = target;
  servo_current[ch] = cur;
  ledcWrite(channel_to_pin[ch], (int)cur);
}

void setup() {
    // Start serial connection and report status
    Serial.begin(115200);
    tcode.StringInput("D0");
    tcode.StringInput("D1");

    SerialBT.begin("craftyhandy"); //Bluetooth device name

    Serial.println("[固件] 无温控版本 - 适用于无NTC PCB的老用户");

    // Set SR6 arms to startup positions
    if (!OSR2_MODE) { tcode.StringInput("R2750"); }

    // #ESP32# Enable EEPROM
    EEPROM.begin(530);

    // Load ZERO values from EEPROM
    if (EEPROM.read(320) == 0x5A) {
        EEPROM.get(321, LowerLeftServo_ZERO);
        EEPROM.get(325, UpperLeftServo_ZERO);
        EEPROM.get(329, LowerRightServo_ZERO);
        EEPROM.get(333, UpperRightServo_ZERO);
        EEPROM.get(337, LeftPitchServo_ZERO);
        EEPROM.get(341, RightPitchServo_ZERO);
        EEPROM.get(345, TwistServo_ZERO);
        EEPROM.get(349, ValveServo_ZERO);
        EEPROM.get(361, servo_speed);
        // Validate speed values
        for (int i = 0; i < 6; i++) {
          if (isnan(servo_speed[i]) || servo_speed[i] < 0.1 || servo_speed[i] > 5.0) servo_speed[i] = 1.0;
        }
        // Load Bluetooth name
        char btName[32];
        EEPROM.get(386, btName);
          strncpy(bt_device_name, btName, 32);
        btName[31] = '\0';
        if (btName[0] != '\0' && btName[0] != 0xFF) {
          SerialBT.end();
          SerialBT.begin(btName);
        }
        // Load WiFi credentials
        EEPROM.get(419, wifi_ssid);
        EEPROM.get(467, wifi_pass);
        wifi_ssid[47] = '\0'; wifi_pass[47] = '\0';
    }

    // Connect WiFi if credentials exist
    if (wifi_ssid[0] != '\0' && wifi_ssid[0] != 0xFF) {
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifi_ssid, wifi_pass);
      Serial.print("[WiFi] Connecting to ");
      Serial.println(wifi_ssid);
      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500); attempts++;
        Serial.print(".");
      }
      if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        Serial.println("\n[WiFi] Connected! IP: " + WiFi.localIP().toString());
        // Start web server
        wifiServer.on("/", []() {
          String html = "<html><body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>";
          html += "<h1>SR6 ESP32 Control</h1>";
          html += "<p>IP: " + WiFi.localIP().toString() + "</p>";
          html += "<form action='/cmd' method='get'>";
          html += "TCode: <input type='text' name='c' style='width:300px;padding:8px'>";
          html += "<input type='submit' value='Send'></form>";
          html += "<p><a href='/status'>Status</a></p>";
          html += "</body></html>";
          wifiServer.send(200, "text/html", html);
        });
        wifiServer.on("/status", []() {
          String s = "WiFi Connected\nIP: " + WiFi.localIP().toString() + "\nFirmware: " FIRMWARE_ID;
          wifiServer.send(200, "text/plain", s);
        });
        wifiServer.on("/cmd", []() {
          if (wifiServer.hasArg("c")) {
            String cmd = wifiServer.arg("c");
            tcode.StringInput(cmd);
            wifiServer.send(200, "text/plain", "OK: " + cmd);
          } else {
            wifiServer.send(400, "text/plain", "Missing cmd");
          }
        });
        wifiServer.begin();
        Serial.println("[WiFi] Web server started");
        // Start WebSocket server on port 81
        webSocket.begin();
        webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
          if (type == WStype_TEXT) {
            String cmd = String((char*)payload);
            cmd.trim();
            if (cmd.length() > 0) {
              tcode.StringInput(cmd);
              webSocket.sendTXT(num, "OK: " + cmd);
            }
          }
        });
        Serial.println("[WiFi] WebSocket server started (port 81)");
      } else {
        Serial.println("\n[WiFi] Failed to connect");
        WiFi.mode(WIFI_OFF);
      }
    }

    // Register device axes
    tcode.RegisterAxis("L0", "Up");
    if (!OSR2_MODE) {
        tcode.RegisterAxis("L1", "Forward");
        tcode.RegisterAxis("L2", "Left");
    }
    tcode.RegisterAxis("R0", "Twist");
    tcode.RegisterAxis("R1", "Roll");
    tcode.RegisterAxis("R2", "Pitch");
    tcode.RegisterAxis("V0", "Vibe1");
    if (!LUBE_V1) { tcode.RegisterAxis("V1", "Vibe2"); }
    tcode.RegisterAxis("A0", "Valve");
    tcode.RegisterAxis("A1", "Suck");
    tcode.AxisInput("A1",VALVE_DEFAULT,'I',3000);
    if (LUBE_V1) {
        tcode.RegisterAxis("A2", "Lube");
        tcode.AxisInput("A2",0,' ',0);
        pinMode(Lube_PIN,INPUT);
    }

    // Setup Servo PWM channels (ESP32 Core 3.x API)
    ledcAttach(LowerLeftServo_PIN, MainServo_Freq, 16);
    ledcAttach(UpperLeftServo_PIN, MainServo_Freq, 16);
    ledcAttach(LowerRightServo_PIN, MainServo_Freq, 16);
    ledcAttach(UpperRightServo_PIN, MainServo_Freq, 16);
    ledcAttach(LeftPitchServo_PIN, PitchServo_Freq, 16);
    ledcAttach(RightPitchServo_PIN, PitchServo_Freq, 16);
    ledcAttach(TwistServo_PIN, TwistServo_Freq, 16);
    ledcAttach(ValveServo_PIN, ValveServo_Freq, 16);

    // Set vibration PWM pins
    ledcAttach(Vibe0_PIN, VibePWM_Freq, 8);
    ledcAttach(Vibe1_PIN, VibePWM_Freq, 8);

    // Initiate position tracking for twist
    if (TWIST_PARALLAX) {
        pinMode(TwistFeedback_PIN,INPUT);
        attachInterrupt(TwistFeedback_PIN, twistRising, RISING);
    }

    // Signal done and print ZERO values
    Serial.println("Ready! (无温控版本)");
    Serial.println(LowerLeftServo_ZERO);
    Serial.println(UpperLeftServo_ZERO);
    Serial.println(LowerRightServo_ZERO);
    Serial.println(UpperRightServo_ZERO);
    Serial.println(LeftPitchServo_ZERO);
    Serial.println(RightPitchServo_ZERO);
}

void loop() {
    // WiFi web server处理
    if (wifi_connected) {
    wifiServer.handleClient();
    webSocket.loop();
}

    while (Serial.available() > 0) {
        tcode.ByteInput(Serial.read());
    }

    while (SerialBT.available() > 0) {
        tcode.ByteInput(SerialBT.read());
    }

    // Collect inputs
    xLin = tcode.AxisRead("L0");
    if (!OSR2_MODE) {
        yLin = tcode.AxisRead("L1");
        zLin = tcode.AxisRead("L2");
    }
    xRot = tcode.AxisRead("R0");
    yRot = tcode.AxisRead("R1");
    zRot = tcode.AxisRead("R2");
    vibe0 = tcode.AxisRead("V0");
    if (!LUBE_V1) { vibe1 = tcode.AxisRead("V1"); }
    valveCmd = tcode.AxisRead("A0");
    suckCmd = tcode.AxisRead("A1");
    if (LUBE_V1) { lube = tcode.AxisRead("A2"); }

    // If t-wist3, calculate twist position
    if (TWIST_PARALLAX) {
        float dutyCycle = twistPulseLength;
        dutyCycle = dutyCycle/twistPulseCycle;
        float angPos = (dutyCycle - 0.029)/0.942;
        angPos = constrain(angPos,0,1) - 0.5;
        if (angPos - twistServoAngPos < - 0.8) { twistTurns += 1; }
        if (angPos - twistServoAngPos > 0.8) { twistTurns -= 1; }
        twistServoAngPos = angPos;
        twistPos = 1000*(angPos + twistTurns);
    }

    // Calculate valve position
    unsigned long t = millis();
    float upVelNow;
    if (t > tLast) {
        upVelNow = xLin - xLast;
        upVelNow /= t - tLast;
        upVel = (upVelNow + 9*upVel)/10;
    }
    tLast = t;
    xLast = xLin;

    boolean suck;
    if (tcode.AxisLast("A1") >= tcode.AxisLast("A0")) {
        suck = true;
        valveCmd = suckCmd;
    } else {
        suck = false;
    }

    if (suck) {
        if (upVel < -5) {
            valveCmd = 0;
        } else if ( upVel < 0 ) {
            valveCmd = map(100*upVel,0,-500,suckCmd,0);
        }
    }
    valvePos = (9*valvePos + map(valveCmd,0,9999,0,1000))/10;

    // OSR2 Kinematics
    if (OSR2_MODE) {
        int stroke,roll,pitch;
        stroke = map(xLin,0,9999,-350,350);
        roll   = map(yRot,0,9999,-180,180);
        pitch  = map(zRot,0,9999,-350,350);

        ledcWrite(LowerLeftServo_PIN, map(LowerLeftServo_ZERO + stroke + roll,0,MainServo_Int,0,65535));
        ledcWrite(LowerRightServo_PIN, map(LowerRightServo_ZERO - stroke + roll,0,MainServo_Int,0,65535));
        ledcWrite(LeftPitchServo_PIN, map(LeftPitchServo_ZERO - pitch,0,PitchServo_Int,0,65535));
        ledcWrite(UpperLeftServo_PIN, map(UpperLeftServo_ZERO,0,MainServo_Int,0,65535));
        ledcWrite(RightPitchServo_PIN, map(RightPitchServo_ZERO,0,PitchServo_Int,0,65535));
        ledcWrite(UpperRightServo_PIN, map(UpperRightServo_ZERO,0,MainServo_Int,0,65535));
    }

    // SR6 Kinematics
    else {
        int roll,pitch,fwd,thrust,side;
        int out1,out2,out3,out4,out5,out6;
        roll = map(yRot,0,9999,-3000,3000);
        pitch = map(zRot,0,9999,-2500,2500);
        fwd = map(yLin,0,9999,-3000,3000);
        thrust = map(xLin,0,9999,-6000,6000);
        side = map(zLin,0,9999,-3000,3000);

        out1 = SetMainServo(16248 - fwd, 1500 + thrust + roll);
        out2 = SetMainServo(16248 - fwd, 1500 - thrust - roll);
        out5 = SetMainServo(16248 - fwd, 1500 - thrust + roll);
        out6 = SetMainServo(16248 - fwd, 1500 + thrust - roll);
        out3 = SetPitchServo(16248 - fwd, 4500 - thrust,  side - 1.5*roll, -pitch);
        out4 = SetPitchServo(16248 - fwd, 4500 - thrust, -side + 1.5*roll, -pitch);

        // 带速度插值的舵机输出
        servoWriteSpeed(0, map(LowerLeftServo_ZERO - out1,0,MainServo_Int,0,65535));
        servoWriteSpeed(1, map(UpperLeftServo_ZERO + out2,0,MainServo_Int,0,65535));
        servoWriteSpeed(4, map(constrain(LeftPitchServo_ZERO - out3,LeftPitchServo_ZERO-600,LeftPitchServo_ZERO+1000),0,PitchServo_Int,0,65535));
        servoWriteSpeed(5, map(constrain(RightPitchServo_ZERO + out4,RightPitchServo_ZERO-1000,RightPitchServo_ZERO+600),0,PitchServo_Int,0,65535));
        servoWriteSpeed(3, map(UpperRightServo_ZERO - out5,0,MainServo_Int,0,65535));
        servoWriteSpeed(2, map(LowerRightServo_ZERO + out6,0,MainServo_Int,0,65535));
    }

    // Twist and valve
    int twist,valve;
    if (TWIST_PARALLAX) {
        twist  = (xRot - map(twistPos,-1500,1500,9999,0))/5;
        twist  = constrain(twist, -750, 750);
    } else {
        twist  = map(xRot,0,9999,1000,-1000);
        if (REVERSE_TWIST_SERVO) { twist = -twist; }
    }
    valve  = valvePos - 500;
    valve  = constrain(valve, -500, 500);
    if (REVERSE_VALVE_SERVO) { valve = -valve; }

    ledcWrite(TwistServo_PIN, map(TwistServo_ZERO + twist,0,TwistServo_Int,0,65535));
    ledcWrite(ValveServo_PIN, map(ValveServo_ZERO + valve,0,ValveServo_Int,0,65535));

    // Output vibration channels
    if (vibe0 > 0 && vibe0 <= 9999) {
        ledcWrite(Vibe0_PIN, map(vibe0,1,9999,31,255));
    } else {
        ledcWrite(Vibe0_PIN, 0);
    }
    if (!LUBE_V1 && vibe1 > 0 && vibe1 <= 9999) {
        ledcWrite(Vibe1_PIN, map(vibe1,1,9999,31,255));
    } else {
        ledcWrite(Vibe1_PIN, 0);
    }

    // Vibe timeout functions
    if (millis() - tcode.AxisLast("V0") > VIBE_TIMEOUT) { tcode.AxisInput("V0",0,'I',500); }
    if (!LUBE_V1 && millis() - tcode.AxisLast("V1") > VIBE_TIMEOUT) { tcode.AxisInput("V1",0,'I',500); }

    // Lube functions
    if (LUBE_V1) {
        if (lube > 0 && lube <= 9999) {
            ledcWrite(Vibe1_PIN, map(lube,1,9999,127,255));
        } else if (digitalRead(Lube_PIN) == HIGH) {
            ledcWrite(Vibe1_PIN,Lube_SPEED);
        } else {
            ledcWrite(Vibe1_PIN,0);
        }
        if (millis() - tcode.AxisLast("A2") > 500) { tcode.AxisInput("A2",0,' ',0); }
    }
}

// Function to calculate the angle for the main arm servos
int SetMainServo(float x, float y) {
    x /= 100; y /= 100;
    float gamma = atan2(x,y);
    float csq = sq(x) + sq(y);
    float c = sqrt(csq);
    float beta = acos((csq - 28125)/(100*c));
    int out = ms_per_rad*(gamma + beta - 3.14159);
    return out;
}

// Function to calculate the angle for the pitcher arm servos
int SetPitchServo(float x, float y, float z, float pitch) {
    pitch *= 0.0001745;
    x += 5500*sin(0.2618 + pitch);
    y -= 5500*cos(0.2618 + pitch);
    x /= 100; y /= 100; z /= 100;
    float bsq = 36250 - sq(75 + z);
    float gamma = atan2(x,y);
    float csq = sq(x) + sq(y);
    float c = sqrt(csq);
    float beta = acos((csq + 5625 - bsq)/(150*c));
    int out = ms_per_rad*(gamma + beta - 3.14159);
    return out;
}

// T-wist3 parallax position detection functions
void twistRising() {
    attachInterrupt(TwistFeedback_PIN, twistFalling, FALLING);
    twistPulseCycle = micros()-twistPulseStart;
    twistPulseStart = micros();
}

void twistFalling() {
    attachInterrupt(TwistFeedback_PIN, twistRising, RISING);
    twistPulseLength = micros()-twistPulseStart;
}
