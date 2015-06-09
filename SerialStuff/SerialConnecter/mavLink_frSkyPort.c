/*
APM2.5 Mavlink to FrSky X8R SPort interface using Teensy 3.1  http://www.pjrc.com/teensy/index.html
based on ideas found here http://code.google.com/p/telemetry-convert/
******************************************************
Cut board on the backside to separate Vin from VUSB

Connection on Teensy 3.1:
SPort S --> TX1//The signal will go to BBB


SPort + --> Vin
SPort  - --> GND

APM Telemetry DF13-5  Pin 2 --> RX2
APM Telemetry DF13-5  Pin 3 --> TX2
APM Telemetry DF13-5  Pin 5 --> GND

Analog input  --> A0 (pin14) on Teensy 3.1 ( max 3.3 V )


This is the data we send to FrSky, you can change this to have your own
set of data
******************************************************
Data transmitted to FrSky Taranis:
Cell           ( Voltage of Cell=Cells/4. [V] This is my LiPo pack 4S ) 
Cells         ( Voltage from LiPo [V] )
A2             ( Analog voltage from input A0 on Teensy 3.1 )
Alt             ( Altitude from baro.  [m] )
GAlt          ( Altitude from GPS   [m])
HDG         ( Compass heading  [deg])
Rpm         ( Throttle when ARMED [%] )
AccX         ( AccX m/s ? )
AccY         ( AccY m/s ? )
AccZ         ( AccZ m/s ? )
VSpd        ( Vertical speed [m/s] )
Speed      ( Ground speed from GPS,  [km/h] )
T1            ( GPS status = ap_sat_visible*10) + ap_fixtype )
T2            ( ARMED=1, DISARMED=0 )
Vfas          ( same as Cells )
Longitud    
Latitud
Dist          ( Will be calculated by FrSky Taranis as the distance from first received lat/long = Home Position

*******************************************************/

#include <GCS_MAVLink.h>//The data from the AutoPilot
#include <SoftwareSerial.h>
//#define debugSerial           Serial
//#define _MavLinkSerial      Serial2
SoftwareSerial _MavLinkSerial (9,10);
#define START                   1
#define MSG_RATE            10              // Hertz

// ******************************************
// Message #0  HEARTHBEAT 
uint8_t    ap_type = 0;
uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
uint32_t  ap_custom_mode = 0;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;

// Message # 1  SYS_STATUS 
uint16_t  ap_voltage_battery = 0;    // 1000 = 1V
int16_t    ap_current_battery = 0;    //  10 = 1A

// Message #24  GPS_RAW_INT 
uint8_t    ap_fixtype = 3;                  //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    ap_sat_visible = 0;           // numbers of visible satelites
// FrSky Taranis uses the first recieved lat/long as homeposition. 
int32_t    ap_latitude = 0;              // 585522540;
int32_t    ap_longitude = 0;            // 162344467;
int32_t    ap_gps_altitude = 0;        // 1000 = 1m


// Message #74 VFR_HUD 
int32_t    ap_airspeed = 0;
uint32_t  ap_groundspeed = 0;
uint32_t  ap_heading = 0;
uint16_t  ap_throttle = 0;

// FrSky Taranis uses the first recieved value after 'PowerOn' or  'Telemetry Reset'  as zero altitude
int32_t    ap_bar_altitude = 0;    // 100 = 1m
int32_t    ap_climb_rate=0;        // 100= 1m/s

// Message #27 RAW IMU 
int32_t   ap_accX = 0;
int32_t   ap_accY = 0;
int32_t   ap_accZ = 0;

int32_t   ap_accX_old = 0;
int32_t   ap_accY_old = 0;
int32_t   ap_accZ_old = 0;

// ******************************************
// These are special for FrSky
int32_t   adc2 = 0;               // 100 = 1.0V
int32_t     vfas = 0;                // 100 = 1,0V
int32_t     gps_status = 0;     // (ap_sat_visible * 10) + ap_fixtype
                                             // ex. 83 = 8 sattelites visible, 3D lock 
uint8_t   ap_cell_count = 0;

// ******************************************
uint8_t     MavLink_Connected;
uint8_t     buf[MAVLINK_MAX_PACKET_LEN];

uint16_t  hb_count;
uint16_t Volt_AverageBuffer[10]; 
uint16_t Current_AverageBuffer[10]; 
  
//returns the average of Voltage for the 10 last values  
uint32_t Get_Volt_Average(uint16_t value)  {
      uint8_t i;
      uint32_t sum=0;
      
      for(i=9;i>0;i--)  {
          Volt_AverageBuffer[i]=Volt_AverageBuffer[i-1];
          sum+=Volt_AverageBuffer[i];
          }
      Volt_AverageBuffer[0]=value;    
      return (sum+=value)/10;
  }
  
  //returns the average of Current for the 10 last values  
uint32_t Get_Current_Average(uint16_t value)  {
      uint8_t i;
      uint32_t sum=0;
      
      for(i=9;i>0;i--)  {
          Current_AverageBuffer[i]=Current_AverageBuffer[i-1];
          sum+=Current_AverageBuffer[i];
          }
      Current_AverageBuffer[0]=value;    
      return (sum+=value)/10;
  }

unsigned long MavLink_Connected_timer;
unsigned long hb_timer;
unsigned long acc_timer;

int led = 13;

mavlink_message_t msg;

// ******************************************
void setup()  {  
  //_MavLinkSerial.begin(57600);
  //debugSerial.begin(57600);
  _MavLinkSerial.begin(57600);
  //gpsSerial.begin(576000);
  MavLink_Connected = 0;
  MavLink_Connected_timer=millis();
  hb_timer = millis();
  acc_timer=millis();
  hb_count = 0;  
  
  pinMode(led,OUTPUT);
  pinMode(12,OUTPUT);  
  pinMode(14,INPUT);
  analogReference(DEFAULT);   
}

// ******************************************
void loop()  
{
   
    uint16_t len;  
    if(millis()-hb_timer > 1500) 
    {
        hb_timer=millis();
        if( !MavLink_Connected ) // Start requesting data streams from MavLink
        {    
            digitalWrite(led,HIGH);
            mavlink_msg_request_data_stream_pack(0xFF,0xBE,&msg,1,1,MAV_DATA_STREAM_EXTENDED_STATUS, MSG_RATE, START);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            _MavLinkSerial.write(buf,len);
            delay(10);
            
            mavlink_msg_request_data_stream_pack(0xFF,0xBE,&msg,1,1,MAV_DATA_STREAM_EXTRA2, MSG_RATE, START);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            _MavLinkSerial.write(buf,len);
            delay(10);
           
            mavlink_msg_request_data_stream_pack(0xFF,0xBE,&msg,1,1,MAV_DATA_STREAM_RAW_SENSORS, MSG_RATE, START);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            _MavLinkSerial.write(buf,len);
            digitalWrite(led,LOW);
        }
    }
 
    if((millis() - MavLink_Connected_timer) > 1500)  // if no HEARTBEAT from APM  in 1.5s then we are not connected  
    {   
        MavLink_Connected=0;
        hb_count = 0;
    }   
    _MavLink_receive();                       // Check MavLink communication
    //BBB_Process();                          // Check FrSky S.Port communication  
    //serialEvent();
    //SendToBBB();

    adc2 =analogRead(0)/4;                   // Read ananog value from A0 (Pin 14). ( Will be A2 value on FrSky LCD)  
    if((millis() - acc_timer) > 1000)  // Reset timer for AccX, AccY and AccZ 
    {   
        ap_accX_old=ap_accX;
        ap_accY_old=ap_accY;
        ap_accZ_old=ap_accZ;
        acc_timer=millis();        
        //debugSerial.println(ap_base_mode);
    }  
}

void _MavLink_receive() 
{ 
    mavlink_message_t msg;
    mavlink_status_t status;
    //Serial.println("inside Maveline Receive");
    while(_MavLinkSerial.available()) 
    { 
        //Serial.println("into the loop");
        uint8_t c = _MavLinkSerial.read();
        //uint8_t c = Serial.read();
        //Serial.println("CCCCCCCCCCCCCCCCCCCCCCCCCC" + c);
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
        {
            switch(msg.msgid)
	    {
                case MAVLINK_MSG_ID_HEARTBEAT:  // 0
                    
                    ap_base_mode = (mavlink_msg_heartbeat_get_base_mode(&msg) & 0x80) > 7;
                    ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
                    MavLink_Connected_timer=millis(); 
                    
                    if(!MavLink_Connected); 
                    {
                        hb_count++;   
                        if((hb_count++) > 10)            // If  received > 10 heartbeats from MavLink then we are connected
                        {        
                            MavLink_Connected=1;
                            hb_count=0;
                            digitalWrite(led,HIGH);      // LED will be ON when connected to MavLink, else it will slowly blink
                        }
                     }
                     break;
                           
                 case MAVLINK_MSG_ID_SYS_STATUS :   // 1
                 
                     ap_voltage_battery = Get_Volt_Average(mavlink_msg_sys_status_get_voltage_battery(&msg));        // 1 = 1mV
                     ap_current_battery = Get_Current_Average(mavlink_msg_sys_status_get_current_battery(&msg));     // 1 = 10mA

                     if(ap_voltage_battery > 21000)
                     { 
                         ap_cell_count = 6;
                     }
                     else if (ap_voltage_battery > 16800 && ap_cell_count != 6) 
                     {
                         ap_cell_count = 5;
                     }
                     else if(ap_voltage_battery > 12600 && ap_cell_count != 5) 
                     {
                         ap_cell_count = 4;
                     }
                     else if(ap_voltage_battery > 8400 && ap_cell_count != 4) 
                     {
                         ap_cell_count = 3;
                     }
                     else if(ap_voltage_battery > 4200 && ap_cell_count != 3) 
                     {
                         ap_cell_count = 2;
                     }
                     else 
                     {
                         ap_cell_count = 0;
                     }
                     break;
                          
                 case MAVLINK_MSG_ID_GPS_RAW_INT:   // 24
            
                     //Serial.println("hola");
                     ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                         // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix``
                     ap_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);          // numbers of visible satelites
                     gps_status = (ap_sat_visible*10) + ap_fixtype; 
                     
                     if(ap_fixtype == 3)  
                     {
                         //Serial.print("inside  3333333333333333333333333333333333333333333333333333333333333");
                         ap_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
                         ap_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
                         ap_gps_altitude = mavlink_msg_gps_raw_int_get_alt(&msg);                       // 1m = 1000
                     }
                     break;
                                 
                 case MAVLINK_MSG_ID_RAW_IMU:   // 27
                     ap_accX = mavlink_msg_raw_imu_get_xacc(&msg) / 10;   
                     ap_accY = mavlink_msg_raw_imu_get_yacc(&msg) / 10;
                     ap_accZ = mavlink_msg_raw_imu_get_zacc(&msg) / 10;
                     break;
                                 
                 case MAVLINK_MSG_ID_VFR_HUD:   //  74
                     ap_airspeed = 0;
                     ap_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);                        // 100 = 1m/s
                     ap_heading = mavlink_msg_vfr_hud_get_heading(&msg);                                // 100 = 100 deg
                     ap_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);                              // 100 = 100%
                     ap_bar_altitude = mavlink_msg_vfr_hud_get_alt(&msg) * 100;                         // m
                     ap_climb_rate=mavlink_msg_vfr_hud_get_climb(&msg) * 100;                           // m/s
                     break; 
                 default:
                     break;
	     }
         }
     }
}   
/*boolean printGPS = false;

void SendToBBB()
{
  if(ap_fixtype << 2 )
    {
      Serial.println("0,0,0");
    }
   else
   {
    //Serial.print("Latitude is ");
    Serial.print(ap_latitude);
    Serial.print(",");
    
    //Serial.print("Longitude is ");
    Serial.print(ap_longitude);
    Serial.print(",");
    
    //Serial.print("Altitude is ");
    Serial.println(ap_gps_altitude);
    //printGPS = false;
  }
  delay(500);
}
*/
//void serialEvent()
//{
  //while (Serial.available())
  //{
    //char inchar = (char)Serial.read();
    //if(inchar == 'A')
  //  printGPS = true;
    
  //}
//}
/*#include "FrSkySPort.h"

#define _FrSkySPort_Serial            Serial1
#define _FrSkySPort_C1                UART0_C1
#define _FrSkySPort_C3                UART0_C3
#define _FrSkySPort_S2                UART0_S2
#define _FrSkySPort_BAUD           57600
#define   MAX_ID_COUNT              19

short crc;                         // used for crc calc of frsky-packet
uint8_t lastRx;
uint32_t FR_ID_count = 0;
uint8_t cell_count = 0;
uint8_t latlong_flag = 0;
uint32_t latlong = 0;
uint8_t first=0;
// ***********************************************************************
void FrSkySPort_Init(void)  {
      _FrSkySPort_Serial.begin(_FrSkySPort_BAUD);
      _FrSkySPort_C3 = 0x10;            // Tx invert
      _FrSkySPort_C1= 0xA0;            // Single wire mode
      _FrSkySPort_S2 = 0x10;           // Rx Invert
      
}

// ***********************************************************************
void FrSkySPort_Process(void) {
    uint8_t data = 0;
        uint32_t temp=0;
    uint8_t offset;
        while ( _FrSkySPort_Serial.available()) 
          {
      data =  _FrSkySPort_Serial.read();
          if (lastRx == START_STOP && ((data == SENSOR_ID1) || (data == SENSOR_ID2) || (data == SENSOR_ID3)  || (data == SENSOR_ID4))) 
            {
             
              switch(FR_ID_count) {
                 case 0:
                   if(ap_fixtype==3) {
                     FrSkySPort_SendPackage(FR_ID_SPEED,ap_groundspeed *20 );  // from GPS converted to km/h
                    }
                   break;
                 case 1:
                   FrSkySPort_SendPackage(FR_ID_RPM,ap_throttle * 2);   //  * 2 if number of blades on Taranis is set to 2
                   break;
                case 2:
                   FrSkySPort_SendPackage(FR_ID_CURRENT,ap_current_battery / 10); 
                   break; 
               case 3:        // Sends the altitude value from barometer, first sent value used as zero altitude
                  FrSkySPort_SendPackage(FR_ID_ALTITUDE,ap_bar_altitude);   // from barometer, 100 = 1m
                  break;       
                case 4:        // Sends the ap_longitude value, setting bit 31 high
                   if(ap_fixtype==3) {
                       if(ap_longitude < 0)
                           latlong=((abs(ap_longitude)/100)*6)  | 0xC0000000;
                           else
                           latlong=((abs(ap_longitude)/100)*6)  | 0x80000000;
                       FrSkySPort_SendPackage(FR_ID_LATLONG,latlong);
                       }
                   break;
                 case 5:        // Sends the ap_latitude value, setting bit 31 low  
                     if(ap_fixtype==3) {
                         if(ap_latitude < 0 )
                             latlong=((abs(ap_latitude)/100)*6) | 0x40000000;
                             else
                             latlong=((abs(ap_latitude)/100)*6);
                         FrSkySPort_SendPackage(FR_ID_LATLONG,latlong);
                         }
                    break;  
                 case 6:        // Sends the compass heading
                   FrSkySPort_SendPackage(FR_ID_HEADING,ap_heading * 100);   // 10000 = 100 deg
                   break;    
                 case 7:        // Sends the analog value from input A0 on Teensy 3.1
                    FrSkySPort_SendPackage(FR_ID_ADC2,adc2);                  
                    break;       
                 case 8:        // First 2 cells
                       temp=((ap_voltage_battery/(ap_cell_count * 2)) & 0xFFF);
                       FrSkySPort_SendPackage(FR_ID_CELLS,(temp << 20) | (temp << 8));          // Battery cell 0,1
                       break;
                  case 9:    // Optional 3 and 4 Cells
                      if(ap_cell_count > 2) {
                          offset = ap_cell_count > 3 ? 0x02: 0x01;
                          temp=((ap_voltage_battery/(ap_cell_count * 2)) & 0xFFF);
                          FrSkySPort_SendPackage(FR_ID_CELLS,(temp << 20) | (temp << 8) | offset);  // Battery cell 2,3
                          }
                      break;
                 case 10:    // Optional 5 and 6 Cells
                      if(ap_cell_count > 4) {
                          offset = ap_cell_count > 5 ? 0x04: 0x03;
                          temp=((ap_voltage_battery/(ap_cell_count * 2)) & 0xFFF);
                          FrSkySPort_SendPackage(FR_ID_CELLS,(temp << 20) | (temp << 8) | offset);  // Battery cell 2,3
                          }
                      break;     
                 case 11:
                   FrSkySPort_SendPackage(FR_ID_ACCX,ap_accX_old - ap_accX);    
                     break;
                case 12:
                   FrSkySPort_SendPackage(FR_ID_ACCY,ap_accY_old - ap_accY); 
                   break; 
                case 13:
                   FrSkySPort_SendPackage(FR_ID_ACCZ,ap_accZ_old - ap_accZ ); 
                   break; 
                case 14:        // Sends voltage as a VFAS value
                   FrSkySPort_SendPackage(FR_ID_VFAS,ap_voltage_battery/10); 
                   break;   
                case 15:
                   FrSkySPort_SendPackage(FR_ID_T1,gps_status); 
                   break; 
                case 16:
                   FrSkySPort_SendPackage(FR_ID_T2,ap_base_mode); 
                   break;
               case 17:
                   FrSkySPort_SendPackage(FR_ID_VARIO,ap_climb_rate );       // 100 = 1m/s        
                   break;
               case 18:
                   //if(ap_fixtype==3) {
                       FrSkySPort_SendPackage(FR_ID_GPS_ALT,ap_gps_altitude / 10);   // from GPS,  100=1m
                     // }
                   break;
               case 19:
                   FrSkySPort_SendPackage(FR_ID_FUEL,ap_custom_mode); 
                   break;      
                   
               }
            FR_ID_count++;
            if(FR_ID_count > MAX_ID_COUNT) FR_ID_count = 0;  
            }
          lastRx=data;
          }
}


// ***********************************************************************
void FrSkySPort_SendByte(uint8_t byte) {
    
       _FrSkySPort_Serial.write(byte);
    
        // CRC update
    crc += byte;         //0-1FF
    crc += crc >> 8;   //0-100
    crc &= 0x00ff;
    crc += crc >> 8;   //0-0FF
    crc &= 0x00ff;
}


// ***********************************************************************
void FrSkySPort_SendCrc() {
    _FrSkySPort_Serial.write(0xFF-crc);
        crc = 0;          // CRC reset
}


// ***********************************************************************
void FrSkySPort_SendPackage(uint16_t id, uint32_t value) {
             
    if(MavLink_Connected) {
            digitalWrite(led,HIGH);
            }
        _FrSkySPort_C3 |= 32;      //  Transmit direction, to S.Port
    FrSkySPort_SendByte(DATA_FRAME);
    uint8_t *bytes = (uint8_t*)&id;
    FrSkySPort_SendByte(bytes[0]);
    FrSkySPort_SendByte(bytes[1]);
    bytes = (uint8_t*)&value;
    FrSkySPort_SendByte(bytes[0]);
    FrSkySPort_SendByte(bytes[1]);
    FrSkySPort_SendByte(bytes[2]);
    FrSkySPort_SendByte(bytes[3]);
    FrSkySPort_SendCrc();
    _FrSkySPort_Serial.flush();
    _FrSkySPort_C3 ^= 32;      // Transmit direction, from S.Port

        digitalWrite(led,LOW);
        
}*/

