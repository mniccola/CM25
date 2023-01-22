// =======================================================================================
//                     PS3 Starting Sketch for Notre Dame Droid Class
// =======================================================================================
//                          Last Revised Date: 04/18/2021
//                             Revised By: Corinne Niezgodzki
// =======================================================================================
// ---------------------------------------------------------------------------------------
//                          Libraries
// ---------------------------------------------------------------------------------------
#include <PS3BT.h>
#include <usbhub.h>
#include <Sabertooth.h>
#include <Adafruit_TLC5947.h>
#include <MP3Trigger.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NewPing.h>
#include <Adafruit_TLC5947.h>

// ---------------------------------------------------------------------------------------
//                       Integrated Routine Setup
// ---------------------------------------------------------------------------------------
// Routine 1 - Party at the Tavern (the obligatory party mode)
boolean partyMode = false;
long partyPulse = 0;
long donutPulse = 0;
long donutPulse2 = 0;
/*boolean danceMove1 = true;
boolean danceMove2 = false;
boolean danceMove3 = false;*/
boolean playPartySound1 = true;
boolean playPartySound2 = false;
boolean playApplause = false;
boolean donut1Sound = true;
boolean donut2Sound = true;
long intro_forward;
long intro_backwards;
long partyLightPulse;
boolean doPartyLights = true;
long partySound1Pulse;
long partySound2Pulse;
long applause;

boolean talking_lights = false;
boolean talk_on = true;
long short_pause = 250;
long long_pause = 450; 
long talk_interval = 0;

boolean doApplauseLights = false;
boolean clap_on = true;
long clap_pause = 100;
long clap_interval = 0;


// Routine 2 (Level UP)
boolean leveling_up = false;
boolean levelSound1 = true;
boolean levelSound2 = false;
boolean levelSound3 = false;
boolean levelSound4 = false;
long levelPulse;
long preStrongMagicPulse;
long strongMagicPulse;
long ambientMagicPulse;
long powerUpPulse;
long fanfarePulse;
long levelTalk = 0;

long ambientLightPulse;
long ambientLightInterval = 200;
int ambientLightLevel = 0;
int ambientLightChange = 100;

long randomAmbientPulse;
long randomAmbientInterval = 100;
int randomAmbient = 0;
int randomAmbientChange = 100;

boolean climaxOn = true;
long powerUpClimaxPulse;
long climaxLightPulse;
long climaxInterval = 250;
long wigglePulse;
int wiggleTurn = 50;

long fanfareLightPulse;
long fanfareInterval = 45;

// Routine 3 - Fight
boolean fight = false;
long fightPulse;
long papiPulse;
boolean playPapi = false;
boolean papiTalk = false;
long rollPulse;
boolean playRoll = false;
long fightMusicPulse;
boolean playFightMusic = false;
long pintPulse;
boolean playPint = false;
long dicePulse;
long diceWaitPulse;
boolean diceDropped = false;
boolean playWorm = true;
long wormPulse;
boolean wormTalk = false;
boolean wormOn = true;
long hitPulse = 0;
long ambientTavernPulse;
long ambientTavernInterval = 200;

// ---------------------------------------------------------------------------------------
//                       LED
// ---------------------------------------------------------------------------------------
#define clock 5
#define data 4
#define latch 6

long LEDpulse = millis();
boolean LEDOn = false;
boolean brawlFight = false;
long fireplacePulse = millis();

Adafruit_TLC5947 LEDControl = Adafruit_TLC5947(1, clock, data, latch);
int ledMaxBright = 4000; // 4095 is MAX brightness

// ---------------------------------------------------------------------------------------
//                       Debug - Verbose Flags
// ---------------------------------------------------------------------------------------
//#define SHADOW_DEBUG       //uncomment this for console DEBUG output

// ---------------------------------------------------------------------------------------
//                 Setup for USB, Bluetooth Dongle, & PS3 Controller
// ---------------------------------------------------------------------------------------
USB Usb;
BTD Btd(&Usb);
PS3BT *PS3Controller=new PS3BT(&Btd);

// ---------------------------------------------------------------------------------------
//                 Setup for Servo
// ---------------------------------------------------------------------------------------
Servo myServo;
int servoPos = 5;
boolean requestServoZero = false;
boolean requestServo180 = false;
boolean requestServo = false;


// ---------------------------------------------------------------------------------------
//                 Setup for Motors
// ---------------------------------------------------------------------------------------
int driveDeadBandRange = 10;
#define SABERTOOTH_ADDR 128
Sabertooth *ST=new Sabertooth(SABERTOOTH_ADDR, Serial1);

int requestSpeed = 0;
int requestTurn = 0;
int currentSpeed = 0;
int currentTurn = 0;
int rampUp = 0;
uint32_t lastRamp = millis();
int rampThresh = 100;
boolean requestStop = true;
boolean droidStopped = true;

// ---------------------------------------------------------------------------------------
//                 Setup for OLED
// ---------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------
//                 Setup for Sound
// ---------------------------------------------------------------------------------------
boolean randomSound = false ; // state variable for the random sound mode
boolean randomSoundInit = false;
boolean joystickUp = false;
boolean joystickUpInit = false;
boolean joystickStop = true;
boolean playJoystickStop = false;
long randomSoundMillis = millis();
int randomSong = 1;  // which sound are we playing?
long randomInterval = 1000;
long songLengths[] = {2000, 2000, 3500, 16000, 14000, 9000, 11000, 2000, 175000, 15000, 3000, 2000, 6000, 10000, 3000};
String songNames[] = {"MasterAction", "DoAgain", "Applause", "Tavern1", "Tavern2", "ZeldaMagic", "Victory", "StrongMagic", "Hobbits", "AmbientMagic", "Pint", "Papi", "Roll", "Bar Fight", "Worm"};
long firstStickUp;
long secondStickUp;

// ---------------------------------------------------------------------------------------
//                 Setup for sensors
// ---------------------------------------------------------------------------------------
#define TRIGGER_PIN_FRONT 9
#define ECHO_PIN_FRONT 8 

#define TRIGGER_PIN_LEFT 10
#define ECHO_PIN_LEFT 11

#define TRIGGER_PIN_BACK 12
#define ECHO_PIN_BACK 13

#define MAX_DISTANCE 400 // (in centimeters).

NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarBack(TRIGGER_PIN_BACK, ECHO_PIN_BACK, MAX_DISTANCE);

// State Variables for Autonomous Driving
boolean autoMode = false;
boolean autoInit = false;
boolean autogoStraight = false;

int autoSpeed = 90;

float wallDistance = 0;
float currentDistanceFront = 0;
float currentDistanceLeft = 0;
float currentDistanceBack = 0;

int adjustLeftTurnAmount = 0;
long autoInitTimer;
int autoInitWait = 2000;
int pingFrequencyFront = 75; // frequency of ping (in ms). 
int pingFrequencyLeft = 75;
int pingFrequencyBack = 75;

long pingTimerFront;
long pingTimerLeft;
long pingTimerBack;

int frontPingCount = 0;
float frontPingValues[9] = {0,0,0,0,0,0,0,0,0};

int leftPingCount = 0;
float leftPingValues[9] = {0,0,0,0,0,0,0,0,0};

int backPingCount = 0;
float backPingValues[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};



long turnPulse = 0;
long turn_time = 1550;
long autoPulse = 0;
bool autoTurnRight = false;

long initPulse = 0;
long initTime = 2000;
boolean startAutoMode = false;
boolean initAutoMode = false;
int forwardTurnCount = 0;

boolean startAutoBackwards = false;
boolean autogoBack = false;
boolean autoTurnLeft = false;
boolean initBackMode;
int backwardTurnCount = 0;
long initBackPulse;
long adjustPulse;

long callLeftSonar;
long callFrontSonar;
long callBackSonar;
long callOffset = 200;

int turnAdjust = 0;

// ---------------------------------------------------------------------------------------
//    Output String for Serial Monitor Output
// ---------------------------------------------------------------------------------------
char output[300];

// ---------------------------------------------------------------------------------------
//    Deadzone range for joystick to be considered at nuetral
// ---------------------------------------------------------------------------------------
byte joystickDeadZoneRange = 15;

// ---------------------------------------------------------------------------------------
//    Used for PS3 Fault Detection
// ---------------------------------------------------------------------------------------
uint32_t msgLagTime = 0;
uint32_t lastMsgTime = 0;
uint32_t currentTime = 0;
uint32_t lastLoopTime = 0;
int badPS3Data = 0;

boolean isPS3ControllerInitialized = false;
boolean mainControllerConnected = false;
boolean WaitingforReconnect = false;
boolean isFootMotorStopped = true;

// ---------------------------------------------------------------------------------------
//    Used for PS3 Controller Click Management
// ---------------------------------------------------------------------------------------
long previousMillis = millis();
boolean extraClicks = false;
int turnRate = 0;
// ---------------------------------------------------------------------------------------
//    Used for Pin 13 Main Loop Blinker
// ---------------------------------------------------------------------------------------
long blinkMillis = millis();
boolean blinkOn = false;

MP3Trigger MP3Trigger;

// =======================================================================================
//                                 Main Program
// =======================================================================================
// =======================================================================================
//                          Initialize - Setup Function
// =======================================================================================
void setup()
{
  
    //Debug Serial for use with USB Debugging
    Serial.begin(115200);
    while (!Serial);
    
    if (Usb.Init() == -1)
    {
        Serial.print(F("\r\nOSC did not start"));
        while (1); //halt
    }

    strcpy(output, "");
    
    Serial.print(F("\r\nBluetooth Library Started"));
    
    //Setup for PS3 Controller
    PS3Controller->attachOnInit(onInitPS3Controller); // onInitPS3Controller is called upon a new connection

//    pinMode(13, OUTPUT);
//    digitalWrite(13, LOW);

    //servo pin
    myServo.attach(2);
    myServo.write(servoPos);

    // Sabertooth Setup
    Serial1.begin(9600);
    ST->autobaud();
    ST->setTimeout(200);
    ST->setDeadband(driveDeadBandRange);

    pingTimerFront = millis(); // Start now.
    pingTimerLeft = millis() + 25;
    pingTimerBack = millis() + 50;

    // sound
    MP3Trigger.setup(&Serial3);
    Serial3.begin(MP3Trigger::serialRate());
    randomSeed(analogRead(1));

    //Start LED Controller
    LEDControl.begin();
    
}

// =======================================================================================
//           Main Program Loop - This is the recurring check loop for entire sketch
// =======================================================================================
void loop()
{   
   // Make sure the Bluetooth Dongle is working - skip main loop if not
   if ( !readUSB() )
   {
     //We have a fault condition that we want to ensure that we do NOT process any controller data
     printOutput(output);
     return;
   }
    
   // Check and output PS3 Controller inputs
   checkController();

   listenServo();
   //moveServo();

   // Ignore extra inputs from the PS3 Controller
   if (extraClicks)
   {
      if ((previousMillis + 500) < millis())
      {
          extraClicks = false;
      }
   }
   
    if(startAutoMode) {
//      Serial.print("FrontDistance: ");
//      Serial.print(currentDistanceFront);
//      Serial.print("\n");
//      Serial.print("WallDistance: ");
//      Serial.print(wallDistance);
//      Serial.print("\n");
      if(!initAutoMode) {
        if(autoPulse < initPulse) {
          if(millis() >= pingTimerFront) {
            pingTimerFront = millis() + pingFrequencyFront;
            sonarFront.ping_timer(setFrontDistance);
          }
          if(millis() >= pingTimerLeft) {
            pingTimerLeft = millis() + pingFrequencyLeft;
            sonarLeft.ping_timer(setLeftDistance);
            wallDistance = currentDistanceLeft;
          }
        } else {
          initAutoMode = true;
          autoMode = true;
          autogoStraight = true;
        }
      }

      if(millis() >= pingTimerFront) {
        pingTimerFront = millis() + pingFrequencyFront;
        sonarFront.ping_timer(setFrontDistance);
      }
//      if(autoPulse >= pingTimerLeft) {
//        pingTimerLeft = autoPulse + pingFrequencyLeft;
//        sonarLeft.ping_timer(setLeftDistance);
//      }
//      if(autoPulse >= pingTimerBack) {
//        pingTimerBack = autoPulse + pingFrequencyBack;
//        sonarBack.ping_timer(setBackDistance);
      //}
      if (autoMode && autogoStraight) {
        if(currentDistanceFront > wallDistance) {   

            ST->drive(-50);
            ST->turn(0);
          
          if(droidStopped) {
            droidStopped = false;
          }
          
        } else {
          
          if(!droidStopped) {
            ST->stop();
            droidStopped = true;
          }
        
          turnPulse = millis() + turn_time;
          autogoStraight = false;
          autoTurnRight = true;
      }
//      pingLeftSonar();
    } else if(autoTurnRight && forwardTurnCount < 3) {
      
      if (autoPulse < turnPulse) {
        ST->drive(-30);
        ST->turn(40);
    
      } else {
        forwardTurnCount++;
        autoTurnRight = false;
        autogoStraight = true;
      }
    } else if(autoTurnRight && forwardTurnCount >= 3) {
      ST->stop();
      startAutoMode = false;
      startAutoBackwards = true;
      initBackMode = false;
      initBackPulse = autoPulse + initTime;
    }
    
    autoPulse = millis();
  }


  if(startAutoBackwards) {
    if(!initBackMode) {
      if(autoPulse < initBackPulse) {
          pingBackSonar();
        } else {
          initBackMode = true;
          autoMode = true;
          autogoBack = true;
        }
    }
    pingBackSonar();
    if (autoMode && autogoBack) {
      if(currentDistanceBack > wallDistance) {
          ST->drive(50);
          ST->turn(0);
          
          if(droidStopped) {
            droidStopped = false;
          }
          
        } else {
          
          if(!droidStopped) {
            ST->stop();
            droidStopped = true;
          }
        
          turnPulse = millis() + turn_time;
          autogoBack = false;
          autoTurnLeft = true;
      }
    } else if (autoTurnLeft && backwardTurnCount < 3) {
      if (autoPulse < turnPulse) {
          ST->drive(30);
          ST->turn(-40);
    
      } else {
        backwardTurnCount++;
        autoTurnLeft = false;
        autogoBack = true;
      }
    } else if (autoTurnLeft && backwardTurnCount >= 3) {
      ST->stop();
      startAutoBackwards = false;
      autogoBack = false;
    }
    autoPulse = millis();
  }






  

    if(!startAutoMode && !startAutoBackwards) {
      setRequestMovement();
      moveDroidRamp();
    }
    //Control the Main Loop Blinker
   if ((LEDpulse + 500) < millis()) {
      if (LEDOn) {
        //digitalWrite(13, LOW);
        LEDControl.setPWM(0, 0);
        LEDOn = false;
      } else {
        //digitalWrite(13, HIGH);
        LEDControl.setPWM(0, ledMaxBright);
        LEDOn = true;
      }
      LEDpulse = millis();
   }

    // Regular Lights
   if(!partyMode && !leveling_up && !fight){  // when we have the other routines, update this statement
//    fireplace();
//    tavernLights();
//    magicLights(ledMaxBright);
//    LEDControl.write();
   }

   // Integrated Routine 1
   bardPerformance();
   
   // Integrated Routine 2
   levelUp();

   // Integrated Routine 3
   tavernFight();
   
   if(!partyMode && !leveling_up && !fight){
    initRandomSound();
    playRandomSound();
   }
   MP3Trigger.update();

   initStickSound();
   playStickSound();
   
   printOutput(output);
}

// =======================================================================================
//          Check Controller Function to show all PS3 Controller inputs are Working
// =======================================================================================
void checkController()
{
       if (PS3Controller->PS3Connected && PS3Controller->getButtonPress(UP) && !extraClicks)
     {              
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: UP Selected.\r\n");
            #endif
            
            previousMillis = millis();
            extraClicks = true;
            
     }
  
     if (PS3Controller->PS3Connected && PS3Controller->getButtonPress(DOWN) && !extraClicks)
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: DOWN Selected.\r\n");
            #endif                     
            
            previousMillis = millis();
            extraClicks = true;
       
     }

     if (PS3Controller->PS3Connected && PS3Controller->getButtonPress(LEFT) && !extraClicks)
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: LEFT Selected.\r\n");
            #endif  
            
            previousMillis = millis();
            extraClicks = true;

     }
     
     if (PS3Controller->PS3Connected && PS3Controller->getButtonPress(RIGHT) && !extraClicks)
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: RIGHT Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;
                     
     }
     
     if (PS3Controller->PS3Connected && PS3Controller->getButtonPress(CIRCLE) && !extraClicks)
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: CIRCLE Selected.\r\n");
            #endif      
            
            previousMillis = millis();
            extraClicks = true;

            startAutoMode = false;
            startAutoBackwards = false;
            autogoBack = false;
            autoMode = false;
            requestStop = true;
         
     }

     if (PS3Controller->PS3Connected && PS3Controller->getButtonPress(CROSS) && !extraClicks)
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: CROSS Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;
            
            randomSound = false;  // deactivate random sound mode
              
     }
     
     if (PS3Controller->PS3Connected && PS3Controller->getButtonPress(TRIANGLE) && !extraClicks)
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: TRIANGLE Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;
            
            randomSound = true;  // activate random sound mode
              
     }
     

     if (PS3Controller->PS3Connected && PS3Controller->getButtonPress(SQUARE) && !extraClicks)
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: SQUARE Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;
            startAutoMode = true;
            initAutoMode = false;
            initBackMode = false;
            forwardTurnCount = 0;
            backwardTurnCount = 0;
            autoPulse = millis();
            pingTimerFront = millis(); // Start now.
            pingTimerLeft = millis() + 25;
            pingTimerBack = millis() + 50;
            initPulse = autoPulse + initTime;
//            adjustPulse = initPulse + 250;
            //autoMode = true;
            //autoInit = true;
            
              
     }
     
     if (PS3Controller->PS3Connected && !extraClicks && PS3Controller->getButtonPress(L1))
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: LEFT 1 Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;

            if(brawlFight){
              brawlFight = false;
            } else if(!brawlFight){
              brawlFight = true;
            }
     }

     if (PS3Controller->PS3Connected && !extraClicks && PS3Controller->getButtonPress(L2))
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: LEFT 2 Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;

            if(!partyMode) {
              partyMode = true;  // ACTIVATE PARTY MODE
              doPartyLights = true;
              playPartySound1 = true;
              playPartySound2 = false;
              playApplause = false;
              donut1Sound = true;
              donut2Sound = true;
              partyPulse = millis();  // get the time that the party starts
              intro_forward = partyPulse + 1000;
              partySound1Pulse = intro_forward + songLengths[2];
              intro_backwards = partySound1Pulse + 1000;
              donutPulse = intro_backwards + 10000;
              partySound2Pulse = donutPulse + 2500;
              donutPulse2 = partySound2Pulse + 10000;
              applause = donutPulse2 + 4000;
            } else {
              partyMode = false;
              MP3Trigger.stop();
              ST->stop();
            }
     }

     if (PS3Controller->PS3Connected && !extraClicks && PS3Controller->getButtonPress(R1))
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: RIGHT 1 Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;

            if(!leveling_up){
              leveling_up = true;
              levelSound1 = true;
              levelSound2 = false;
              levelSound3 = false;
              levelSound4 = false;
              levelPulse = millis();  // beginning of routine 2
              preStrongMagicPulse = levelPulse + 2000;
              strongMagicPulse = preStrongMagicPulse + songLengths[7];     // 8 - 1
              ambientMagicPulse = strongMagicPulse + songLengths[9] + 500;  // 10 - 1
              ambientLightPulse = strongMagicPulse;
              powerUpPulse = ambientMagicPulse + songLengths[5] + 500; // 6 - 1
              wigglePulse = ambientMagicPulse;
              powerUpClimaxPulse = powerUpPulse - 2500;
              climaxLightPulse = powerUpClimaxPulse;
              climaxOn = true;
              randomAmbientPulse = ambientMagicPulse;
              fanfarePulse = powerUpPulse + songLengths[6] + 100;  // 7 - 1
              fanfareLightPulse = powerUpPulse;
            } else if(leveling_up) {
              leveling_up = false;
              ST->stop();
            }
     }

     if (PS3Controller->PS3Connected && !extraClicks && PS3Controller->getButtonPress(R2))
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: RIGHT 2 Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;

            if(!fight){
              fight = true;
              playWorm = true;
              playPapi = false;
              papiTalk = false;
              playFightMusic = false;
              playPint = false;
              diceDropped = false;
              wormTalk = false;
              fightPulse = millis();
              talk_interval = fightPulse;
              wormPulse = fightPulse + songLengths[14] + 250;
              papiPulse = wormPulse + songLengths[11]; // 12 - 1
              rollPulse = papiPulse + songLengths[12]; // 13 - 1
              dicePulse = rollPulse + 500;
              diceWaitPulse = dicePulse + 1000;
              ambientTavernPulse = diceWaitPulse;
              hitPulse = diceWaitPulse;
              fightMusicPulse = diceWaitPulse + songLengths[13]; // 14 - 1
              pintPulse = fightMusicPulse + songLengths[10]; // 11 - 1
            } else if(fight){
              fight = false;
            }
     }

     if (PS3Controller->PS3Connected && !extraClicks && PS3Controller->getButtonPress(SELECT))
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: SELECT Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;
     }

     if (PS3Controller->PS3Connected && !extraClicks && PS3Controller->getButtonPress(START))
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: START Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;
     }

     if (PS3Controller->PS3Connected && !extraClicks && PS3Controller->getButtonPress(PS))
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: PS Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;
     }

     if (PS3Controller->PS3Connected && ((abs(PS3Controller->getAnalogHat(LeftHatY)-128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(LeftHatX)-128) > joystickDeadZoneRange)))
     {
            
            int currentValueY = PS3Controller->getAnalogHat(LeftHatY) - 128;
            int currentValueX = PS3Controller->getAnalogHat(LeftHatX) - 128;
            
            char yString[5];
            itoa(currentValueY, yString, 10);

            char xString[5];
            itoa(currentValueX, xString, 10);

            #ifdef SHADOW_DEBUG
                strcat(output, "LEFT Joystick Y Value: ");
                strcat(output, yString);
                strcat(output, "\r\n");
                strcat(output, "LEFT Joystick X Value: ");
                strcat(output, xString);
                strcat(output, "\r\n");
            #endif

            if(abs(currentValueY)>0 && abs(currentValueX)>0){
              if(joystickStop){
                joystickUp = true;
                joystickStop = false;
              }
            }
     }

     if (PS3Controller->PS3Connected && ((abs(PS3Controller->getAnalogHat(RightHatY)-128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(RightHatX)-128) > joystickDeadZoneRange)))
     {
            int currentValueY = PS3Controller->getAnalogHat(RightHatY) - 128;
            int currentValueX = PS3Controller->getAnalogHat(RightHatX) - 128;

            char yString[5];
            itoa(currentValueY, yString, 10);

            char xString[5];
            itoa(currentValueX, xString, 10);

            #ifdef SHADOW_DEBUG
                strcat(output, "RIGHT Joystick Y Value: ");
                strcat(output, yString);
                strcat(output, "\r\n");
                strcat(output, "RIGHT Joystick X Value: ");
                strcat(output, xString);
                strcat(output, "\r\n");
            #endif       
     }
}

// =======================================================================================
//           PPS3 Controller Device Mgt Functions
// =======================================================================================
// =======================================================================================
//           Initialize the PS3 Controller Trying to Connect
// =======================================================================================
void onInitPS3Controller(){
    PS3Controller->setLedOn(LED1);
    isPS3ControllerInitialized = true;
    badPS3Data = 0;

    mainControllerConnected = true;
    WaitingforReconnect = true;

    #ifdef SHADOW_DEBUG
       strcat(output, "\r\nWe have the controller connected.\r\n");
       Serial.print("\r\nDongle Address: ");
       String dongle_address = String(Btd.my_bdaddr[5], HEX) + ":" + String(Btd.my_bdaddr[4], HEX) + ":" + String(Btd.my_bdaddr[3], HEX) + ":" + String(Btd.my_bdaddr[2], HEX) + ":" + String(Btd.my_bdaddr[1], HEX) + ":" + String(Btd.my_bdaddr[0], HEX);
       Serial.println(dongle_address);
    #endif
}

// =======================================================================================
//           Determine if we are having connection problems with the PS3 Controller
// =======================================================================================
boolean criticalFaultDetect(){
    if (PS3Controller->PS3Connected){
        
        currentTime = millis();
        lastMsgTime = PS3Controller->getLastMessageTime();
        msgLagTime = currentTime - lastMsgTime;            
        
        if (WaitingforReconnect){
            
            if (msgLagTime < 200){
                WaitingforReconnect = false; 
            }
            
            lastMsgTime = currentTime;
        } 
        
        if ( currentTime >= lastMsgTime){
              msgLagTime = currentTime - lastMsgTime;  
        } else {
             msgLagTime = 0;
        }
        
        if (msgLagTime > 300 && !isFootMotorStopped){
            #ifdef SHADOW_DEBUG
              strcat(output, "It has been 300ms since we heard from the PS3 Controller\r\n");
              strcat(output, "Shut down motors and watching for a new PS3 message\r\n");
            #endif
            
//          You would stop all motors here
            isFootMotorStopped = true;
        }
        
        if ( msgLagTime > 10000 ){
            #ifdef SHADOW_DEBUG
              strcat(output, "It has been 10s since we heard from Controller\r\n");
              strcat(output, "\r\nDisconnecting the controller.\r\n");
            #endif
            
//          You would stop all motors here
            isFootMotorStopped = true;
            
            PS3Controller->disconnect();
            WaitingforReconnect = true;
            return true;
        }

        //Check PS3 Signal Data
        if(!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged)){
            //We don't have good data from the controller.
            //Wait 15ms - try again
            delay(15);
            Usb.Task();   
            lastMsgTime = PS3Controller->getLastMessageTime();
            
            if(!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged)){
                badPS3Data++;
                #ifdef SHADOW_DEBUG
                    strcat(output, "\r\n**Invalid data from PS3 Controller. - Resetting Data**\r\n");
                #endif
                return true;
            }
        } else if (badPS3Data > 0){
            badPS3Data = 0;
        }
        
        if ( badPS3Data > 10 ){
            #ifdef SHADOW_DEBUG
                strcat(output, "Too much bad data coming from the PS3 Controller\r\n");
                strcat(output, "Disconnecting the controller and stop motors.\r\n");
            #endif
            
//          You would stop all motors here
            isFootMotorStopped = true;
            
            PS3Controller->disconnect();
            WaitingforReconnect = true;
            return true;
        }
    } else if (!isFootMotorStopped) {
        #ifdef SHADOW_DEBUG      
            strcat(output, "No PS3 controller was found\r\n");
            strcat(output, "Shuting down motors and watching for a new PS3 message\r\n");
        #endif
        
//      You would stop all motors here
        isFootMotorStopped = true;
        
        WaitingforReconnect = true;
        return true;
    }
    
    return false;
}

// =======================================================================================
//           USB Read Function - Supports Main Program Loop
// =======================================================================================
boolean readUSB(){
  
     Usb.Task();
     
    //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
    if (PS3Controller->PS3Connected) 
    {
        if (criticalFaultDetect())
        {
            //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
            printOutput(output);
            return false;
        }
        
    } else if (!isFootMotorStopped)
    {
        #ifdef SHADOW_DEBUG      
            strcat(output, "No controller was found\r\n");
            strcat(output, "Shuting down motors, and watching for a new PS3 foot message\r\n");
        #endif
        
//      You would stop all motors here
        isFootMotorStopped = true;
        
        WaitingforReconnect = true;
    }
    
    return true;
}

// =======================================================================================
//          Motor Control
// =======================================================================================
// Set the REQUEST state for drive motors
// Motor controller range is -127 to 127 for drive and turn
// Joystick values are 0-255 for x and y

void setRequestMovement() {
  if(PS3Controller->PS3Connected) {
    requestSpeed = PS3Controller->getAnalogHat(LeftHatY) - 128;
    requestTurn = PS3Controller->getAnalogHat(LeftHatX) - 128;
    if (requestSpeed > 50) {
      requestSpeed = 50;
    } else if(requestSpeed < -50) {
      requestSpeed = -50;
    }
    if (requestTurn > 30) {
      requestTurn = 30;
    } else if(requestTurn < -30) {
      requestTurn = -30;
    }
   
    if(abs(requestSpeed) <= joystickDeadZoneRange) {
      requestSpeed = 0;
    }
    if(abs(requestTurn) <= joystickDeadZoneRange) {
      requestTurn = 0;
    }
    if (requestSpeed == 0 && requestTurn == 0) {
      requestStop = true;
    } else {
      requestStop = false;
    }
  }
}

void moveDroid() {
  if (!requestStop) {
    currentSpeed = requestSpeed;
    currentTurn = requestTurn;
    ST->turn(currentTurn);
    ST->drive(currentSpeed);
    if(droidStopped) {
      droidStopped = false;
    } else {
      if(!droidStopped) {
        ST->stop();
        droidStopped = true;
      }
     }
   }
 }


void moveDroidRamp() {
  boolean timeToRamp = false;
  if (millis() - lastRamp >= 250) {
    timeToRamp = true;
  }
  int deltaSpeed = requestSpeed - currentSpeed;
  if(deltaSpeed >= rampThresh && timeToRamp) {
    rampUp = 1;
  } else {
    rampUp = 0;
  }
  
  if(!requestStop) {
    switch(rampUp) {
      case 0:
        currentSpeed = requestSpeed;
//        strcat(output, "Not Ramping. Speed: ");
//        strcat(output, currentSpeed);
        currentTurn = requestTurn;
        ST->turn(currentTurn);
        ST->drive(currentSpeed);
        break;
      case 1:
        currentSpeed += 10;
        if(currentSpeed >= requestSpeed) {
          rampUp = 0;
        }
//        strcat(output, "RAMPING. Speed: ");
//        strcat(output, currentSpeed);
        currentTurn = requestTurn;
        ST->turn(currentTurn);
        ST->drive(currentSpeed);
        break;
      }
    if(droidStopped) {
      droidStopped = false;
    }
  } else {
    if(!droidStopped) {
        ST->stop();
        droidStopped = true;
    }
  }
}

void autoMoveForward() {
  if (autoMode && autoInit && autogoStraight) {
    if(currentDistanceFront > 10.0) {
//      Serial.print("\r\nCurrentDistance: ");
//      Serial.print(currentDistanceFront);
//      Serial.print("  wallDistance: ");
//      Serial.print(wallDistance);
      if(!blinkOn) {
        digitalWrite(13, HIGH);
        blinkOn = true;
      }
      ST->drive(-50);
      ST->turn(0);
      if(droidStopped) {
        droidStopped = false;
      }
     } else {
      if(blinkOn) {
        digitalWrite(13, LOW);
        blinkOn = false;
      }
      if(!droidStopped) {
        ST->stop();
        droidStopped = true;
      }
//        Serial.print("\r\nDroid Stopped");
    }
  }
}


// =======================================================================================
//          Sensor Functions
// =======================================================================================
void pingFrontSonar() {
  if(millis() >= pingTimerFront) {
    pingTimerFront = millis() + pingFrequencyFront;
    sonarFront.ping_timer(setFrontDistance);
  }
}

void pingLeftSonar() {
  if(millis() >= pingTimerLeft) {
    pingTimerLeft = millis() + pingFrequencyLeft;
    sonarLeft.ping_timer(setLeftDistance);
  }
}

void setLeftDistance() {
  if(sonarLeft.check_timer()) {
    float lDist = (sonarLeft.ping_result / US_ROUNDTRIP_CM) * 0.39370079;
//    Serial.print("\r\nLeftPing: ");
//    Serial.print(lDist);
    if(leftPingCount < 9) {
      leftPingValues[leftPingCount] = lDist;
//      currentDistanceLeft = calcAvg(leftPingCount, leftPingValues);
    } else {
      currentDistanceLeft = sortSelect(leftPingCount, leftPingValues);
      leftPingCount = 0;
      leftPingValues[leftPingCount] = lDist;
    }
    leftPingCount++;
  }
}

void setFrontDistance() {
  //Serial.print("\r\nTop of function");
  if(sonarFront.check_timer()) {
    float fDist = (sonarFront.ping_result / US_ROUNDTRIP_CM) * 0.39370079;
    if (frontPingCount < 9) {
      frontPingValues[frontPingCount] = fDist;
      currentDistanceFront = sortSelect(frontPingCount, frontPingValues);
    } else {
      currentDistanceFront = sortSelect(frontPingCount, frontPingValues);
      frontPingCount = 0;
      frontPingValues[frontPingCount] = fDist;
    }
    frontPingCount++;
  }
}

void pingBackSonar() {
  if(millis() >= pingTimerBack) {
//    Serial.print("\npinging back\n");
    pingTimerBack = millis() + pingFrequencyBack;
    sonarBack.ping_timer(setBackDistance);
  }
}



void setBackDistance() {
  if(sonarBack.check_timer()) {
    float bDist = (sonarBack.ping_result / US_ROUNDTRIP_CM) * 0.39370079;
//    Serial.print("Back REading: ");
//    Serial.print(bDist);
//    Serial.print("\n");
    if (backPingCount < 9) {
      backPingValues[backPingCount] = bDist;
      currentDistanceBack = sortSelect(backPingCount, backPingValues);
    } else {
      currentDistanceBack = sortSelect(backPingCount, backPingValues);
      backPingCount = 0;
      backPingValues[backPingCount] = bDist;
    }
    backPingCount++;
  }
}


float calcAvg(int len, float dist[]) { // fix func
  float sum = 0;
  //int len = sizeof(dist) / sizeof(dist[0]);
  for(int i = 0; i < len; i++) {
    sum += dist[i];
  }
  return sum / len;
}

void swap(float *xp, float *yp){ // swap function
  float temp = *xp;
  *xp = *yp;
  *yp = temp;
}

float sortSelect(int len, float dist[]) { // fix func
  // SORT
  int i, j;
  for (i = 0; i < len-1; i++){    
    // Last i elements are already in place
    for (j = 0; j < len-i-1; j++){
      if (dist[j] > dist[j+1]){
        swap(&dist[j], &dist[j+1]);
      }
    }
  }

  // FIND MEDIAN
  if(len%2 != 0){ // if there are an uneven amount of current pings
    return dist[len/2];
  } else {        // if there are an even amount of current pings
    return (dist[(len - 1) / 2] + dist[len / 2]) / 2.0;
  }
}


// =======================================================================================
//          Sound Functions
// =======================================================================================
void initRandomSound(){
  if(randomSound){
    //Serial.print("random sound mode activated\n");
    if(!randomSoundInit){
      randomInterval = random(1,6) * 1000;
      randomSong = random(1,3);
      //Serial.print(randomSong);
      //Serial.print(" ");
      //Serial.print(songLengths[randomSong - 1]);
      //Serial.print("\n");
      randomSoundMillis = millis() + randomInterval + songLengths[randomSong-1];
      randomSoundInit = true;
    }
  }

//  playRandomSound();
}

void playRandomSound(){
  if(randomSound && randomSoundInit){
    if(millis() > randomSoundMillis){
      
      MP3Trigger.trigger(randomSong);
      randomSoundInit = false;
      
    }
  }
}

void initStickSound(){
  if(joystickUp){
    if(!joystickUpInit){
      MP3Trigger.setVolume(60);
      firstStickUp = millis() + 2000;
      secondStickUp = firstStickUp + 1000;
      joystickUpInit = true;
      MP3Trigger.trigger(9);
    }
  }
}

void playStickSound(){
  if(joystickUp && joystickUpInit){
    if(millis() > secondStickUp){
      //Serial.print("Second volume increase\n");
      MP3Trigger.setVolume(0);
    } else if(millis() > firstStickUp){
      //Serial.print("First volume increase\n");
      MP3Trigger.setVolume(30);
    }
  } else if(joystickStop && playJoystickStop){
    //Serial.print("stopSound playing\n");
    MP3Trigger.setVolume(0);
    MP3Trigger.trigger(1);
    playJoystickStop = false;
  }
}

// =======================================================================================
//          Servo Functions
// =======================================================================================

void listenServo() {
  if (PS3Controller->PS3Connected && PS3Controller->getButtonPress(LEFT) && !extraClicks)
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: LEFT Selected.\r\n");
            #endif  
            
            previousMillis = millis();
            extraClicks = true;
            //requestServoZero = true;
            myServo.write(2);
     }
   if (PS3Controller->PS3Connected && PS3Controller->getButtonPress(RIGHT) && !extraClicks)
     {
            #ifdef SHADOW_DEBUG
                strcat(output, "Button: RIGHT Selected.\r\n");
            #endif       
            
            previousMillis = millis();
            extraClicks = true;
            //requestServo180 = true;
            myServo.write(179);
                     
     }
}

void moveServo() {
  if(requestServoZero) {
    //Serial.print("moving Servo to 0\n");
    myServo.write(2);
    requestServoZero = false;
  } else if(requestServo180) {
    //Serial.print("moving Servo to 180\n");
    myServo.write(179);
    requestServo180 = false;
  }
}

// =======================================================================================
//          Light Functions
// =======================================================================================

void applause_lights() {
  if(clap_on) {
    LEDControl.setPWM(5, ledMaxBright);
    LEDControl.setPWM(6, ledMaxBright);
    LEDControl.setPWM(7, ledMaxBright);
    LEDControl.setPWM(8, ledMaxBright);
    LEDControl.setPWM(9, ledMaxBright);
    LEDControl.setPWM(10, ledMaxBright);
    LEDControl.setPWM(11, ledMaxBright);
    clap_on = !clap_on;
  } else {
    LEDControl.setPWM(5, 0);
    LEDControl.setPWM(6, 0);
    LEDControl.setPWM(7, 0);
    LEDControl.setPWM(8, 0);
    LEDControl.setPWM(9, 0);
    LEDControl.setPWM(10, 0);
    LEDControl.setPWM(11, 0);
    clap_on = !clap_on;
  }
  LEDControl.write();
}

void led_voice() {
    if(talk_on) {
      LEDControl.setPWM(18, ledMaxBright);
      LEDControl.setPWM(19, ledMaxBright);
      LEDControl.setPWM(20, ledMaxBright);
      LEDControl.setPWM(21, ledMaxBright);
      LEDControl.setPWM(22, ledMaxBright);
      LEDControl.setPWM(23, ledMaxBright);
      talk_on = !talk_on;
    } else {
      LEDControl.setPWM(18, 0);
      LEDControl.setPWM(19, 0);
      LEDControl.setPWM(20, 0);
      LEDControl.setPWM(21, 0);
      LEDControl.setPWM(22, 0);
      LEDControl.setPWM(23, 0);
      talk_on = !talk_on;
  }
  LEDControl.write();
}

void ambientLights(){
  long currTime = millis();
  if(currTime >= ambientLightPulse){  // if the current time is greater than the time to pulse
    if(ambientLightLevel > 1500 || ambientLightLevel < 0){
      ambientLightChange = -1 * ambientLightChange;
      ambientLightLevel += ambientLightChange;
    }
      LEDControl.setPWM(0, ambientLightLevel);
      LEDControl.setPWM(1, ambientLightLevel);
      LEDControl.setPWM(2, ambientLightLevel);
      LEDControl.setPWM(3, ambientLightLevel);
      LEDControl.setPWM(4, ambientLightLevel);
      LEDControl.setPWM(5, ambientLightLevel);
      LEDControl.setPWM(6, ambientLightLevel);
      LEDControl.setPWM(7, ambientLightLevel);
      LEDControl.setPWM(8, ambientLightLevel);
      LEDControl.setPWM(9, ambientLightLevel);
      LEDControl.setPWM(10, ambientLightLevel);
      LEDControl.setPWM(11, ambientLightLevel);
      LEDControl.setPWM(12, ambientLightLevel);
      LEDControl.setPWM(13, ambientLightLevel);
      LEDControl.setPWM(14, ambientLightLevel);
      LEDControl.setPWM(15, ambientLightLevel);
      LEDControl.setPWM(16, ambientLightLevel);
      LEDControl.setPWM(17, ambientLightLevel);
      LEDControl.setPWM(18, ambientLightLevel);
      LEDControl.setPWM(19, ambientLightLevel);
      LEDControl.setPWM(20, ambientLightLevel);
      LEDControl.setPWM(21, ambientLightLevel);
      LEDControl.setPWM(22, ambientLightLevel);
      LEDControl.setPWM(23, ambientLightLevel);
      ambientLightLevel += ambientLightChange;
    ambientLightPulse = currTime + ambientLightInterval;
    LEDControl.write();
  }
}

void randomIncrement(){
  long currTime = millis();
  if(currTime >= randomAmbientPulse){
  LEDControl.setPWM(0, random(0,2)*randomAmbient);
  LEDControl.setPWM(1, random(0,2)*randomAmbient);
  LEDControl.setPWM(2, random(0,2)*randomAmbient);
  LEDControl.setPWM(3, random(0,2)*randomAmbient);
  LEDControl.setPWM(4, random(0,2)*randomAmbient);
  LEDControl.setPWM(5, random(0,2)*randomAmbient);
  LEDControl.setPWM(6, random(0,2)*randomAmbient);
  LEDControl.setPWM(7, random(0,2)*randomAmbient);
  LEDControl.setPWM(8, random(0,2)*randomAmbient);
  LEDControl.setPWM(9, random(0,2)*randomAmbient);
  LEDControl.setPWM(10, random(0,2)*randomAmbient);
  LEDControl.setPWM(11, random(0,2)*randomAmbient);
  LEDControl.setPWM(12, random(0,2)*randomAmbient);
  LEDControl.setPWM(13, random(0,2)*randomAmbient);
  LEDControl.setPWM(14, random(0,2)*randomAmbient);
  LEDControl.setPWM(15, random(0,2)*randomAmbient);
  LEDControl.setPWM(16, random(0,2)*randomAmbient);
  LEDControl.setPWM(17, random(0,2)*randomAmbient);
  LEDControl.setPWM(18, random(0,2)*randomAmbient);
  LEDControl.setPWM(19, random(0,2)*randomAmbient);
  LEDControl.setPWM(20, random(0,2)*randomAmbient);
  LEDControl.setPWM(21, random(0,2)*randomAmbient);
  LEDControl.setPWM(22, random(0,2)*randomAmbient);
  LEDControl.setPWM(23, random(0,2)*randomAmbient);
  LEDControl.write();
  randomAmbient += randomAmbientChange;
  randomAmbientPulse = currTime + randomAmbientInterval;
  }
}

void wormLights() {
  if(talk_on) {
    redLights(ledMaxBright);
    talk_on = !talk_on;
  } else {
    redLights(0);
    talk_on = !talk_on;
  }
  LEDControl.write();
}

void papiLights() {
  if(talk_on) {
    yellowLights(ledMaxBright);
    talk_on = !talk_on;
  } else {
    yellowLights(0);
    talk_on = !talk_on;
  }
  LEDControl.write();
}


void ambientTavern() {
  LEDControl.setPWM(5, random(0, 2)* ledMaxBright);
  LEDControl.setPWM(6, random(0, 2)* ledMaxBright);
  LEDControl.setPWM(7, random(0, 2)* ledMaxBright);
  LEDControl.setPWM(8, random(0, 2)* ledMaxBright);
  LEDControl.setPWM(9, random(0, 2)* ledMaxBright);
  LEDControl.setPWM(10, random(0, 2)* ledMaxBright);
  LEDControl.setPWM(11, random(0, 2)* ledMaxBright);
  LEDControl.write();
}

void climaxLights() {
  long currTime = millis();
  if(currTime >= climaxLightPulse) {
    if(climaxOn) {
      yellowLights(ledMaxBright);
      redLights(ledMaxBright);
      magicLights(ledMaxBright);
      climaxOn = !climaxOn;
    } else {
      yellowLights(0);
      redLights(0);
      magicLights(0);
      climaxOn = !climaxOn;
    }
    climaxLightPulse = currTime + climaxInterval;
    LEDControl.write();
  }
}


void fanfareLights(){
  long currTime = millis();
  if(currTime >= fanfareLightPulse){
    if(talk_on) {
      yellowLights(ledMaxBright);
      redLights(ledMaxBright);
      magicLights(ledMaxBright);
      talk_on = !talk_on;
    } else {
      yellowLights(0);
      redLights(0);
      magicLights(0);
      talk_on = !talk_on;
    }
    fanfareLightPulse = currTime + fanfareInterval;
    LEDControl.write();
  }
}

void fireplace(){
  if((fireplacePulse + 200) < millis()) {
    LEDControl.setPWM(12, random(0,2)*ledMaxBright);
    LEDControl.setPWM(13, random(0,2)*ledMaxBright);
    LEDControl.setPWM(14, random(0,2)*ledMaxBright);
    LEDControl.setPWM(15, random(0,2)*ledMaxBright);
    fireplacePulse = millis();
  }
}

void yellowLights(long brightness){
  LEDControl.setPWM(5, brightness);
  LEDControl.setPWM(6, brightness);
  LEDControl.setPWM(7, brightness);
  LEDControl.setPWM(8, brightness);
  LEDControl.setPWM(9, brightness);
  LEDControl.setPWM(10, brightness);
  LEDControl.setPWM(11, brightness);
}

void redLights(long brightness){
  LEDControl.setPWM(0, brightness);
  LEDControl.setPWM(1, brightness);
  LEDControl.setPWM(2, brightness);
  LEDControl.setPWM(3, brightness);
  LEDControl.setPWM(4, brightness);
  LEDControl.setPWM(16, brightness);
  LEDControl.setPWM(17, brightness);
}

void tavernLights(){
  if(!brawlFight){ // yellow lights on, red lights off
    yellowLights(ledMaxBright);
    redLights(0);
  } else if(brawlFight) { // red lights on, yellow lights off
    redLights(ledMaxBright);
    yellowLights(0);
  }
}

void magicLights(long brightness){
  LEDControl.setPWM(18, brightness);
  LEDControl.setPWM(19, brightness);
  LEDControl.setPWM(20, brightness);
  LEDControl.setPWM(21, brightness);
  LEDControl.setPWM(22, brightness);
  LEDControl.setPWM(23, brightness);
}

void clearLights() {
  LEDControl.setPWM(0, 0);
  LEDControl.setPWM(1, 0);
  LEDControl.setPWM(2, 0);
  LEDControl.setPWM(3, 0);
  LEDControl.setPWM(4, 0);
  LEDControl.setPWM(5, 0);
  LEDControl.setPWM(6, 0);
  LEDControl.setPWM(7, 0);
  LEDControl.setPWM(8, 0);
  LEDControl.setPWM(9, 0);
  LEDControl.setPWM(10, 0);
  LEDControl.setPWM(11, 0);
  LEDControl.setPWM(12, 0);
  LEDControl.setPWM(13, 0);
  LEDControl.setPWM(14, 0);
  LEDControl.setPWM(15, 0);
  LEDControl.setPWM(16, 0);
  LEDControl.setPWM(17, 0);
  LEDControl.setPWM(18, 0);
  LEDControl.setPWM(19, 0);
  LEDControl.setPWM(20, 0);
  LEDControl.setPWM(21, 0);
  LEDControl.setPWM(22, 0);
  LEDControl.setPWM(23, 0);
  LEDControl.write();  // update the LEDs
}

// ---------------------------------------------------------------------------------------
//                       Integrated Mode 1 - Party at the Tavern
// ---------------------------------------------------------------------------------------

void partyModeLights(){
  if((partyLightPulse + 300) < millis()){
  LEDControl.setPWM(0, random(0,2)*ledMaxBright);
  LEDControl.setPWM(1, random(0,2)*ledMaxBright);
  LEDControl.setPWM(2, random(0,2)*ledMaxBright);
  LEDControl.setPWM(3, random(0,2)*ledMaxBright);
  LEDControl.setPWM(4, random(0,2)*ledMaxBright);
  LEDControl.setPWM(5, random(0,2)*ledMaxBright);
  LEDControl.setPWM(6, random(0,2)*ledMaxBright);
  LEDControl.setPWM(7, random(0,2)*ledMaxBright);
  LEDControl.setPWM(8, random(0,2)*ledMaxBright);
  LEDControl.setPWM(9, random(0,2)*ledMaxBright);
  LEDControl.setPWM(10, random(0,2)*ledMaxBright);
  LEDControl.setPWM(11, random(0,2)*ledMaxBright);
  LEDControl.setPWM(12, random(0,2)*ledMaxBright);
  LEDControl.setPWM(13, random(0,2)*ledMaxBright);
  LEDControl.setPWM(14, random(0,2)*ledMaxBright);
  LEDControl.setPWM(15, random(0,2)*ledMaxBright);
  LEDControl.setPWM(16, random(0,2)*ledMaxBright);
  LEDControl.setPWM(17, random(0,2)*ledMaxBright);
  LEDControl.setPWM(18, random(0,2)*ledMaxBright);
  LEDControl.setPWM(19, random(0,2)*ledMaxBright);
  LEDControl.setPWM(20, random(0,2)*ledMaxBright);
  LEDControl.setPWM(21, random(0,2)*ledMaxBright);
  LEDControl.setPWM(22, random(0,2)*ledMaxBright);
  LEDControl.setPWM(23, random(0,2)*ledMaxBright);
  LEDControl.write();  // update the LEDs
  partyLightPulse = millis();
  }
}

void bardPerformance() {
  if(partyMode){
      if(doPartyLights) {
        partyModeLights();
      }
    
      if(partyPulse <= intro_forward) {  // drive forward
        ST->turn(0);
        ST->drive(-50);
      }
      
      if(playPartySound1 && partyPulse >= intro_forward){  // "observe a master in action"
        ST->stop();
        MP3Trigger.trigger(1);
        playPartySound1 = false;
        playPartySound2 = true;
        talking_lights = true;
        doPartyLights = false;
        clearLights();
        MP3Trigger.update();
      }

      if(talking_lights) {
        if(partyPulse >= talk_interval) {
          led_voice();
          talk_interval = partyPulse + short_pause;
        }
      }

      if(partyPulse >= partySound1Pulse && partyPulse <= intro_backwards) {  // drive backwards
        talking_lights = false;
        doPartyLights = true;
        ST->turn(0);
        ST->drive(50);
        //partyPulse = millis();
      }
      
      if(partyPulse <= donutPulse && !playPartySound1 && partyPulse >= intro_backwards){  // time to do the donuts
        if(donut1Sound){
          MP3Trigger.trigger(4);
          MP3Trigger.update();
          donut1Sound = false;
        }
        ST->turn(-100);
        ST->drive(-100);
      }

      if(playPartySound2 && partyPulse >= donutPulse){  // "wanna see me do it again??"
        ST->stop();
        MP3Trigger.trigger(2);
        playPartySound2 = false;
        playApplause = true;
        talking_lights = true;
        doPartyLights = false;
        clearLights();
        MP3Trigger.update();
      }

      if(partyPulse < donutPulse2 && partyPulse >= partySound2Pulse){  // donuts round 2 electric boogaloo
        if(donut2Sound){
          MP3Trigger.trigger(5);
          MP3Trigger.update();
          donut2Sound = false;
          talking_lights = false;
          doPartyLights = true;
        }
        ST->turn(100);
        ST->drive(-100);
      }

      if(playApplause && partyPulse >= donutPulse2){  // applause
        ST->stop();
        MP3Trigger.trigger(3);
        MP3Trigger.update();
        doApplauseLights = true;
        doPartyLights = false;
        clearLights();
        playApplause = false;
      }

      if(doApplauseLights) {
        if(partyPulse >= clap_interval) {
          applause_lights();
          clap_interval = partyPulse + clap_pause;
        }
      }

      if(partyPulse >= applause){
        partyMode = false;
        doApplauseLights = false;
      }
      
      partyPulse = millis();
   }
}

// ---------------------------------------------------------------------------------------
//                       Integrated Mode 2 - Level Up
// ---------------------------------------------------------------------------------------
void levelUp(){
  if(leveling_up) {
    if(levelPulse <= preStrongMagicPulse){
      ST->turn(0);
      ST->drive(-50);
    }
    
    if(levelPulse > preStrongMagicPulse && levelPulse <= strongMagicPulse){  // 1. "I sense strong magic..."
      if(levelSound1){
        MP3Trigger.trigger(8);
        levelSound1 = false;
        levelSound2 = true;
      }

      if(levelPulse >= levelTalk){
        led_voice();
        levelTalk = levelPulse + (short_pause - 100);
      }
      
      ST->stop();
    }
    
    if(levelPulse > strongMagicPulse && levelPulse <= ambientMagicPulse){  // 2. ambient exploring
      if(levelSound2){
        MP3Trigger.trigger(10);
        levelSound2 = false;
        levelSound3 = true;
        //clearLights();
      }

      ST->drive(-40);
      ST->turn(16);
      
      ambientLights();
    }
    
    if(levelPulse > ambientMagicPulse && levelPulse <= powerUpPulse){  // 3. Power Up sound + wiggles
      if(levelSound3){
        ST->stop();
        MP3Trigger.trigger(6);
        levelSound3 = false;
        levelSound4 = true;
      }
      if(levelPulse > powerUpClimaxPulse) {
        climaxLights();
      } else {
        randomIncrement(); // lights
        ST->drive(0);
        if(levelPulse >= wigglePulse){
          wiggleTurn = -1 * wiggleTurn;
          ST->turn(wiggleTurn);
          wigglePulse += 400;
        }
      }
    }
    
    if(levelPulse > powerUpPulse && levelPulse <= fanfarePulse){  // 4. Fanfare + Victory Lap
      if(levelSound4){
        MP3Trigger.trigger(7);
        levelSound4 = false;
      }

      ST->turn(-100);
      ST->drive(100);
      fanfareLights();
    }

    if(levelPulse > fanfarePulse){
      leveling_up = false;
    }

    levelPulse = millis();
   }
}


void tavernFight() {
  if(fight){
      if(fightPulse <= wormPulse) {
        if(playWorm) {
          MP3Trigger.trigger(15);
          playPapi = true;
          playWorm = false;
          wormTalk = true;
        }

        ST->drive(-50);
        ST->turn(0);
      }

      if(wormTalk) {
        if(fightPulse >= talk_interval) {
          wormLights();
          //led_voice();
          talk_interval = fightPulse + short_pause;
        }
      }
      
      if(fightPulse <= papiPulse && fightPulse >= wormPulse){
        if(playPapi){
          MP3Trigger.trigger(12);
          playPapi = false;
          playRoll = true;
          wormTalk = false;
          papiTalk = true;
        }

        ST->turn(-35);
        ST->drive(-30);
      }

      if(papiTalk) {
        if(fightPulse >= talk_interval) {
          //led_voice();
          papiLights();
          talk_interval = fightPulse + short_pause;
        }
      }

      if(fightPulse <= rollPulse && fightPulse > papiPulse){
        if(playRoll){
          MP3Trigger.trigger(13);
          playRoll = false;
          playFightMusic = true;
          papiTalk = false;
        }
        randomIncrement();
      }

      if(fightPulse > rollPulse && fightPulse < dicePulse) {
        if(!diceDropped) {
          myServo.write(179);
          diceDropped = true;
        }
      }

      if(fightPulse <= fightMusicPulse && fightPulse > diceWaitPulse){
        long backUpPulse;
        long hit1Pulse;
        long hit2Pulse;
        long hit3Pulse;
        long moveForwardPulse;
        long backUp2Pulse;
        long hit4Pulse;
        long hit5Pulse;
        long hit6Pulse;
        long moveForward2Pulse;
        
        if(playFightMusic){
          backUpPulse = fightPulse + 1000;
          hit1Pulse = backUpPulse + 1000;
          hit2Pulse = hit1Pulse + 1000;
          hit3Pulse = hit2Pulse + 1000;
          moveForwardPulse = hit3Pulse + 1000;
          backUp2Pulse = moveForwardPulse + 1000;
          hit4Pulse = backUp2Pulse + 1000;
          hit5Pulse = hit4Pulse + 1000;
          hit6Pulse = hit5Pulse + 1000;
          moveForward2Pulse = hit6Pulse + 500;
          
          magicLights(0);
          MP3Trigger.trigger(14);
          playFightMusic = false;
          playPint = true;
          myServo.write(5);
        }

        if(fightPulse >= ambientTavernPulse) {
          ambientTavern();
          ambientTavernPulse = fightPulse + ambientTavernInterval;
        }
        if(fightPulse >= hitPulse) {
          wormLights();
          hitPulse = fightPulse + 425;
        }

        if(fightPulse <= backUpPulse){  // fight 1: quickly back up
          ST->turn(0);
          ST->drive(60);
        }

        if(fightPulse > backUpPulse && fightPulse <= hit1Pulse){  // fight 2: get hit 1
          ST->turn(50);
          ST->drive(0);
        }

        if(fightPulse > hit1Pulse && fightPulse <= hit2Pulse){
          ST->turn(-60);
          ST->drive(0);
        }

        if(fightPulse > hit2Pulse && fightPulse <= hit3Pulse){
          ST->turn(55);
          ST->drive(0);
        }

        if(fightPulse > hit3Pulse && fightPulse <= moveForwardPulse){
          ST->turn(0);
          ST->drive(-60);
        }

        if(fightPulse > moveForwardPulse && fightPulse <= backUp2Pulse){
          ST->turn(0);
          ST->drive(50);
        }

        if(fightPulse > backUp2Pulse && fightPulse <= hit4Pulse){
          ST->turn(70);
          ST->drive(0);
        }

        if(fightPulse > hit4Pulse && fightPulse <= hit5Pulse){
          ST->turn(-60);
          ST->drive(0);
        }

        if(fightPulse > hit5Pulse && fightPulse <= hit6Pulse){
          ST->turn(65);
          ST->drive(0);
        }

        if(fightPulse > hit6Pulse && fightPulse <= moveForward2Pulse){
          ST->turn(0);
          ST->drive(-50);
        }
      }

      if(fightPulse <= pintPulse && fightPulse > fightMusicPulse){
        if(playPint){
          MP3Trigger.trigger(11);
          playPint = false;
          papiTalk = true;
        }
      }

      if(fightPulse > pintPulse){
        fight = false;
      }

      fightPulse = millis();
   }
}

// =======================================================================================
//          Print Output Function
// =======================================================================================

void printOutput(const char *value)
{
    if ((strcmp(value, "") != 0))
    {
        if (Serial) Serial.println(value);
        strcpy(output, ""); // Reset output string
    }
}
