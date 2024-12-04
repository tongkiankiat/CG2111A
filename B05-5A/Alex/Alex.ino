  #include <serialize.h>
  #include <stdarg.h>
  #include "packet.h"
  #include "constants.h"
  #include <math.h>

  #define TRIG_PIN_F 26
  #define ECHO_PIN_F 27
  #define TRIG_PIN_L 22
  #define ECHO_PIN_L 23
  //#define ALEX_LENGTH 26
  //#define ALEX_BREADTH 16
  #define S0 53
  #define S1 52
  #define S2 50
  #define S3 49
  #define sensorOut 47

  int sound_speed = 343;

  volatile TDirection dir;

  /*
    Alex's configuration constants
  */

  // Number of ticks per revolution from the
  // wheel encoder.

  //#define COUNTS_PER_REV 4

  // Wheel circumference in cm.
  // We will use this to calculate forward/backward distance traveled
  // by taking revs * WHEEL_CIRC

  //#define WHEEL_CIRC 22  // actual value is 21.9

  /*
  // Store the ticks from Alex's left and
  // right encoders.
  volatile unsigned long leftForwardTicks;
  volatile unsigned long rightForwardTicks;
  volatile unsigned long leftReverseTicks;
  volatile unsigned long rightReverseTicks;

  // Left and right encoder ticks for turning
  volatile unsigned long leftForwardTicksTurns;
  volatile unsigned long rightForwardTicksTurns;
  volatile unsigned long leftReverseTicksTurns;
  volatile unsigned long rightReverseTicksTurns;

  // Store the revolutions on Alex's left
  // and right wheels
  volatile unsigned long leftRevs;
  volatile unsigned long rightRevs;

  // Forward and backward distance traveled
  volatile unsigned long forwardDist;
  volatile unsigned long reverseDist;
  unsigned long deltaDist;
  unsigned long newDist;
  unsigned long deltaTicks;
  unsigned long targetTicks;

  unsigned long computeDeltaTicks(float ang) {
    unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
    return ticks;
  } */

  void left(float ang, float speed) {
    ccw(ang, speed);
  }

  void right(float ang, float speed) {
    cw(ang, speed);
  }

  void stopTimer(){
    TCCR5B = 0;
    TCNT5H = 0;
    TCNT5L = 0;
  }

  ISR(TIMER5_COMPA_vect) {
    //stop the timer and clear counters
    stopTimer();
    stop();
  }

  // Variables for colour detection
  int redPW = 0;
  int greenPW = 0;
  int bluePW = 0;
  bool detected = 0;

  // Function to read Red Pulse Widths
  int getRedPW() {
    // Set sensor to read Red only
    PORTB &= 0b11110011;
    delay(100);
    // Define integer to represent Pulse Width
    int PW;
    // Read the output Pulse Width
    PW = pulseIn(sensorOut, LOW);
    // Return the value
    return PW;
  }

  // Function to read Green Pulse Widths
  int getGreenPW() {
    // Set sensor to read Green only
    PORTB |= 0b00001100;
    delay(100);
    // Define integer to represent Pulse Width
    int PW;
    // Read the output Pulse Width
    PW = pulseIn(sensorOut, LOW);
    // Return the value
    return PW;
  }

  // Function to read Blue Pulse Widths
  int getBluePW() {
    // Set sensor to read Blue only
    PORTB &= 0b11111011;
    PORTB |= 0b00001000;
    delay(100);
    // Define integer to represent Pulse Width
    int PW;
    // Read the output Pulse Width
    PW = pulseIn(sensorOut, LOW);
    // Return the value
    return PW;
  }

  const char *detectColourdiff() { 
    //dprintf("detecting colour\n");
    if (redPW <= 170) {
      return "wht";
    } else if (redPW >= bluePW) {
      return "grn";
    } else if (redPW <= bluePW) {
      return "red";
    }
  }

  // Alex Speaker
  #define NOTE_C4 262
  #define NOTE_D4 294
  #define NOTE_E4 330
  #define NOTE_F4 349
  #define NOTE_G4 392
  #define NOTE_A4 440
  #define NOTE_B4 494
  #define NOTE_C5 523
  #define NOTE_G3 196
  #define SPEAKER_PIN 44

  int red_melody[] = {
    NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5,
    NOTE_B4, NOTE_A4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4
  };

  int green_melody[] = {
    NOTE_E4, NOTE_E4, NOTE_E4, NOTE_C4, NOTE_E4, NOTE_G4, NOTE_G3
  };

  int red_noteDurations[] = {
    4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4
  };

  int green_noteDurations[] = {
    8, 4, 4, 8, 4, 2, 2
  };

  void red_music() {
    for (int thisNote = 0; thisNote < 14; thisNote++) {
      int red_noteDuration = 1000 / red_noteDurations[thisNote];
      tone(SPEAKER_PIN, red_melody[thisNote], red_noteDuration);
      int pauseBetweenNotes = red_noteDuration * 1.30;
      delay(pauseBetweenNotes);
      // stop the tone playing:
      noTone(SPEAKER_PIN);
    }
  }

  void green_music() {
    for (int thisNote = 0; thisNote < 7; thisNote++) {
      int green_noteDuration = 800 / green_noteDurations[thisNote];
      tone(SPEAKER_PIN, green_melody[thisNote], green_noteDuration);
      int pauseBetweenNotes = green_noteDuration * 1.30;
      delay(pauseBetweenNotes);
      // stop the tone playing:
      noTone(SPEAKER_PIN);
    }
  }

  /*

    Alex Communication Routines.

  */

  TResult readPacket(TPacket *packet) {
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".

    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if (len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
  }

  /*
  void sendStatus() {
    // Implement code to send back a packet containing key
    // information like leftTicks, rightTicks, leftRevs, rightRevs
    // forwardDist and reverseDist
    // Use the params array to store this information, and set the
    // packetType and command files accordingly, then use sendResponse
    // to send out the packet. See sendMessage on how to use sendResponse.
    //
    uint32_t thing[16] = { (uint32_t)leftForwardTicks, (uint32_t)rightForwardTicks,
                          (uint32_t)leftReverseTicks, (uint32_t)rightReverseTicks,
                          (uint32_t)leftForwardTicksTurns, (uint32_t)rightForwardTicksTurns,
                          (uint32_t)leftReverseTicksTurns, (uint32_t)rightReverseTicksTurns, (uint32_t)forwardDist, (uint32_t)reverseDist };
    TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_STATUS;
    sendResponse(&statusPacket);
  }
  */

  void sendMessage(const char *message) {
    // Sends text messages back to the Pi. Useful
    // for debugging.

    TPacket messagePacket;
    messagePacket.packetType = PACKET_TYPE_MESSAGE;
    strncpy(messagePacket.data, message, MAX_STR_LEN);
    sendResponse(&messagePacket);
  }

  void dprintf(char *format, ...) {
    va_list args;
    char buffer[128];

    va_start(args, format);
    vsprintf(buffer, format, args);
    sendMessage(buffer);
  }

  void sendBadPacket() {
    // Tell the Pi that it sent us a packet with a bad
    // magic number.

    TPacket badPacket;
    badPacket.packetType = PACKET_TYPE_ERROR;
    badPacket.command = RESP_BAD_PACKET;
    sendResponse(&badPacket);
  }

  void sendBadChecksum() {
    // Tell the Pi that it sent us a packet with a bad
    // checksum.

    TPacket badChecksum;
    badChecksum.packetType = PACKET_TYPE_ERROR;
    badChecksum.command = RESP_BAD_CHECKSUM;
    sendResponse(&badChecksum);
  }

  void sendBadCommand() {
    // Tell the Pi that we don't understand its
    // command sent to us.

    TPacket badCommand;
    badCommand.packetType = PACKET_TYPE_ERROR;
    badCommand.command = RESP_BAD_COMMAND;
    sendResponse(&badCommand);
  }

  void sendBadResponse() {
    TPacket badResponse;
    badResponse.packetType = PACKET_TYPE_ERROR;
    badResponse.command = RESP_BAD_RESPONSE;
    sendResponse(&badResponse);
  }

  void sendOK() {
    TPacket okPacket;
    okPacket.packetType = PACKET_TYPE_RESPONSE;
    okPacket.command = RESP_OK;
    sendResponse(&okPacket);
  }

  void sendResponse(TPacket *packet) {
    // Takes a packet, serializes it then sends it out
    // over the serial port.
    char buffer[PACKET_SIZE];
    int len;

    len = serialize(buffer, packet, sizeof(TPacket));
    writeSerial(buffer, len);
  }


  /*
    Setup and start codes for external interrupts and
    pullup resistors.

  */
  /*
  // Enable pull up resistors on pins 18 and 19
  void enablePullups() {
    // Use bare-metal to enable the pull-up resistors on pins
    // 19 and 18. These are pins PD2 and PD3 respectively.
    // We set bits 2 and 3 in DDRD to 0 to make them inputs.
    DDRD &= 0b11110011;
    PORTD |= 0b00001100;
  }
  */
  /*
    Setup and start codes for the timer
  */

  void setupTimer() {
    TCCR5A = 0;
    OCR5AH = 0b01111010;
    OCR5AL = 0b00010010;
    TIMSK5 = 0b00000010;
  }

  void startTimer() {
    //clear counter to be sure
    TCNT5H = 0; 
    TCNT5L = 0;
    TCCR5B = 0b00001101;
  }

  /*
    Setup and start codes for serial communications

  */
  // Set up the serial connection. For now we are using
  // Arduino Wiring, you will replace this later
  // with bare-metal code.
  void setupSerial() {
    // To replace later with bare-metal.
    Serial.begin(9600);
    // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
  }

  // Start the serial connection. For now we are using
  // Arduino wiring and this function is empty. We will
  // replace this later with bare-metal code.

  void startSerial() {
    // Empty for now. To be replaced with bare-metal code
    // later on.
  }

  // Read the serial port. Returns the read character in
  // ch if available. Also returns TRUE if ch is valid.
  // This will be replaced later with bare-metal code.

  int readSerial(char *buffer) {

    int count = 0;

    // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

    while (Serial.available())
      buffer[count++] = Serial.read();

    return count;
  }

  // Write to the serial port. Replaced later with
  // bare-metal code

  void writeSerial(const char *buffer, int len) {
    Serial.write(buffer, len);
    // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
  }

  /*
    Alex's setup and run codes

  // Clears all our counters
  void clearCounters() {
    leftForwardTicks = 0;
    rightForwardTicks = 0;
    leftReverseTicks = 0;
    rightReverseTicks = 0;
    leftForwardTicksTurns = 0;
    rightForwardTicksTurns = 0;
    leftReverseTicksTurns = 0;
    rightReverseTicksTurns = 0;
    forwardDist = 0;
    reverseDist = 0;
  }

  // Clears one particular counter
  void clearOneCounter(int which) {
    clearCounters();
  }

  // Intialize Alex's internal states

  void initializeState() {
    clearCounters();
  } */

  void handleCommand(TPacket *command) {
    switch (command->command) {
      // For movement commands, param[0] = distance, param[1] = speed.
      case COMMAND_FORWARD:
        sendOK();
        forward(100, 50);
        break;

      case COMMAND_REVERSE:
        sendOK();
        backward(100, 50);
        break;

      case COMMAND_TURN_LEFT:
        sendOK();
        left(100, (double)command->params[1]);
        break;

      case COMMAND_TURN_RIGHT:
        sendOK();
        right(100, (double)command->params[1]);
        break;

      case COMMAND_STOP:
        sendOK();
        stop();
        break;

      case COMMAND_FORWARD_AUTO:
        sendOK();
        forward_auto(0, 50);
        break;

      case COMMAND_GET_COLOUR:
        sendOK();
        PORTC |= 0b00000001;
        // Set Pulse Width scaling to 20%
        PORTB |= 0b00000001;
        PORTB &= 0b11111101;
        while (!detected) {
          // Read Red Pulse Width
          redPW = getRedPW();
          // Delay to stabilize sensor
          delay(200);

          // Read Green Pulse Width
          greenPW = getGreenPW();
          // Delay to stabilize sensor
          delay(200);

          // Read Blue Pulse Width
          bluePW = getBluePW();
          // Delay to stabilize sensor
          delay(200);
          detected = true;
        }
        PORTC &= 0b11111110;
        PORTB &= 0b11111100;

        if (detectColourdiff() == "red") {
          sendMessage(detectColourdiff());
          sendMessage(String(redPW).c_str());
          sendMessage(String(bluePW).c_str());
          sendMessage(String(greenPW).c_str());
          red_music();
        } else if (detectColourdiff() == "wht") {
          sendMessage(detectColourdiff());
          sendMessage(String(redPW).c_str());
          sendMessage(String(bluePW).c_str());
          sendMessage(String(greenPW).c_str());
        } else if (detectColourdiff() == "grn") {
          sendMessage(detectColourdiff());
          sendMessage(String(redPW).c_str());
          sendMessage(String(bluePW).c_str());
          sendMessage(String(greenPW).c_str());
          green_music();
        }
        detected = false;
        break;

      default:
        sendBadCommand();
    }
  }

  void waitForHello() {
    int exit = 0;

    while (!exit) {
      TPacket hello;
      TResult result;

      do {
        result = readPacket(&hello);
      } while (result == PACKET_INCOMPLETE);

      if (result == PACKET_OK) {
        if (hello.packetType == PACKET_TYPE_HELLO) {
          sendOK();
          exit = 1;
        } else
          sendBadResponse();
      } else if (result == PACKET_BAD) {
        sendBadPacket();
      } else if (result == PACKET_CHECKSUM_BAD)
        sendBadChecksum();
    }
  }

  bool checkUS() {
    PORTA |= 0b00010000;
    delay(50);
    PORTA &= 0b11101111;
    if (pulseIn(ECHO_PIN_F, HIGH) * sound_speed / 20000 < 10) {
      return true;
    }
    PORTA |= 0b00000001;
    delay(50);
    PORTA &= 0b11111110;
    if (pulseIn(ECHO_PIN_L, HIGH) * sound_speed / 20000 < 10) {
      return true;
    }
    return false;
  }

  void setup() {
    // put your setup code here, to run once:

    cli();
    //setupEINT();
    //enablePullups();
    //initializeState();
    setupSerial();
    startSerial();
    setupTimer();
    sei();

    // Set TRIG_PIN_F and TRIG_PIN_L to OUTPUT
    DDRA |= 0b00010001;
    // Set ECHO_PIN_F and ECHO_PIN_L to INPUT
    DDRA &= 0b1101101;

    //alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
    //alexCirc = PI * alexDiagonal;

    // Colour Sensor Bare-Metal
    // Set Toggle pin for colour sensor
    DDRC |= 0b00000001;
    PORTC &= 0b11111110;

    // Set S0 - S3 as outputs
    DDRB |= 0b00001111;

    // Set Sensor output as input
    DDRL &= 0b111111011;
  }

  void handlePacket(TPacket *packet) {
    switch (packet->packetType) {
      case PACKET_TYPE_COMMAND:
        handleCommand(packet);
        break;

      case PACKET_TYPE_RESPONSE:
        break;

      case PACKET_TYPE_ERROR:
        break;

      case PACKET_TYPE_MESSAGE:
        break;

      case PACKET_TYPE_HELLO:
        break;
    }
  }

  void loop() {
      // put your main code here, to run repeatedly:
    TPacket recvPacket;

    TResult result = readPacket(&recvPacket);

    if (result == PACKET_OK)
      handlePacket(&recvPacket);

    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }

    else if (result == PACKET_CHECKSUM_BAD)
    {
      sendBadChecksum();
    }

    if (dir == FORWARD || dir == FORWARD_AUTO) {
      if (checkUS()) {
        stop();
        stopTimer();
        sendMessage("Obstacle");
      }
    }
  }