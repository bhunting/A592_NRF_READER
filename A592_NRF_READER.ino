/*
 Copyright (C) 2012 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 
 Update 2014 - TMRh20
 */

/**
 * Simplest possible example of using RF24Network 
 *
 * TRANSMITTER NODE
 * Every 2 seconds, send a payload to the receiver node.
 */

#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include "printf.h"

RF24 radio(9,10);                    // nRF24L01(+) radio attached using Getting Started board 

RF24Network network(radio);          // Network uses that radio

const uint16_t this_node = 00;        // Address of our node in Octal format
const uint16_t other_node = 01;       // Address of the other node in Octal format

const unsigned long interval = 5000; //ms  // How often to send 'hello world to the other unit

unsigned long last_sent;             // When did we last send?
unsigned long packets_sent;          // How many have we sent already


struct payload_t {                  // Structure of our payload
  unsigned long ms;
  unsigned long counter;
};

uint8_t statusArray[ 6 ];
 // formatted data pulled from raw sensor data
typedef struct sensorTemperatureData
{
    uint8_t     id;             // sensor id (1 - N_sensors)
    uint8_t     status;         // 0x80 = BATTERY LOW bit, 0x40 = Data Fresh bit, 
    uint16_t    temperature;    // temperature value in C, no offset
    uint32_t    timestamp;      // number of seconds since startup
} sensorTemperatureData;


void setup(void)
{
  Serial.begin(115200);
  printf_begin();
  Serial.println("nRF Network Sensor Hub");
 
  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_8);
  network.begin(/*channel*/ 90, /*node address*/ this_node);
}

static bool read_Status (RF24NetworkHeader& cmdHeader); 
static bool read_Data   (RF24NetworkHeader& cmdHeader);
static bool print_Error (RF24NetworkHeader& cmdHeader);
static uint8_t nextQuery = 'S';
static uint8_t sensorIdQuery = 0;
static bool sendStatus;
void loop() {
  
  network.update();                          // Check the network regularly

  while ( network.available() )              // Is there anything ready for us?
  {                
      //printf_P(PSTR("\n\r loopNRF network available \n\r"));
      RF24NetworkHeader header;                  // If so, take a look at it
      network.peek(header);
      switch (header.type)
      {                             // Dispatch the message to the correct handler.
          case 'S': read_Status(header); break;
          case 'D': read_Data(header); break;
          case 'E': print_Error(header); break;
          default:  printf_P(PSTR("*** WARNING *** Unknown message type %c\n\r"),header.type);
                    network.read(header,0,0);
                    break;
      };
  }  
  
  unsigned long now = millis();              // If it's time to send a message, send it!
  if ( now - last_sent >= interval  )
  {
    last_sent = now;

    Serial.println("");
    Serial.print("Sending...");
    RF24NetworkHeader header(/*to node*/ other_node);
    switch( nextQuery )
    {
      case 'S':
      // Get Status
      header.type = 'S';
      sendStatus = network.write(header,0,0);
      nextQuery = 'D';
      break;

      case 'D':
      sensorIdQuery++;
      if( sensorIdQuery > 6 ) sensorIdQuery = 1;
      header.type = 'D';
      sendStatus = network.write(header,&sensorIdQuery, sizeof(sensorIdQuery));
      nextQuery = 'S';
      break;

      default:
      nextQuery = 'S';
      break;
    }
    
    if (sendStatus)
      Serial.println("ok.");
    else
      Serial.println("failed.");
  }
}

/***********************************************************************/
/**
 * Send 'S' message, the current status
 */
bool read_Status(RF24NetworkHeader& cmdHeader)
{
    printf_P(PSTR("CMD: S : Received from 0%o\n\r"), cmdHeader.from_node);
    //RF24NetworkHeader rspHeader(/*to node*/ cmdHeader.from_node, /*type*/ 'S' /*Status*/);
    
    // The 'S' message is status of all sensor nodes
    uint8_t statusArray[ 6 ];
    network.read(cmdHeader, statusArray, sizeof(statusArray));

    Serial.print("Sensor Status: 1: ");
    Serial.print(statusArray[0]);
    Serial.print("\t2: ");
    Serial.print(statusArray[1]);
    Serial.print("\t3: ");
    Serial.print(statusArray[2]);
    Serial.print("\t4: ");
    Serial.print(statusArray[3]);
    Serial.print("\t5: ");
    Serial.print(statusArray[4]);
    Serial.print("\t6: ");
    Serial.print(statusArray[5]);
    Serial.println("");
    

//    for( int i = 0; i < 6; i++ )
//    {
//        Serial.print("Sensor Status ");
//        Serial.print(i);
//        Serial.print(" = ");
//        Serial.println(statusArray[i]);
//    }
}

/***********************************************************************/
/**
 * Send 'D' message, send data
 */
bool read_Data(RF24NetworkHeader& cmdHeader)
{
    printf_P(PSTR("CMD: D : Received from 0%o\n\r"), cmdHeader.from_node);
    //RF24NetworkHeader rspHeader(/*to node*/ cmdHeader.from_node, /*type*/ 'D' /*Sensor Data*/);
    
    // The 'D' message is data from a sensor node
    sensorTemperatureData sensorData;
    network.read(cmdHeader, &sensorData, sizeof(sensorTemperatureData));

    printf_P(PSTR("id: %u  status: 0x%2X  temperature: %d  timestamp: %d \n\r"), sensorData.id, sensorData.status, sensorData.temperature, sensorData.timestamp);

    //Serial.println(sensorData.id);
    //Serial.println(sensorData.status);
    //Serial.println(sensorData.temperature);
    //Serial.println(sensorData.timestamp);
}

/***********************************************************************/
/**
 * Print 'E' message
 */
bool print_Error(RF24NetworkHeader& cmdHeader)
{
    printf_P(PSTR("RSP: E : Received from 0%o\n\r"), cmdHeader.from_node);
    network.read(cmdHeader, 0, 0); // read to clear message
}
