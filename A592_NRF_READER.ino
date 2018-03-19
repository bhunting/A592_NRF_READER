/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example: Network topology, and pinging across a tree/mesh network
 *
 * Using this sketch, each node will send a ping to every other node in the network every few seconds. 
 * The RF24Network library will route the message across the mesh to the correct node.
 *
 * This sketch is greatly complicated by the fact that at startup time, each
 * node (including the base) has no clue what nodes are alive.  So,
 * each node builds an array of nodes it has heard about.  The base
 * periodically sends out its whole known list of nodes to everyone.
 *
 * To see the underlying frames being relayed, compile RF24Network with
 * #define SERIAL_DEBUG.
 *
 * Update: The logical node address of each node is set below, and are grouped in twos for demonstration.
 * Number 0 is the master node. Numbers 1-2 represent the 2nd layer in the tree (02,05).
 * Number 3 (012) is the first child of number 1 (02). Number 4 (015) is the first child of number 2.
 * Below that are children 5 (022) and 6 (025), and so on as shown below 
 * The tree below represents the possible network topology with the addresses defined lower down
 *
 *     Addresses/Topology                            Node Numbers  (To simplify address assignment in this demonstration)
 *             00                  - Master Node         ( 0 )
 *           02  05                - 1st Level children ( 1,2 )
 *    32 22 12    15 25 35 45    - 2nd Level children (7,5,3-4,6,8)
 *
 * eg:
 * For node 4 (Address 015) to contact node 1 (address 02), it will send through node 2 (address 05) which relays the payload
 * through the master (00), which sends it through to node 1 (02). This seems complicated, however, node 4 (015) can be a very
 * long way away from node 1 (02), with node 2 (05) bridging the gap between it and the master node.
 *
 * To use the sketch, upload it to two or more units and set the NODE_ADDRESS below. If configuring only a few
 * units, set the addresses to 0,1,3,5... to configure all nodes as children to each other. If using many nodes,
 * it is easiest just to increment the NODE_ADDRESS by 1 as the sketch is uploaded to each device.
 */

#include <avr/pgmspace.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include "printf.h"

/***********************************************************************
************* Set the Node Address *************************************
/***********************************************************************/

// These are the Octal addresses that will be assigned
const uint16_t node_address_set[10] = { 00, 02, 05, 012, 015, 022, 025, 032, 035, 045 };
 
// 0 = Master
// 1-2 (02,05)   = Children of Master(00)
// 3,5 (012,022) = Children of (02)
// 4,6 (015,025) = Children of (05)
// 7   (032)     = Child of (02)
// 8,9 (035,045) = Children of (05)

uint8_t NODE_ADDRESS = 0;  // Use numbers 0 through to select an address from the array

/***********************************************************************/
/***********************************************************************/


RF24 radio(9,10);                              // CE & CS pins to use (Using 7,8 on Uno,Nano)
RF24Network network(radio); 

uint16_t this_node;                           // Our node address

const unsigned long interval = 1000; // ms       // Delay manager to send pings regularly.
unsigned long last_time_sent;


const short max_active_nodes = 10;            // Array of nodes we are aware of
uint16_t active_nodes[max_active_nodes];
short num_active_nodes = 0;
short next_ping_node_index = 0;


bool send_S(uint16_t to);                      // Prototypes for functions to send & handle messages
bool send_D(uint16_t to);
void handle_S(RF24NetworkHeader& header);
void handle_D(RF24NetworkHeader& header);

typedef struct sensorTemperatureData
{
    uint8_t     id;             // sensor id (1 - N_sensors)
    uint8_t     status;         // 0x80 = BATTERY LOW bit, 0x40 = Data Fresh bit, 
    uint16_t    temperature;    // temperature value in C, no offset
    uint32_t    timestamp;      // number of seconds since startup
} sensorTemperatureData;

static sensorTemperatureData sensorData[6]; // I have 6 temperature sensors

void setup()
{
  Serial.begin(115200);
  printf_begin();
  printf_P(PSTR("\n\r A00592TX READER \n\r"));

  this_node = node_address_set[NODE_ADDRESS];            // Which node are we?
  
  SPI.begin();                                           // Bring up the RF network
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  network.begin(/*channel*/ 100, /*node address*/ this_node );

}

static uint8_t sendType = 0;
void loop()
{    
  network.update();                                      // Pump the network regularly

   while ( network.available() )  
   {                                                     // Is there anything ready for us?     
        RF24NetworkHeader header;                            // If so, take a look at it
        network.peek(header);
        Serial.println( "\n\r---------------------------------");
        Serial.println( header.toString() );
        switch (header.type)
        {                              // Dispatch the message to the correct handler.
            case 'S': handle_S(header); break;
            case 'D': handle_D(header); break;
            default:  printf_P(PSTR("*** WARNING *** Unknown message type %c\n\r"),header.type);
                      network.read(header,0,0);
                      break;
        };
    }

    unsigned long now = millis();                         // Send a ping every 'interval' ms
    if ( now > (last_time_sent + interval) )
    {
        last_time_sent = now;
        uint16_t to = 01;                                   // Who should we send to? By default, send to base

        bool ok;
        sendType++;
        if ( sendType & 0x01 )
        {
            ok = send_S(to);   
        }
        else
        {
            ok = send_D(to);
        }

        if (ok)
        {                                              // Notify us of the result
            printf_P(PSTR("%lu: APP Send ok\n\r"),millis());
        }
        else
        {
            printf_P(PSTR("%lu: APP Send failed\n\r"),millis());
        }
    }
}

/**
 * 
 */
bool send_S(uint16_t to)
{
    RF24NetworkHeader header(/*to node*/ to, /*type*/ 'S' /*Status*/);

    // The 'S' message 
    printf_P(PSTR("\n\r---------------------------------\n\r"));
    printf_P(PSTR("Sending %c to 0%o...\n\r"), 'S', to);
    return network.write(header, 0, 0);
}

/**
 * 
 */
bool send_D(uint16_t to)
{
    static uint8_t nodeId = 1; // which node id to ask for
    
    RF24NetworkHeader header(/*to node*/ to, /*type*/ 'D' /*Data*/);

    // The 'D' message 
    uint8_t cmd = nodeId;
    printf_P(PSTR("\n\r---------------------------------\n\r"));
    printf_P(PSTR("Sending %c id %u to 0%o...\n\r"), 'D', nodeId, to);
    bool rsp = network.write(header,&cmd,sizeof(cmd));
    
    if( (nodeId++) > 6 )
    {
        nodeId = 1;
    }
    
    return rsp;
}

/**
 * Receive and display the sensor status bytes
 * 
 */
void handle_S(RF24NetworkHeader& header)
{
    // The 'S' message is status of all sensor nodes
    uint8_t statusArray[ 6 ]; // I have 6 temperature sensors in my house
    int numread = network.read(header,&statusArray,sizeof(statusArray));
    
    //for( int i = 0; i < 6; i++ )
    //{
    //    printf_P(PSTR("Received %02X from 0%o\n\r"), statusArray[i],header.from_node);
    //}
    
    printf("STATUS: ");
    printf("%02X ", statusArray[0] );
    printf("%02X ", statusArray[1] );
    printf("%02X ", statusArray[2] );
    printf("%02X ", statusArray[3] );
    printf("%02X ", statusArray[4] );
    printf("%02X ", statusArray[5] );
    printf("\n\r");
}

/**
 * Receive and display a sensor reading
 */
void handle_D(RF24NetworkHeader& header)
{
    // The 'D' message is data
    sensorTemperatureData sensorData;  
    int numread = network.read(header,&sensorData,sizeof(sensorData));
    
    Serial.print("id = ");
    Serial.print(sensorData.id);
    Serial.print(", status = ");
    Serial.print(sensorData.status, HEX);
    Serial.print(", temperature = ");
    Serial.print(sensorData.temperature);
    Serial.print(", time = ");
    Serial.println(sensorData.timestamp);
}



