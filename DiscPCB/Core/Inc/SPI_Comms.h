/*
 * SPI_Comms.h
 *
 *  Created on: Apr 5, 2025
 *      Author: chama
 */

#ifndef INC_SPI_COMMS_H_
#define INC_SPI_COMMS_H_

#include "stm32g4xx_hal.h"
#include "String.h"
#include "main.h"


#define PACKET_TYPE_MOVE			0x10 // Strelka sends an extension to reach
#define PACKET_TYPE_RETRACT_FULL 	0x20 // Strelka requests airbrakes fully closed
#define PACKET_TYPE_EXTEND_FULL 	0x30 // Strelka requests airbrakes fully open
#define PACKET_TYPE_DEVICE_STATUS	0x40 // Strelka requests status of the Disc
#define PACKET_TYPE_POWER			0x50 // Strelka requests battery voltage and current readings

#define PACKET_SIZE_DISC_RX			7
#define PACKET_SIZE_STRELKA_RX		13

// Define this buffer within main.c to be used in SPI Receive IT/DMA calls
extern uint8_t rxDiscSPI[8];
extern DeviceStatus_t discStatus;

typedef struct {
	uint8_t packetType;
// TODO: Add any other fields you need for your packet header
} PacketHeader_t;

/* Packet structures sent from Strelka*/
typedef struct __attribute__((packed)) {
	uint8_t header;
	uint32_t timestamp;
	uint16_t targetPosition;
} PacketMove_t;

typedef struct __attribute__((packed)) {
	uint8_t header;
	uint32_t timestamp;
} PacketRetractFull_t;

typedef struct __attribute__((packed)) {
	uint8_t header;
	uint32_t timestamp;
} PacketExtendFull_t;

/* Packet structures sent from the Disc */
typedef struct __attribute__((packed)) {
	uint8_t header;
	uint32_t timestamp;
	uint16_t currentPosition;
	uint16_t targetPosition;
	uint8_t isMoving;
} PacketDeviceStatus_t;

typedef struct __attribute__((packed)) {
	uint8_t header;
	uint32_t timestamp;
	uint32_t battV;
	uint32_t battI;
} PacketPower_t;

/* Decode and encode methods */
uint16_t decodeMovePacket(void);		// Used by Disc to work out target position
void decodeDeviceStatus(void);		// Used by Strelka to work out status of Disc
void decodePower(void);				// Used by Strelka to work out Disc power consumption

// Both used by Disc to create packets for Strelka
void encodeDeviceStatus(PacketDeviceStatus_t* packetPtr, uint32_t timestamp, uint16_t currentPosition, uint16_t targetPosition, uint8_t isMoving);
void encodePower(PacketPower_t* packetPtr, uint32_t timestamp, uint32_t battV, uint32_t battI);

// Both used by Strelka to create packets for Disc
void encodeRetractPacket(PacketRetractFull_t* packetPtr, uint32_t timestamp);
void encodeExtendPacket(PacketExtendFull_t* packetPtr, uint32_t timestamp);

#endif /* INC_SPI_COMMS_H_ */
