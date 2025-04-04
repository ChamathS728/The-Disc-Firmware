/*
 * SPI_Comms.c
 *
 *  Created on: Apr 5, 2025
 *      Author: chama
 */
#include "SPI_Comms.h"

/* Decode and encode methods */
void decodeMovePacket(void) {
	// Used by Disc to work out target position
	static PacketMove_t packet;
	memcpy(&packet, rxDiscSPI, sizeof(packet));

	if (packet.header != PACKET_TYPE_MOVE) {
		// If header does not match
		return;
	}

	// TODO - Handle timestamp and position values

}


void decodeDeviceStatus(void);		// Used by Strelka to work out status of Disc
void decodePower(void);				// Used by Strelka to work out Disc power consumption

// Both used by Disc to create packets for Strelka
void encodeDeviceStatus(PacketDeviceStatus_t* packetPtr, uint32_t timestamp, uint32_t currentPosition, uint32_t targetPosition, uint8_t isMoving);
void encodePower(PacketPower_t* packetPtr, uint32_t timestamp, uint32_t battV, uint32_t battI);

// Both used by Strelka to create packets for Disc
void encodeRetractPacket(PacketRetractFull_t* packetPtr, uint32_t timestamp) {
	packetPtr->header = PACKET_TYPE_RETRACT_FULL;
	packetPtr->timestamp = timestamp;
}
void encodeExtendPacket(PacketExtendFull_t* packetPtr, uint32_t timestamp) {
	packetPtr->header = PACKET_TYPE_EXTEND_FULL;
	packetPtr->timestamp = timestamp;
}
