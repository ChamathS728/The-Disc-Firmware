/*
 * SPI_Comms.c
 *
 *  Created on: Apr 5, 2025
 *      Author: chama
 */
#include "SPI_Comms.h"

// Status related functions
void requestDiscStatus(void) {
	/*
	 * Run by Master to Disc to request device status
	 * Master sends a Device Status Packet with nonsense attributes.
	 * 	Disc just needs the header to know that it needs to send status back
	 * */

	// Construct DEVICE_STATUS request buffer
	PacketDeviceStatus_t* packetPtr = (PacketDeviceStatus_t*) malloc(sizeof(PacketDeviceStatus_t));
	packetPtr->header = PACKET_TYPE_DEVICE_STATUS;

	memcpy(txStrelkaSPI, packetPtr, sizeof(*packetPtr));

	// Run a regular Transmit Receive - receive isn't important - timeout of 100ms
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(SPICommsHandle, txStrelkaSPI, rxStrelkaSPI, sizeof(txStrelkaSPI));
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	// Now that the Disc has been notified that status is requested, run transmit receive in IT mode
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(SPICommsHandle, txStrelkaSPI, rxStrelkaSPI, sizeof(txStrelkaSPI));
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	// NOTE - HAL_SPI_TxRxCpltCallback on Master should handle the received status data

	// Now that the transmission is done, free the memory allocated for that tx packet
	free(packetPtr);
}

// Movement related functions
void transmitTargetPosition(uint32_t timestamp, uint16_t position) {
	/*
	 * Run by Master to tell what position Disc should move to
	 *
	 * NOTE - User should use this function and requestDiscStatus to tell when Disc is finished moving
	 * */

	// Construct MOVE buffer
	PacketMove_t packet = {
			.header = PACKET_TYPE_MOVE,
			.targetPosition = position,
			.timestamp = timestamp
	};
	memcpy(txStrelkaSPI, &packet, sizeof(packet));

	// Transmit the move packet. It shouldn't receive anything meaningful
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(SPICommsHandle, txStrelkaSPI, rxStrelkaSPI, sizeof(txStrelkaSPI));
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	// Now transmit the move packet again, since the Disc should've recognised the move packet
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(SPICommsHandle, txStrelkaSPI, rxStrelkaSPI, sizeof(txStrelkaSPI));
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	// Quick delay to give Disc time to process command
	osDelay(2);

}


//void exampleDisc_HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
//	/*
//	 * Example of an SPI TxRx callback that can be used on the Disc
//	 * */
//
//	if (hspi == SPICommsHandle) {
//		// Cast packet to a header packet to only access the first byte
//		PacketHeader_t* packetHeader = (PacketHeader_t*) rxDiscSPI;
//
//		switch (packetHeader->header) {
//			case PACKET_TYPE_MOVE:
//				// Decode the entire packet this time
//				PacketMove_t* packet = (PacketMove_t*) rxDiscSPI;
//				discStatus.targetPosition = (float) packet->targetPosition/65535;
//
//				// Prepare the txBuffer on the Disc to have device status
//				PacketDeviceStatus_t packet = {
//						.header = PACKET_TYPE_DEVICE_STATUS,
//						.currentPosition = discStatus.currentPosition,
//						.targetPosition = discStatus.targetPosition,
//						.isMoving = discStatus.isMoving,
//						.timestamp = timestamp
//				};
//				memcpy(&packet, txDiscSPI, sizeof(packet));
//				break;
//			case PACKET_TYPE_DEVICE_STATUS:
//				// Prepare the txBuffer on the Disc to have device status
//				PacketDeviceStatus_t packet = {
//						.header = PACKET_TYPE_DEVICE_STATUS,
//						.currentPosition = discStatus.currentPosition,
//						.targetPosition = discStatus.targetPosition,
//						.isMoving = discStatus.isMoving,
//						.timestamp = timestamp
//				};
//				memcpy(&packet, txDiscSPI, sizeof(packet));
//				break;
//		}
//	}
//}
//
