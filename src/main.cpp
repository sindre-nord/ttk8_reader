#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <CAN.h>

// Define NFC Pins
#define SS_PIN 5
#define RST_PIN 4

// Define CAN Pins
#define CAN_TX_PIN 22
#define CAN_RX_PIN 21

// CAN interrupt handler
volatile bool canMessageReceived = false;
void IRAM_ATTR onCANReceive(int messageSize) {
  canMessageReceived = true;  // Set flag when CAN message is received
}

// Enum for CAN commands
enum CANCommand {
    START = 0x101,
    STOP  = 0x102,
    ACK   = 0x103,
    UID   = 0x104,
};

// MFRC522 instance
MFRC522 mfrc522(SS_PIN, RST_PIN);

// Function to send CAN command without data
void sendCANCommand(CANCommand command) {
  CAN.beginPacket(command);  // The command is implied through CAN ID
  CAN.endPacket();
}

// Function to send a CAN message with a string, split into 8-byte packets if necessary
void sendCANMessage(uint32_t id, const char *message) {
    const uint8_t maxPayloadSize = 8;  // CAN frames can hold up to 8 bytes
    const char *ptr = message;         // Pointer to the current position in the message

    while (*ptr) {
        CAN.beginPacket(id);           // Start the CAN packet

        // Write up to 8 bytes to the CAN packet
        for (uint8_t i = 0; i < maxPayloadSize && *ptr; i++) {
            CAN.write(*ptr++);         // Write the current byte and move to the next
        }

        CAN.endPacket();               // End the current packet
    }
}

bool isLookingForCard = false;

void setup() {
  // Initialize serial communications
  Serial.begin(115200);
  while (!Serial);  // Wait for serial to initialize

  // Initialize CAN bus
  CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
  if (!CAN.begin(10000)) {
    Serial.println("Starting CAN failed!");
    while (1);  // Infinite loop if CAN fails to start
  }
  Serial.println("CAN initialized.");

  // Set CAN receive interrupt
  CAN.onReceive(onCANReceive);

  // Initialize SPI bus and NFC reader
  SPI.begin();
  mfrc522.PCD_Init();
  delay(20);  // Optional delay
  mfrc522.PCD_DumpVersionToSerial();
  Serial.println("Ready to receive CAN message.");
}

void processCANMessage(uint32_t receivedId) {
  switch (receivedId) {
    case START:
      Serial.println("START command received.");
      sendCANCommand(ACK);  // Send ACK
      Serial.println("ACK sent.");
      Serial.println("Looking for NFC sticker...");
      isLookingForCard = true;
      break;

    case STOP:
      Serial.println("STOP command received.");
      isLookingForCard = false;
      break;

    default:
      Serial.print("Unknown CAN command received: 0x");
      Serial.println(receivedId, HEX);
      break;
  }
}

void processNFCCard() {
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    Serial.println("NFC sticker detected!");

    // Read the UID and send it over CAN with ID 0x104 (UID)
    String uid = "";
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      uid += String(mfrc522.uid.uidByte[i], HEX);
    }

    // Send the UID string as a message with CAN ID 0x104
    char uidMessage[30];
    uid.toCharArray(uidMessage, sizeof(uidMessage));
    sendCANMessage(UID, uidMessage);
    
    Serial.print("Sent UID: ");
    Serial.println(uidMessage);

    mfrc522.PICC_HaltA();  // Halt the PICC to allow another read later
    isLookingForCard = false;  // Stop looking for a card after successful read
  }
}

void handleCANMessage() {
  if (canMessageReceived) {
    canMessageReceived = false;  // Reset the flag
    sendCANCommand(ACK);  // Send an ACK message

    if (CAN.available()) {
      uint32_t receivedId = CAN.packetId();
      processCANMessage(receivedId);  // Process the message
    }
  }
}

void loop() {
  // Handle received CAN messages
  handleCANMessage();

  // Handle NFC card processing if we are looking for a card
  if (isLookingForCard) {
    processNFCCard();
  }
}