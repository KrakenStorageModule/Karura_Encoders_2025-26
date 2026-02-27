# Karura_Encoders_2025-26
By: Johan Kornet/ Karura US

Code for running multiple AMT20 on a single Arduino Uno through SPI

Configuration Guide:
// --- Configure your encoder CS pins here ---

const int NUM_ENCODERS = 1;  // Change to however many encoders you have

const int CS_PINS[Number Of Encoders] = {Encoder 1 Chip Select Pin, Encoder 2 Chip Select Pin, etc...};

uint16_t positions[Number of Encoders];  // Stores the latest position for each encoder


