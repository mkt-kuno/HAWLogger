#include "oled1602.h"
#include "Wire.h"

#define OLED_Address 0x3c
#define OLED_Command_Mode 0x80
#define OLED_Data_Mode 0x40
#define ENABLE_REMAP
OLED1602::OLED1602(void) {

}

void OLED1602::sendCommand(unsigned char command) {
    Wire.beginTransmission(OLED_Address); 	 // **** Start I2C 
    Wire.write(OLED_Command_Mode);     		 // **** Set OLED Command mode
    Wire.write(command);
    Wire.endTransmission();                 	 // **** End I2C 
    delay(10);
}

void OLED1602::sendData(unsigned char data)
{
  Wire.beginTransmission(OLED_Address);  	// **** Start I2C 
  Wire.write(OLED_Data_Mode);     		// **** Set OLED Data mode
  Wire.write(data);
  Wire.endTransmission();                     // **** End I2C 
}

void OLED1602::begin()
{
    // *** I2C initial *** //
    delay(100);
    sendCommand(0x2A); // **** Set "RE"=1	00101010B
    sendCommand(0x71);
    sendCommand(0x5C);
    sendCommand(0x28);

    sendCommand(0x08); // **** Set Sleep Mode On
    sendCommand(0x2A); // **** Set "RE"=1	00101010B
    sendCommand(0x79); // **** Set "SD"=1	01111001B

    sendCommand(0xD5);
    sendCommand(0x70);
    sendCommand(0x78); // **** Set "SD"=0  01111000B

    sendCommand(0x08); // **** Set 5-dot, 3 or 4 line(0x09), 1 or 2 line(0x08)

    sendCommand(0x06); // **** Set Com31-->Com0  Seg0-->Seg99

    // **** Set OLED Characterization *** //
    sendCommand(0x2A); // **** Set "RE"=1
    sendCommand(0x79); // **** Set "SD"=1

    // **** CGROM/CGRAM Management *** //
    sendCommand(0x72); // **** Set ROM
    sendCommand(0x00); // **** Set ROM A and 8 CGRAM

    sendCommand(0xDA); // **** Set Seg Pins HW Config
    sendCommand(0x10);

    sendCommand(0x81); // **** Set Contrast
    sendCommand(0xFF);

    sendCommand(0xDB); // **** Set VCOM deselect level
    sendCommand(0x30); // **** VCC x 0.83

    sendCommand(0xDC); // **** Set gpio - turn EN for 15V generator on.
    sendCommand(0x03);

    sendCommand(0x78); // **** Exiting Set OLED Characterization
    sendCommand(0x28);
    sendCommand(0x2A);
    // sendCommand(0x05); 	// **** Set Entry Mode
    sendCommand(0x06); // **** Set Entry Mode
    sendCommand(0x08);
    sendCommand(0x28); // **** Set "IS"=0 , "RE" =0 //28
    sendCommand(0x01);
    sendCommand(0x80); // **** Set DDRAM Address to 0x80 (line 1 start)

    delay(100);
    sendCommand(0x0C); // **** Turn on Display
}

void OLED1602::clear(){
    sendCommand(0x01);
}

inline size_t OLED1602::write(uint8_t value) {
#ifdef ENABLE_REMAP
    //ROM A->B対策
    switch(value) {
        case '[':
            sendData(0b11111010);
            break;
        case ']':
            sendData(0b11111100);
            break;
        case '|':
            sendData(0b11111110);
            break;
        case '\\':
            sendData(0b11111011);
            break;
        case '_':
            sendData(0b11000100);
            break;
        default:
            sendData(value);
            break;
    }
#endif
    return 1;
}

inline size_t OLED1602::write(const uint8_t *buffer, size_t size) {
    for (size_t i = 0; i < size; i++) {
        write(buffer[i]);
    }
    return size;
}

void OLED1602::setCursor(uint8_t col, uint8_t row) {
    int row_offsets[] = { 0x00, 0x40 };
    sendCommand(0x80 | (col + row_offsets[row]));
}

void OLED1602::lcdOff(){
    sendCommand(0x08);
}
void OLED1602::lcdOn(){
    sendCommand(0x0C);
}

void OLED1602::setContrast(unsigned char contrast)
{
    //Set OLED Command set
	sendCommand(0x2A); 
	sendCommand(0x79); 
	
	sendCommand(0x81);  	// Set Contrast
	sendCommand(contrast);	// send contrast value
	sendCommand(0x78);  	// Exiting Set OLED Command set
    sendCommand(0x28);
}