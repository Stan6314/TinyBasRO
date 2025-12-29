/********************************** (C) COPYRIGHT *******************************
 * File Purpose       : TinyBasicRO main file
 * Author             : Stan6314
 * Version            : V1.0
 * Date               : 2025/12/25
 * Description        : TinyBasic for CH32V003
 *      + character constants are in charstable.h
 * The program contains 3 parts:
 *      -> initialization, interrupt and auxiliary functions
 *      -> Tiny BASIC commands
 *      -> main loop
*********************************************************************************
*******************************************************************************/
#include "ch32fun.h"
#include "charstable.h"

// Uncomment for OLED display upside down
//#define OLEDUPSIDEDOWN

// I2C Timeout maximum count
#define TIMEOUT_MAX 500000 

// Global variables for display
volatile uint8_t VideoRAM[336];    // VideoRAM for 16 * 21 chars

// Global variables for Tiny BASIC
volatile uint32_t comENDTimer = 0;   // Internal timer for END command
volatile uint32_t basTimer = 0;      // Internal timer for Tiny BASIC TIME command incremented with "vertical sync"
static uint16_t dispPointer = 0;     // Pointer to current display position
volatile uint8_t RunMode = 0;     // If it is 1, then basic program is in run
uint8_t currentLine=0;    // Actual line to proccess
uint8_t noOfMemory = 0;   // Number of section in program memory or "file" (4 KB sections)
uint8_t compFlags = 0;    // Bit7 = Is DISK1, Bit6 = Is PCF857x, 
// Bit5 = Is OLED, Bit4 = Is CardKB, Bits3-0 = Low Addres of EEPROM (Disk adr)
uint8_t linePointer=0;    // Pointer to line memory buffer
uint8_t line_buffer[33];  // buffer for data read/write
uint8_t stack_buffer[8];  // buffer for GOSUB stack
uint8_t stackPointer=0;    // Pointer to GOSUB buffer
int charVariable[26]; // Variables A..Z
int arrayVariable[100]; // Variables in array @0..99
int evalNumber;     // Value of number for evaluation in BASIC line

// Define devices on I2C bus in compFlags
#define ISDISKf (compFlags & 0x80)
#define ISEXPANDER (compFlags & 0x40)
#define ISOLED (compFlags & 0x20)
#define ISKEYB (compFlags & 0x10)

// ****************************************************************
//         Systick interrupt handler
// ****************************************************************
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
	// Increment the Compare Register for the next trigger
	SysTick->CMP += 1504;
	// Clear the trigger state for the next IRQ
	SysTick->SR = 0x00000000;
	// Increment BASIC counter and END command counter
	basTimer++; comENDTimer++;
}

// ****************************************************************
//         Delay functions
// ****************************************************************
// Own delay functions with SysTick counter
void My_Delay_Us(uint32_t delayTime)
{
    uint32_t recentTime = SysTick->CNT;
    while((SysTick->CNT - recentTime) < (48 * delayTime)) {;}
}
void My_Delay_Ms(uint32_t delayTime)
{
    for(uint32_t i=0; i<delayTime; i++) My_Delay_Us(1000);
}

// ****************************************************************
//         I2C module functions
// ****************************************************************
// Initialization of I2C module
void IIC_Init()
{
	// Enable GPIOC and I2C
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1; 
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO;
    // Reset I2C1 to init all regs
	// PC1 is SDA, 30MHz Output, alt func, open-drain
	GPIOC->CFGLR &= ~(0xf<<(4*1));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*1);
	// PC2 is SCL, 30MHz Output, alt func, open-drain
	GPIOC->CFGLR &= ~(0xf<<(4*2));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*2);
    // set freq
	I2C1->CTLR2 = (FUNCONF_SYSTEM_CORE_CLOCK/2000000)&I2C_CTLR2_FREQ;;
	// Set clock config
    I2C1->CKCFGR = 0xC005;      // Speed up I2C to slightly bellow 400000 kHz
	// Enable I2C
	I2C1->CTLR1 |= I2C_CTLR1_PE;
	// set ACK mode
	I2C1->CTLR1 |= I2C_CTLR1_ACK;
}

// Wait for executing or timeout on I2C
int I2CTimeout(uint32_t EventCode)
{
    uint32_t status, timeout;
	timeout = TIMEOUT_MAX;
    do {
        status = I2C1->STAR1 | (I2C1->STAR2<<16);
        status &= EventCode;       
    }
	while((!(status == EventCode)) && (timeout--));
	if(timeout==-1) { I2C1->CTLR1 |= I2C_CTLR1_STOP; return 1; }
    return 0;
}

// Check I2C address for connected device
int CheckI2CDevice(u8 i2caddr)
{
    uint32_t timeout = TIMEOUT_MAX;
	// Wait for not busy
	while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
	if(timeout==-1) return 0;
	// Set START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;
	// Wait for master mode select
    if(I2CTimeout(I2C_EVENT_MASTER_MODE_SELECT)) return 0;
	// Send 7-bit address + write flag
	I2C1->DATAR = i2caddr;
	// Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {I2C1->CTLR1 |= I2C_CTLR1_STOP; return 0;};
	// set STOP condition
	I2C1->CTLR1 |= I2C_CTLR1_STOP;
    return 1;   // Yes, device is connected
}

// Check I2C for connected CardKB
int CheckI2CKeyboard()
{
    uint32_t timeout = TIMEOUT_MAX;
	// Wait for not busy
    while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
    if(timeout==-1) return 0;
    // Set START condition
    I2C1->CTLR1 |= I2C_CTLR1_START;
    // Wait for master mode select
    if(I2CTimeout(I2C_EVENT_MASTER_MODE_SELECT)) return 0;
    I2C1->DATAR = 0xBF;    // Send keyboard address + read
    // Wait for receive condition
    if(I2CTimeout(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {I2C1->CTLR1 |= I2C_CTLR1_STOP; return 0;};
    if(I2CTimeout(I2C_EVENT_MASTER_BYTE_RECEIVED)) return 0;   // Wait for ready read data
    // set STOP condition
    I2C1->CTLR1 |= I2C_CTLR1_STOP;
    return 1;   // Keyboard is connected
}

// ****************************************************************
//         OLED functions
// ****************************************************************
// Send 1 byte to OLED
void OLEDi2cTransfer(uint8_t data, uint8_t control)
{
    if(!ISOLED) return;
	// Wait for I2C not busy
	uint32_t timeout = TIMEOUT_MAX;
	while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
	if(timeout==-1) {I2C1->CTLR1 |= I2C_CTLR1_STOP; return;}
	// Set START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;
	// Wait for master mode select
    if(I2CTimeout(I2C_EVENT_MASTER_MODE_SELECT)) return;
	// send 7-bit address + write flag
	I2C1->DATAR = 0x78;
	// Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return;
	// Send LSB
	I2C1->DATAR = control;
	// Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
	// Send data
	I2C1->DATAR = data;
	// Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
	// set STOP condition
	I2C1->CTLR1 |= I2C_CTLR1_STOP;
    return;   // Yes, device is connected
}

// Send more (the same) data bytes to OLED
void OLEDi2cBlockTransfer(uint8_t data, uint16_t len)
{
    if(!ISOLED) return;
	// Wait for I2C not busy
	uint32_t timeout = TIMEOUT_MAX;
	while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
	if(timeout==-1) {I2C1->CTLR1 |= I2C_CTLR1_STOP; return;}
	// Set START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;
	// Wait for master mode select
    if(I2CTimeout(I2C_EVENT_MASTER_MODE_SELECT)) return;
	// send 7-bit address + write flag
	I2C1->DATAR = 0x78;
	// Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return;
	// Send LSB
	I2C1->DATAR = 0x40;
	// Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
    for (uint16_t i = 0; i < len; ++i)
    {
  	    // Send data
  	    I2C1->DATAR = data;
     	// Wait for transmit condition
        if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
    }
	// set STOP condition
	I2C1->CTLR1 |= I2C_CTLR1_STOP;
}

// Send to the display a command byte
void command(uint8_t sbyte)
{
    OLEDi2cTransfer(sbyte, 0x00);
}

//Send to the display a data byte
void OLEDdata(uint8_t sbyte)
{
    OLEDi2cTransfer(sbyte, 0x40);
}

// Initialisation of OLED
void OLEDinit()
{
    // Commands from unknown web page (I dont remember what)
    command(0xAE); command(0x00); command(0x10);
    command(0x20); command(0x81); command(0x2F);
#ifdef OLEDUPSIDEDOWN
    command(0xA1);
#else
    command(0xA0);
#endif  // For upside down display 0xA0 -> 0xA1
    command(0xA8); command(0x7F); command(0xA4);
    command(0xA6); command(0xD3); command(0x00); command(0xB0);
#ifdef OLEDUPSIDEDOWN
    command(0xC8);
#else
    command(0xC0);
#endif  // For upside down display 0xC0 -> 0xC8
    command(0xDA); command(0x12); command(0xD5);
    command(0x50); command(0xD9); command(0xF1);
    command(0x8D); command(0x14); command(0xDB);
    command(0x40); command(0xDC); command(0x00);
    command(0xAD); command(0x8A); command(0xAF);
    My_Delay_Ms(100);
}

//Set page address.
void OLEDsetPageAddress(uint8_t add) { command(0xB0 | add); }

//Set column address.
void OLEDsetColumnAddress(uint8_t add)
{
   command(0x10 | (add >> 4));  // Send lower address byte
   command((0x0F & add));       // Send upper address byte
}

// Fill screen with pattern
void OLEDclear(uint8_t c)
{
    for (uint8_t row = 0; row < 16; row++)
    {
        OLEDsetPageAddress(row);
        OLEDsetColumnAddress(0);
        OLEDi2cBlockTransfer(c, 0x80);
    }
}

// Draw character on the actual position on OLED
void PlotChar(uint8_t dispChar)
{
    if(!ISOLED) return;
    OLEDsetPageAddress(dispPointer/21);
    OLEDsetColumnAddress((dispPointer%21)*6);
    int toTabPoint = ((dispChar-0x20)&0x7F)*6;
	// Wait for I2C not busy
	uint32_t timeout = TIMEOUT_MAX;
	while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
	if(timeout==-1) {I2C1->CTLR1 |= I2C_CTLR1_STOP; return;}
	// Set START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;
	// Wait for master mode select
    if(I2CTimeout(I2C_EVENT_MASTER_MODE_SELECT)) return;
	// send 7-bit address + write flag
	I2C1->DATAR = 0x78;
	// Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return;
	// Send LSB
	I2C1->DATAR = 0x40;
	// Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
    for (uint16_t i = 0; i < 6; ++i)
    {
  	    // Send data
  	    I2C1->DATAR = CharTable[toTabPoint++];
     	// Wait for transmit condition
        if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
    }
	// set STOP condition
	I2C1->CTLR1 |= I2C_CTLR1_STOP;
}

// Move display up and make a new line
void MakeNewLine()
{
    int i;
    for(i=0;i<315;i++) VideoRAM[i]=VideoRAM[i+21];  // Roll it up
    for(i=315;i<336;i++) VideoRAM[i]=0; // Clear last line
    dispPointer = 315;  // Cursor to begin on the last line
    if(!ISOLED) return;
    for(i=0;i<16;i++) {
        OLEDsetPageAddress(i);
        OLEDsetColumnAddress(0);
    	// Wait for I2C not busy
    	uint32_t timeout = TIMEOUT_MAX;
    	while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
    	if(timeout==-1) {I2C1->CTLR1 |= I2C_CTLR1_STOP; return;}
    	// Set START condition
    	I2C1->CTLR1 |= I2C_CTLR1_START;
    	// Wait for master mode select
        if(I2CTimeout(I2C_EVENT_MASTER_MODE_SELECT)) return;
    	// send 7-bit address + write flag
    	I2C1->DATAR = 0x78;
    	// Wait for transmit condition
        if(I2CTimeout(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return;
    	// Send LSB
    	I2C1->DATAR = 0x40;
    	// Wait for transmit condition
        if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
        for(int j=0;j<21;j++) {
            int toTabPoint = 6 * VideoRAM[i*21+j];
            for (int k = 0; k < 6; ++k)
             {
            	// Send data
            	I2C1->DATAR = CharTable[toTabPoint++];
               	// Wait for transmit condition
                if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
             }
        }
	    // set STOP condition
	    I2C1->CTLR1 |= I2C_CTLR1_STOP;
    }
}

// ****************************************************************
//         Keyboard functions
// ****************************************************************
volatile uint8_t KbdDataBuffer = 0;       // Keyboard buffer

// Read keyboard status directly on I2C bus
uint8_t readActKbd()
{
    uint8_t actKey = 0;
    if(ISKEYB) {
        uint32_t timeout = TIMEOUT_MAX;
        while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
        if(timeout==-1) {I2C1->CTLR1 |= I2C_CTLR1_STOP; return actKey;}
        // Set START condition
        I2C1->CTLR1 |= I2C_CTLR1_START;
        // Wait for master mode select
        if(I2CTimeout(I2C_EVENT_MASTER_MODE_SELECT)) return actKey;
        I2C1->DATAR = 0xBF;    // Send keyboard address + read
        // Wait for receive condition
        if(I2CTimeout(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return actKey;
        if(I2CTimeout(I2C_EVENT_MASTER_BYTE_RECEIVED)) return actKey;   // Wait for ready read data
        actKey = I2C1->DATAR;
        // set STOP condition
        I2C1->CTLR1 |= I2C_CTLR1_STOP;
    }
    return actKey;
}

// Read keyboard status with check of buffer first and translate KEYWORD
uint8_t kbdRead() {
    uint8_t RetKey = 0;
    // If byte in buffer, return it
    if(KbdDataBuffer) {
        RetKey = KbdDataBuffer;
        KbdDataBuffer = 0;
    } else RetKey = readActKbd();
    // Enter is 0x0D - change it to 0x0A
    if(RetKey == 0x0D) RetKey = 0x0A;
    if(RetKey > 0x7F) {
      // Fn changes character to KEYWORD code
      switch (RetKey) {
        case 0x96: RetKey = 0xA0; break; // PRINT
        case 0x93: RetKey = 0xA1; break; // IF
        case 0x91: RetKey = 0xA2; break; // THEN
        case 0x9E: RetKey = 0xA3; break; // GOTO
        case 0x94: RetKey = 0xA4; break; // INPUT
        case 0xA2: RetKey = 0xA5; break; // LET
        case 0x9F: RetKey = 0xA6; break; // GOSUB
        case 0x92: RetKey = 0xA7; break; // RETURN
        case 0xA7: RetKey = 0xA8; break; // CLEAR
        case 0xA1: RetKey = 0xA9; break; // LIST
        case 0x90: RetKey = 0xAA; break; // RUN
        case 0xAB: RunMode = 0; break; // END is the same code with special function for stop program

        case 0xA9: RetKey = 0xAC; break; // TIME
        case 0x8E: RetKey = 0xAD; break; // CURSOR
        case 0xA8: RetKey = 0xAE; break; // PUTCH
        case 0xAD: RetKey = 0xAF; break; // GETCH
        case 0x9B: RetKey = 0xB0; break; // INKEY
        case 0x9D: RetKey = 0xB1; break; // FILE
        case 0x9C: RetKey = 0xB2; break; // DISK
        case 0xAC: RetKey = 0xB3; break; // DELAY
        case 0xA6: RetKey = 0xB4; break; // BEEP
        case 0x8F: RetKey = 0xB5; break; // REM

        case 0x9A: RetKey = 0xB6; break; // AINP
        case 0xAA: RetKey = 0xB7; break; // DINP
        case 0x95: RetKey = 0xB8; break; // DOUT
        case 0x8D: RetKey = 0xB9; break; // I2CW
        case 0xA0: RetKey = 0xBA; break; // I2CR
        case 0xAE: RetKey = 0xBB; break; // COPY
        // Arrow keys
        case 0xB5: RetKey = 0x38; break; // up
        case 0xB4: RetKey = 0x34; break; // left
        case 0xB7: RetKey = 0x36; break; // right
        case 0xB6: RetKey = 0x32; break; // down
        default: RetKey = 0x00;
      }
    }
    return RetKey;
}

// Wait for keystroke and blink cursor on the corresponding position
uint8_t GetKeyChar()
{
    uint8_t saveChar, retChar;
    saveChar = 0x20 + VideoRAM[dispPointer];  // Save char on the cursor position (for cursor blink)
    do{
        if(basTimer & 0x2000) PlotChar(saveChar); else PlotChar(0x80);  // blink
        retChar = kbdRead();
        My_Delay_Ms(100);
    } while(retChar == 0);
    PlotChar(saveChar);  // Restore position
    return retChar;
}

// ****************************************************************
//         Text functions for OLED
// ****************************************************************
// Display character 0x00 - 0xFF to OLED on cursor position
void WriteChar(uint8_t dispChar)
{
    switch (dispChar) {
        case 0x0C: {      // FormFeed
            for(int i=0;i<336;i++) VideoRAM[i]=0; 
            dispPointer = 0;
            OLEDclear(0x00);
        } break;
        case 0x0A: {    // Enter / New line
            dispPointer = ((dispPointer / 21) + 1) * 21;
        } break;
        case 0x09: {    // Tab
            int CurYPos = dispPointer / 21;     // Which line
            int CurXPos = dispPointer % 21;     // Which position on line
            if(CurXPos < 8) CurXPos = 8;
            else if(CurXPos < 16) CurXPos = 16;
                else{CurXPos = 0; CurYPos++;}
            dispPointer = CurYPos * 21 + CurXPos;
        } break;
        default: {  // Remaining characters
            if((dispChar>0x1F) && (dispChar<0xA0)) {
                PlotChar(dispChar);
                VideoRAM[dispPointer++] = dispChar-0x20;
            } // Printable
            else if((dispChar>0x9F) && (dispChar<0xBC)) { // Tiny BASIC Keywords
              int pKeyWord = ((dispChar - (uint8_t)0xA0)) << 3;
              for(int j=0; j<KeyWordTab[pKeyWord];j++){
                PlotChar(KeyWordTab[pKeyWord+j+1]);
                VideoRAM[dispPointer++] = KeyWordTab[pKeyWord+j+1] - 0x20;
                // Test pointer and roll display if it is over last line
                if(dispPointer>335) MakeNewLine();
              }
            }
            else if(dispChar!=0x08) VideoRAM[dispPointer++] = 0x0E; // Dot for non printable - except backspace
        }; break;
    }
    // Test pointer and roll display if it is over last line
    if(dispPointer>335) MakeNewLine();
}

// Print text (and second version with new line)
void WriteText(char *pointChar)
{
    while(*pointChar) WriteChar(*pointChar++);
}
void WriteTextln(const char *pointChar)
{
    while(*pointChar) WriteChar(*pointChar++);
    WriteChar(0x0A);
}
// Display integer number (32bits) on OLED
void WriteNum(int writNumber)
{
  uint8_t digits[10];     // Integer number has 10 digits max
  uint8_t Signum = 0; 
  uint8_t i = 0;
  if(writNumber) {
    if(writNumber < 0) {Signum = 1; writNumber = -writNumber;}
    while(writNumber) {
      digits[i] = (uint8_t) (writNumber%10) | 0x30;
      writNumber /= 10;
      i++;
    }
    if(Signum) WriteChar('-');
    while(i) WriteChar(digits[--i]);
  } else  WriteChar('0');
}

// Function for error message
void WriteErrMsg(const char *pointChar)
{
    WriteChar(0x0A);WriteChar('L');   // Write L (aka Line)
    WriteNum(currentLine);      // Number of line with error
    WriteChar(0x20);
    while(*pointChar) WriteChar(*pointChar++);  // and error message
    WriteChar(0x0A);
    RunMode = 0;    // Error == stop program
}

// ****************************************************************
//         Settings adc for polling (taken from ch32fun example)
// ****************************************************************
void adc_init( void )
{
	// ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
	RCC->CFGR0 &= ~(0x1F<<11);
	// Enable GPIOD and ADC
	RCC->APB2PCENR |= RCC_APB2Periph_ADC1;
	// PD4 is analog input chl 7
	GPIOD->CFGLR &= ~(0xf<<(4*4));	// CNF = 00: Analog, MODE = 00: Input
	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;
	// Set up single conversion on chl 7
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = 7;	// 0-9 for 8 ext inputs and two internals
	// set sampling time for chl 7
	ADC1->SAMPTR2 &= ~(ADC_SMP0<<(3*7));
	ADC1->SAMPTR2 |= 7<<(3*7);	// 0:7 => 3/9/15/30/43/57/73/241 cycles
	// turn on ADC and set rule group to sw trig
	ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL;
    // Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);
	// Calibrate
	ADC1->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);
	// should be ready for SW conversion now
}

// ****************************************************************
//         Serial I2C EEPROM functions
// ****************************************************************
// Write address of line of program - command + 2 bytes of address (address = lineNumber*32)
void I2C_write_address(unsigned char lineNumber)
{
    uint32_t timeout = TIMEOUT_MAX;
    while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
    if(timeout==-1) {I2C1->CTLR1 |= I2C_CTLR1_STOP; return;}
    // Set START condition
    I2C1->CTLR1 |= I2C_CTLR1_START;
    // Wait for master mode select
    if(I2CTimeout(I2C_EVENT_MASTER_MODE_SELECT)) return;
    I2C1->DATAR = 0xA0 | (compFlags & 0x0F);    // Send 24Cxxx memory address + DISK number
    // Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return;
    I2C1->DATAR = (lineNumber >> 3) | (noOfMemory << 4);  // Send upper adr. byte
    // Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
    I2C1->DATAR = (lineNumber << 5); // Send lower adr. byte
}

// Write 1 line of program - 32 bytes
void I2C_write_line(unsigned char lineNumber) 
{
    I2C_write_address(lineNumber);    // Write memory address of requested line
    for(int j=0; j<32; j++) {
        // Wait for transmit condition
        if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
        I2C1->DATAR = line_buffer[linePointer++];   // Send data
    }
    // Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
    // set STOP condition
    I2C1->CTLR1 |= I2C_CTLR1_STOP;
}

// Read 1 line of program - 32 bytes
void I2C_read_line(unsigned char lineNumber) 
{
    int j=0;  // Counter for received data
    I2C_write_address(lineNumber);    // Write memory address of requested line
    // Wait for transmit condition
    if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
    // set STOP condition
    I2C1->CTLR1 |= I2C_CTLR1_STOP;

    uint32_t timeout = TIMEOUT_MAX;
    while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
    if(timeout==-1) {I2C1->CTLR1 |= I2C_CTLR1_STOP; return;}
    // Set START condition
    I2C1->CTLR1 |= I2C_CTLR1_START;
    // Wait for master mode select
    if(I2CTimeout(I2C_EVENT_MASTER_MODE_SELECT)) return;
    I2C1->DATAR = 0xA0 | (compFlags & 0x0F) | 0x01;    // Send 24Cxxx memory address + DISK number + read
    // Wait for receive condition
    if(I2CTimeout(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return;
    I2C1->CTLR1 |= I2C_CTLR1_ACK;
    do
    {
        if(I2CTimeout(I2C_EVENT_MASTER_BYTE_RECEIVED)) return;   // Wait for ready read data
        line_buffer[j++] = I2C1->DATAR;
    } while(( j < 32 ) && (line_buffer[j-1] != (uint8_t)0xFF));   // Save byte to buffer until 32 bytes or EOL

    I2C1->CTLR1 &= ~I2C_CTLR1_ACK;
    // set STOP condition
    I2C1->CTLR1 |= I2C_CTLR1_STOP;
}
/*********************************************************************
    END Functions for I2C EEPROM END
 *********************************************************************/

/*********************************************************************
    Auxiliary functions for Tiny BASIC
 *********************************************************************/
// Function for LISTing of BASIC program line
uint8_t PrintBasLine(unsigned int lineProgr)
{
    I2C_read_line(lineProgr);
    if(line_buffer[0] == (unsigned char)0xFF) return 0;
    int j=0;
    // Number of line at beginning - right justified
    if(lineProgr <10) WriteChar(' ');
    if(lineProgr <100) WriteChar(' ');
    WriteNum(lineProgr); WriteChar(' ');
    // Now print program line
    while(line_buffer[j] != (unsigned char)0xFF) WriteChar(line_buffer[j++]);   // 0xFF means end of line text
    WriteChar(0x0A);
    return 1;
}

// Remove white spaces (and version moving to next position)
void RemoveWhtSP()
{
    while(line_buffer[linePointer] == ' ') { linePointer++; }
}
void RemoveNextWhtSP()
{
    linePointer++;
    while(line_buffer[linePointer] == ' ') { linePointer++; }
}

// Return pointer to number evaluated from line or NULL if no number
int* isNumberNext()
{
  evalNumber=0;
  if(line_buffer[linePointer] == 'b') {   // Binary number can be on the line
    linePointer++;
    if((line_buffer[linePointer] < '0') || (line_buffer[linePointer] > '1')) return(NULL);  // 0 or 1 must be next
    do{
      evalNumber = (evalNumber<<1) + (0x01 & line_buffer[linePointer]); linePointer++;  // Use LSB for filling number
    }while((line_buffer[linePointer] == '0') || (line_buffer[linePointer] == '1'));  // while 0 or 1
    return(&evalNumber);
  } else {    // Next is test for decadic number
    if((line_buffer[linePointer] < '0') || (line_buffer[linePointer] > '9')) return(NULL);  // 0 to 9 must be first
    do{
      evalNumber = evalNumber*10 + (0x0F & line_buffer[linePointer]); linePointer++;  // Use value for filling number
    }while((line_buffer[linePointer] >= '0') && (line_buffer[linePointer] <= '9'));  // repeat until 0 or 1
    return(&evalNumber);
  }
}

// Return pointer to variable evaluated from line or NULL if no variable
int* isVariableNext()
{
  int aux = line_buffer[linePointer];
  if(line_buffer[linePointer] == '@') // Array variable
    { linePointer++; 
      if((line_buffer[linePointer] >= 'A') && (line_buffer[linePointer] <= 'Z'))      // index is in variable A-Z
      { aux = charVariable[line_buffer[linePointer] - 'A'];   // Get value of index from variable
        linePointer++;
        // Test range of array index 0..99
        if((aux >= 0) && (aux<100)) return(arrayVariable + aux); else {WriteErrMsg("Out of array!"); return(NULL); }
      }
      else {      // Is it direct number?
        if((line_buffer[linePointer] >= '0') && (line_buffer[linePointer] <= '9')) {      // first digit of array index
          aux = 0x0F & line_buffer[linePointer]; linePointer++;
          if((line_buffer[linePointer] >= '0') && (line_buffer[linePointer] <= '9'))      // second digit of array index
            {aux = aux*10 + (0x0F & line_buffer[linePointer]); linePointer++;}
          return(arrayVariable + aux);
        }
      }
      WriteErrMsg("Index error!"); return(NULL); 
    }
  else
  if((line_buffer[linePointer] >= 'A') && (line_buffer[linePointer] <= 'Z')) 
    { linePointer++; return(charVariable + (aux - 'A')); }
  else return(NULL);
}

// Return pointer to factor evaluated from line (number or variable)
int* isFactor()
{
  int* numReturned;
  if((numReturned = isVariableNext()) != NULL) return(numReturned);
  else return(isNumberNext());
}

// Return pointer to term evaluated from line (term with multiplication or division)
int* isTerm()
{
  int auxValue;
  int* numReturned;
  unsigned char isDiv = 0;
  if((numReturned = isFactor()) != NULL) auxValue = *numReturned; else return(NULL);
  RemoveWhtSP();
  while((line_buffer[linePointer] == '/') || (line_buffer[linePointer] == '*')) {
    if(line_buffer[linePointer] == '/') isDiv=1; else isDiv=0;
    RemoveNextWhtSP();
    if((numReturned = isFactor()) != NULL) {
      if(isDiv) {if(*numReturned) auxValue /= *numReturned; else WriteErrMsg("Zero div!");}
      else auxValue *= *numReturned;
      RemoveWhtSP();
    } else {WriteTextln("Missing operand!"); return(NULL); }
  }
  evalNumber = auxValue;
  return(&evalNumber);
}

// Return pointer to members with addition or subtraction
int* isExpression()
{
  int auxValue;
  int* numReturned;
  unsigned char isMinus = 0;
  if(line_buffer[linePointer] == '-') { linePointer++; isMinus=1; RemoveWhtSP(); }
  if(line_buffer[linePointer] == '+') { linePointer++; RemoveWhtSP(); }
  if((numReturned = isTerm()) != NULL) auxValue = *numReturned; else return(NULL);
  if(isMinus) auxValue = -auxValue;
  RemoveWhtSP();
  while((line_buffer[linePointer] == '-') || (line_buffer[linePointer] == '+')) {
    if(line_buffer[linePointer] == '-') isMinus=1; else isMinus=0;
    RemoveNextWhtSP();
    if((numReturned = isTerm()) != NULL) {
      if(isMinus) auxValue -= *numReturned; else auxValue += *numReturned;
      RemoveWhtSP();
    } else {WriteTextln("Missing operand!"); return(NULL); }
  }
  evalNumber = auxValue;
  return(&evalNumber);
}

/*********************************************************************
    Most impotant function for Tiny BASIC interpreter - executes commands
 *********************************************************************/
void Statement()
{
  // Function for executing commands. Command is on the line_buffer[] pointed with linePointer
  int* numOnLine;
  // "Tiny Basic" commands can be added here:
  switch (line_buffer[linePointer]) {
    case (unsigned char)0xFF: break;  // Empty line
    // ...........................................................
    case 0xA9:    // LIST
      RemoveNextWhtSP();
      if((numOnLine = isNumberNext()) != NULL) PrintBasLine((*numOnLine) & 0x7F);
      else { int j=0;
      for(int i=0; i<128; i++) 
        { if(PrintBasLine(i)) j++;
          if(j == 10) {j=0; WriteTextln("Press any key..."); GetKeyChar();}  // List long program in 10 rows pages
        }
      }
      break;  // LIST
    // ...........................................................
    case 0xA8: line_buffer[0] = (unsigned char)0xFF;   // CLEAR
      for(uint8_t i=0; i<128; i++) {linePointer=0; I2C_write_line(i); My_Delay_Ms(10);} 
      RunMode = 0; break;  // end CLEAR
    // ...........................................................
    case 0xAA: RunMode = 1; currentLine = 0; stackPointer=0; break;  // RUN
    // ...........................................................
    case 0xAB: RunMode = 0; currentLine = 0; break; // END
    // ...........................................................
    case 0xA5: // LET
      { uint8_t ArrFlag=0;
      RemoveNextWhtSP();
      if(line_buffer[linePointer] == '@') ArrFlag=1;
      if((numOnLine = isVariableNext()) != NULL) 
        { int* pointVar = numOnLine;      // Here is variable address
          RemoveWhtSP();
          if(line_buffer[linePointer] == '=') {
            RemoveNextWhtSP();
            if((numOnLine = isExpression()) != NULL) {    // First variable is normally filled up
              *pointVar = *numOnLine;
              if(ArrFlag) {   // Array can be filled many times
                RemoveWhtSP();
                do{
                  if(line_buffer[linePointer] == ',') RemoveNextWhtSP();
                  pointVar++;
                  if(((numOnLine = isNumberNext()) != NULL) && (pointVar<(arrayVariable+sizeof(arrayVariable)))) *pointVar = *numOnLine;
                  RemoveWhtSP();
                } while (line_buffer[linePointer] == ',');
              }
            } else WriteErrMsg("Error value!");
          } else WriteErrMsg("Missing =");
        } else WriteErrMsg("Missing variable!");
      }
      break;  // end LET
    // ...........................................................
    case 0xA6: // GOSUB
      { unsigned char retLine = currentLine + 1;
        if(retLine < 128) { 
          if(stackPointer < 8) { 
            stack_buffer[stackPointer] = retLine; 
            stackPointer++;
            RemoveNextWhtSP();
            if( (numOnLine = isExpression()) != NULL )
              { if((*numOnLine >= 0) && (*numOnLine < 128)) currentLine = (unsigned char) *numOnLine; 
                else WriteErrMsg("Line number OVR!");
              }
            else WriteErrMsg("Error value!");
          }
        } else WriteErrMsg("Line number OVR!");
      }
      break;  // end GOSUB
    // ...........................................................
    case 0xA7: // RETURN
      if(stackPointer > 0) {stackPointer--; currentLine = stack_buffer[stackPointer];}
      else WriteErrMsg("No return!");
      break;  // RETURN
    // ...........................................................
    case 0xA3: // GOTO
      RemoveNextWhtSP();
      if( (numOnLine = isExpression()) != NULL )
        { if((*numOnLine >= 0) && (*numOnLine < 128)) currentLine = (unsigned char) *numOnLine; 
          else WriteErrMsg("Line number OVR!");
        }
        else WriteErrMsg("Error value!");
      break;  // end GOTO
    // ...........................................................
    case 0xA0: // PRINT
      RemoveNextWhtSP();
      do{
        if(line_buffer[linePointer] == ',') RemoveNextWhtSP();
        if((numOnLine = isExpression()) != NULL ) WriteNum(*numOnLine);
        else { 
          if(line_buffer[linePointer] == '"') {
            linePointer++;
            while((line_buffer[linePointer] != '"') && (line_buffer[linePointer] != 0xFF) && (linePointer < 32)) {
              if(line_buffer[linePointer] == '\\') {  // Escape sequences
                linePointer++; 
                if(line_buffer[linePointer] == 'n') WriteChar(0x0A);
                if(line_buffer[linePointer] == 't') WriteChar(0x09);
                if(line_buffer[linePointer] == 'f') WriteChar(0x0C); 
              }
              else WriteChar(line_buffer[linePointer]);
              linePointer++;
            }
            if(line_buffer[linePointer] == '"') linePointer++;
          }
        }
        RemoveWhtSP();
      } while (line_buffer[linePointer] == ',');
      break;  // end PRINT
    // ...........................................................
    case 0xA4:    // INPUT
      RemoveNextWhtSP();
      do{
        if(line_buffer[linePointer] == ',') RemoveNextWhtSP();
        uint8_t varName = linePointer;      // save pointer to variable name for prompt
        if( (numOnLine = isVariableNext()) != NULL ) { 
          WriteText("Insert "); 
          WriteChar(line_buffer[varName]);
          WriteTextln(":"); *numOnLine = 0;
          uint8_t recChar = '0'; 
          int minusConst = 1; 
          uint8_t isFirstChar = 1;
          // read integer number
          do{
            recChar = GetKeyChar(); WriteChar(recChar);
            // if char is number, proccess it
            if( isFirstChar && (recChar == '-')) { isFirstChar = 0; minusConst = -1; recChar = '0';}
            else { if((recChar >= '0') && (recChar <= '9')) { *numOnLine *= 10; *numOnLine += (0x0F & recChar); } }
          } while ((recChar >= '0') && (recChar <= '9'));
          *numOnLine *= minusConst;
        }
        RemoveWhtSP();
      } while (line_buffer[linePointer] == ',');
      break;  // end INPUT
    // ...........................................................
    case 0xA1:    // IF - THEN
      RemoveNextWhtSP();
      if( (numOnLine = isExpression()) != NULL ) {   // in the numLine variable is first expression
        int firstExpression = *numOnLine; 
        uint8_t typeRelOp = 0;
        RemoveWhtSP();
        if(line_buffer[linePointer] == '=') { typeRelOp = 1; linePointer++; } else
        if(line_buffer[linePointer] == '<') { typeRelOp = 2; RemoveNextWhtSP();
          if(line_buffer[linePointer] == '=') { typeRelOp = 3; linePointer++;  }
          else if(line_buffer[linePointer] == '>') { typeRelOp = 4; linePointer++; }
        } else
        if(line_buffer[linePointer] == '>') { typeRelOp = 5; RemoveNextWhtSP();
          if(line_buffer[linePointer] == '=') { typeRelOp = 6; linePointer++; }
          else if(line_buffer[linePointer] == '<') { typeRelOp = 4; linePointer++; }
        } else 
        if(line_buffer[linePointer] == '&') { typeRelOp = 7; linePointer++; }
        int* secExpression;
        RemoveWhtSP();
        if( (secExpression = isExpression()) != NULL ) {
          if(typeRelOp) {
            uint8_t relOP = 0;
            switch (typeRelOp) {
              case 1: if(firstExpression == *secExpression) relOP = 1; break;  // =
              case 2: if(firstExpression < *secExpression) relOP = 1; break;  // <
              case 3: if(firstExpression <= *secExpression) relOP = 1; break;  // <=
              case 4: if(firstExpression != *secExpression) relOP = 1; break;  // <>, ><
              case 5: if(firstExpression > *secExpression) relOP = 1; break;  // >
              case 6: if(firstExpression >= *secExpression) relOP = 1; break;  // >=
              case 7: if(firstExpression & *secExpression) relOP = 1; break;  // bit &
              }
            RemoveWhtSP();
            if(line_buffer[linePointer] == 0xA2) {
              RemoveNextWhtSP();
              if(relOP) Statement(); else {if(currentLine<127) currentLine++; else RunMode=0;}
            }  else WriteErrMsg("No THEN");
          } else WriteErrMsg("Operator error");
        } else WriteErrMsg("No 2. expr");
      } else WriteErrMsg("No 1. expr");   
      break;  // end IF - THEN
    // ...........................................................
    // ...........................................................
    // Extension "Basic" commands can be added here:
    case 0xAC:    // TIME
      RemoveNextWhtSP();
      if((numOnLine = isVariableNext()) != NULL) 
        {
	        *numOnLine = basTimer;
        } else WriteErrMsg("Missing variable!");
      break; // TIME
    // ...........................................................
    case 0xAD:     // CURSOR
      RemoveNextWhtSP();
      if((numOnLine = isExpression()) != NULL) { 
        if((*numOnLine >= 0) && (*numOnLine < 336))
        dispPointer = (uint16_t)*numOnLine;
      } else WriteErrMsg("Error value!");
      break;  // end CURSOR
    // ...........................................................
    case 0xAE:       // PUTCH
      RemoveNextWhtSP();
      uint8_t cursorDirection;
      // Test special characters for moving cursor
      if((line_buffer[linePointer] == '.') || (line_buffer[linePointer] == '^') || (line_buffer[linePointer] == '<') 
            ||(line_buffer[linePointer] == '>') || (line_buffer[linePointer] == 'v')|| (line_buffer[linePointer] == 'p')) {
        cursorDirection = line_buffer[linePointer];
        RemoveNextWhtSP();
        if((numOnLine = isExpression()) != NULL) 
          { 
            if(cursorDirection == 'p') { // Put char to position directly
              uint32_t charToPut = *numOnLine;    // save character value
              RemoveWhtSP();
              if(line_buffer[linePointer] == ',') {
                RemoveNextWhtSP();
                if((numOnLine = isExpression()) != NULL) {      // *numOnLine holds position
                  if((*numOnLine >= 0) && (*numOnLine < 336) && (((uint8_t)charToPut & 0xFF)>0x1F) && (((uint8_t)charToPut & 0xFF)<0xA0))   // Putchar to allowed range only
                    {   uint16_t saveDispPointer = dispPointer;
                        dispPointer = (uint16_t)*numOnLine;
                        VideoRAM[dispPointer] = ((uint8_t)charToPut & 0xFF)-0x20;
                        PlotChar((uint8_t)charToPut);
                        dispPointer = saveDispPointer;
                    }
                } else WriteErrMsg("Err posit!");
              } else WriteErrMsg("Missing ,");
            } else  // Put char to cursor position with cursor update
            if((((uint8_t)*numOnLine & 0xFF)>0x1F) && (((uint8_t)*numOnLine & 0xFF)<0xA0)) { 
              VideoRAM[dispPointer] = ((uint8_t)*numOnLine & 0xFF)-0x20; // Only printable
              PlotChar((uint8_t)*numOnLine);
              // And move(over) display pointer
              switch(cursorDirection)
              { // Cursor will be moved only in 4 cases - "p" is only flag
                case '^': { dispPointer -= 21; if(dispPointer>335) dispPointer += 336; } break;
                case '<': { dispPointer--; if(dispPointer>335) dispPointer=20; else if((dispPointer%21) == 20) dispPointer += 21;} break;
                case 'v': { dispPointer += 21; if(dispPointer>335) dispPointer -= 336; } break;
                case '>': { dispPointer++; if((dispPointer%21) == 0) dispPointer -= 21; } break;
              }
            }
          } else WriteErrMsg("Missing value!");
      } else  // No special character parameter, so normal print
      if((numOnLine = isExpression()) != NULL) 
        { 
          WriteChar((uint8_t)*numOnLine & 0xFF);
        } else WriteErrMsg("Missing value!");
      break;  // end PUTCH
    // ...........................................................
    case 0xAF:      // GETCH
      RemoveNextWhtSP();
      uint16_t VideoPoint=dispPointer;
      // Test special character for direct get
      if(line_buffer[linePointer] == 'p'){
        RemoveNextWhtSP();
        if((numOnLine = isExpression()) != NULL) {      // *numOnLine holds position
          if((*numOnLine >= 0) && (*numOnLine < 336))   // Get char from allowed range only
            VideoPoint=(uint16_t)*numOnLine; else VideoPoint = 999;
          RemoveWhtSP();
          if(line_buffer[linePointer] == ',') RemoveNextWhtSP(); else WriteErrMsg("Missing ,");
        } else WriteErrMsg("Err posit!");
      }
      if((VideoPoint<336) && ((numOnLine = isVariableNext()) != NULL)) 
        {
	        *numOnLine = 0x20 + VideoRAM[VideoPoint];
        } else WriteErrMsg("Err param!");
      break; // end GETCH
    // ...........................................................
    case 0xB0:      // INKEY
      RemoveNextWhtSP();
      if((numOnLine = isVariableNext()) != NULL) 
	      *numOnLine = kbdRead();
      else WriteErrMsg("Missing variable!");
      break; // end INKEY
    // ...........................................................
    case 0xB1:      // FILE
      RemoveNextWhtSP();
      if((numOnLine = isExpression()) != NULL) noOfMemory = (uint8_t)(*numOnLine & 0x0F);
      else {  // No parameter, so print setting
        WriteChar(0x0A);   // Print number of FILE on new line
        WriteNum(noOfMemory & 0x0F);
        WriteChar(0x0A);
      }
      break; // end FILE
    // ...........................................................
    case 0xB2:      // DISK
      RemoveNextWhtSP();
      if((line_buffer[linePointer] == 'f') && (compFlags & 0x80)) {compFlags |= 0x02;linePointer++;}
      else if(line_buffer[linePointer] == 'h') {compFlags &= 0xF0;linePointer++;} 
      else {
        WriteChar(0x0A);   // Print DISK character on new line
        if(compFlags & 0x02) WriteChar('f'); else WriteChar('h');
        WriteChar(0x0A);
      }
      break; // end DISK
    // ...........................................................
    case 0xB3:      // DELAY
      RemoveNextWhtSP();
      if((numOnLine = isExpression()) != NULL) 
        { if((*numOnLine > 0) && (*numOnLine <= 1000000)) My_Delay_Ms(*numOnLine); }
      else WriteErrMsg("Error value!");
      break; // end DELAY
    // ...........................................................
    case 0xB4:      // BEEP
      RemoveNextWhtSP();
      if((numOnLine = isExpression()) != NULL) 
        { int32_t Frequency = *numOnLine;      // First parameter is frequency
          RemoveWhtSP();
          if(line_buffer[linePointer] == ',') {
            RemoveNextWhtSP();
            if((numOnLine = isExpression()) != NULL) {      // *numOnLine holds duration (milisec)
              if((Frequency >= 100) && (Frequency <= 4000) && (*numOnLine >= 20) && (*numOnLine <= 100000))
              {
                Frequency = 500000/Frequency;   // Change Frequency to pulse duration HI or LOW [microsec]
                for(int32_t i=0; i<((*numOnLine * 500) / Frequency); i++ ) {
                  GPIOC->BSHR = 0x00000001;
                  My_Delay_Us(Frequency);
                  GPIOC->BSHR = 0x00010000;
                  My_Delay_Us(Frequency);
                }
              }
            } else WriteErrMsg("Error Time!");
          } else WriteErrMsg("Missing ,");
        } else WriteErrMsg("Error Freq!");
      break; // end BEEP
    // ...........................................................
    case 0xB5:  break; // REM
    // ...........................................................
    case 0xBB:      // COPY
      RunMode = 0; currentLine = 0;   // COPY stops program every time
      RemoveNextWhtSP();
      if(line_buffer[linePointer] == 's') {     // Copy lines in this file
        RemoveNextWhtSP();
        if((numOnLine = isNumberNext()) != NULL) { 
          int32_t Source = *numOnLine;      // First parameter is Source line number
          RemoveWhtSP();
          if(line_buffer[linePointer] == ',') {
            RemoveNextWhtSP();
            if((numOnLine = isNumberNext()) != NULL) {
              int32_t Target = *numOnLine;      // Second parameter is Target line number
              RemoveWhtSP();
              if(line_buffer[linePointer] == ',') {
                RemoveNextWhtSP();
                if((numOnLine = isNumberNext()) != NULL) {
                  int32_t nCount = *numOnLine;      // Third parameter is Count line number
                  // Check parameters range
                  if((Source<128)&&(Target<128)&&(nCount>0)&&(nCount<128)&&(Source!=Target)) {
                    int32_t UpOrDown;
                    if(Source>Target) {
                      UpOrDown=1;   // Copy must be from low to high
                      if((Source+nCount)>127) nCount=128-Source;    // Trim counter to max possible
                    } else {
                      UpOrDown=-1;   // Copy must be from high to low
                      if((Target+nCount)>127) nCount=128-Target;    // Trim counter to max possible
                      Source += (nCount-1); Target += (nCount-1);
                    }
                    do {    // Ready to copy, so move it
                      I2C_read_line((uint8_t)Source); My_Delay_Ms(5);
                      linePointer = 0;I2C_write_line((uint8_t)Target); My_Delay_Ms(10);
                      Source+=UpOrDown;Target+=UpOrDown;nCount--;
                    } while(nCount);
                  } else WriteErrMsg("Over range!");
                } else WriteErrMsg("Error Count!");
              } else WriteErrMsg("Missing ,");
            } else WriteErrMsg("Error Target!");
          } else WriteErrMsg("Missing ,");
        } else WriteErrMsg("Error Source!");
      } else
      if((line_buffer[linePointer] == 'h') || (line_buffer[linePointer] == 'f')) {     // Copy this file to h or f disk
        uint8_t ChDisk = line_buffer[linePointer];
        RemoveNextWhtSP();
        if(!(compFlags & 0x80) && (ChDisk == 'f')) WriteErrMsg("No f DISK!");   // Is Disk f connected
        else {    // We can copy, f disk is connected or h disk must be on board
          if((numOnLine = isNumberNext()) != NULL) {    // Get number of file
            uint8_t Target = (uint8_t)(*numOnLine & 0x0F);
            uint8_t OldnoOfMemory = noOfMemory;   // Save variables for I2C memory communication
            uint8_t OldcompFlags = compFlags;
            for(uint8_t i=0; i<128; i++) {    // Copy 128 lines of program to new destination
              I2C_read_line(i); My_Delay_Ms(5);
              noOfMemory = Target;
              if(ChDisk == 'h') compFlags = 0x00; else compFlags = 0x02;    // h/f Disk address is different
              linePointer = 0;
              I2C_write_line(i); My_Delay_Ms(10);
              noOfMemory = OldnoOfMemory;
              compFlags = OldcompFlags;
            }   // end of copy so signal it
            WriteTextln("\nCopy to disk "); WriteChar(ChDisk);
          } else WriteErrMsg("No file number!");
        }
      } else WriteErrMsg("Error parameter");
      break; // end COPY
    // ...........................................................
    // ...........................................................
    // Extension "Input / Output" commands can be added here:
    case 0xB6:      // AINP
      RemoveNextWhtSP();
      if((numOnLine = isVariableNext()) != NULL) 
        { 
	        ADC1->CTLR2 |= ADC_SWSTART;   // Start sw conversion
        	while(!(ADC1->STATR & ADC_EOC)) {};    // Wait for conversion complete
          *numOnLine = ADC1->RDATAR;  // Save value to variable
        } else WriteErrMsg("Missing variable!");
      break; // end AINP
    // ...........................................................
    case 0xB7:     // DINP
      RemoveNextWhtSP();
      if((numOnLine = isVariableNext()) != NULL) 
        { // First parameter is variable
          RemoveWhtSP();
          if(line_buffer[linePointer] == ',') {
            RemoveNextWhtSP();
            int* numPin;
            if((numPin = isExpression()) != NULL){ // Second parameter is pin number 
              switch(*numPin & 0x07)
              { // Cursor will be moved only in 4 cases - "p" is only flag
                case 0: {      // Is input 8?
                    funPinMode( PC3, GPIO_CFGLR_IN_PUPD ); funDigitalWrite( PC3, 1 );
                    *numOnLine = funDigitalRead( PC3 );
                    }  break; 
                case 1: {      // Is input 1?
                    funPinMode( PA2, GPIO_CFGLR_IN_PUPD ); funDigitalWrite( PA2, 1 );
                    *numOnLine = funDigitalRead( PA2 );
                    }  break; 
                case 2: {      // Is input 2?
                    funPinMode( PA1, GPIO_CFGLR_IN_PUPD ); funDigitalWrite( PA1, 1 );
                    *numOnLine = funDigitalRead( PA1 );
                    }  break; 
                case 3: {      // Is input 3?
                    funPinMode( PD6, GPIO_CFGLR_IN_PUPD ); funDigitalWrite( PD6, 1 );
                    *numOnLine = funDigitalRead( PD6 );
                    }  break; 
                case 4: {      // Is input 4?
                    funPinMode( PD5, GPIO_CFGLR_IN_PUPD ); funDigitalWrite( PD5, 1 );
                    *numOnLine = funDigitalRead( PD5 );
                    }  break; 
                case 5: {      // Is input 5?
                    funPinMode( PC7, GPIO_CFGLR_IN_PUPD ); funDigitalWrite( PC7, 1 );
                    *numOnLine = funDigitalRead( PC7 );
                    }  break; 
                case 6: {      // Is input 6?
                    funPinMode( PC6, GPIO_CFGLR_IN_PUPD ); funDigitalWrite( PC6, 1 );
                    *numOnLine = funDigitalRead( PC6 );
                    }  break; 
                case 7: {      // Is input 7?
                    funPinMode( PC4, GPIO_CFGLR_IN_PUPD ); funDigitalWrite( PC4, 1 );
                    *numOnLine = funDigitalRead( PC4 );
                    }  break; 
                }
            } else WriteErrMsg("Error Input!");
          } else WriteErrMsg("Missing ,");
        } else WriteErrMsg("Error variable!");
      break; // end DINP
    // ...........................................................
    case 0xB8:     // DOUT
      RemoveNextWhtSP();
      // Special parameter for driving WS2812 chain (from CH32fun extralibs)
      if(line_buffer[linePointer] == 'w') {
        RemoveNextWhtSP();
        if((numOnLine = isExpression()) != NULL)
        {
            funPinMode( PA2, GPIO_CFGLR_OUT_10Mhz_PP ); // Set as output
            GPIOA->BSHR = 0x00040000;           // and clear it
            int LEDcount = *numOnLine;           // Get how many LEDs is connected
            if(LEDcount > 99) LEDcount = 100;   // Maximum lenght of array

        	int j = 0;
        	while( j < LEDcount )
        	{
        		uint32_t LEDbyte = arrayVariable[j];
        		int i;
        		for( i = 0; i < 24; i++ )
        		{     // From ch32fun library
        			if( LEDbyte & 0x800000 )
        			{
        				// WS2812B's need AT LEAST 625ns for a logical "1"
        				GPIOA->BSHR = 0x00000004;
        				DelaySysTick(25);
        				GPIOA->BSHR = 0x00040000;
        				DelaySysTick(1);
        			}
        			else
        			{
        				// WS2812B's need BETWEEN 62.5 to about 500 ns for a logical "0"
        				__disable_irq();
        				GPIOA->BSHR = 0x00000004;
        				asm volatile( "nop\nnop\nnop\nnop" );
        				GPIOA->BSHR = 0x00040000;
        				__enable_irq();
        				DelaySysTick(15);
        			}
        			LEDbyte <<= 1;
        		}
        		j++;
        	}
          	GPIOA->BSHR = 0x00040000;         
        }  else WriteErrMsg("Error variable!");
      } else
      if((numOnLine = isExpression()) != NULL) 
        { int outValue = *numOnLine;      // First parameter is value
          RemoveWhtSP();
          if(line_buffer[linePointer] == ',') {
            RemoveNextWhtSP();
            if((numOnLine = isExpression()) != NULL){ // Second parameter is pin number
              switch(*numOnLine & 0x07)
              { // Set output pin
                case 0: {      // Is output 8?
                    funPinMode( PC3, GPIO_CFGLR_OUT_10Mhz_PP ); 
                    if(outValue) {funDigitalWrite( PC3, FUN_HIGH );} else {funDigitalWrite( PC3, FUN_LOW );}
                    }  break; 
                case 1: {      // Is output 1?
                    funPinMode( PA2, GPIO_CFGLR_OUT_10Mhz_PP ); 
                    if(outValue) {funDigitalWrite( PA2, FUN_HIGH );} else {funDigitalWrite( PA2, FUN_LOW );}
                    }  break; 
                case 2: {      // Is output 2?
                    funPinMode( PA1, GPIO_CFGLR_OUT_10Mhz_PP ); 
                    if(outValue) {funDigitalWrite( PA1, FUN_HIGH );} else {funDigitalWrite( PA1, FUN_LOW );}
                    }  break; 
                case 3: {      // Is output 3?
                    funPinMode( PD6, GPIO_CFGLR_OUT_10Mhz_PP ); 
                    if(outValue) {funDigitalWrite( PD6, FUN_HIGH );} else {funDigitalWrite( PD6, FUN_LOW );}
                    }  break; 
                case 4: {      // Is output 4?
                    funPinMode( PD5, GPIO_CFGLR_OUT_10Mhz_PP ); 
                    if(outValue) {funDigitalWrite( PD5, FUN_HIGH );} else {funDigitalWrite( PD5, FUN_LOW );}
                    }  break; 
                case 5: {      // Is output 5?
                    funPinMode( PC7, GPIO_CFGLR_OUT_10Mhz_PP ); 
                    if(outValue) {funDigitalWrite( PC7, FUN_HIGH );} else {funDigitalWrite( PC7, FUN_LOW );}
                    }  break; 
                case 6: {      // Is output 6?
                    funPinMode( PC6, GPIO_CFGLR_OUT_10Mhz_PP ); 
                    if(outValue) {funDigitalWrite( PC6, FUN_HIGH );} else {funDigitalWrite( PC6, FUN_LOW );}
                    }  break; 
                case 7: {      // Is output 7?
                    funPinMode( PC4, GPIO_CFGLR_OUT_10Mhz_PP ); 
                    if(outValue) {funDigitalWrite( PC4, FUN_HIGH );} else {funDigitalWrite( PC4, FUN_LOW );}
                    }  break; 
                }
            } else WriteErrMsg("Error output pin!");
          } else WriteErrMsg("Missing ,");
        } else WriteErrMsg("Error variable!"); 
      break; // end DOUT
    // ...........................................................
    case 0xB9:     // I2CW
      RemoveNextWhtSP();
      if((numOnLine = isExpression()) != NULL) {
        if(compFlags & 0x40)
          { 
            // Communication is similar to 24Cxxx - see above
        	// Wait for I2C not busy
        	uint32_t timeout = TIMEOUT_MAX;
        	while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
        	if(timeout==-1) {I2C1->CTLR1 |= I2C_CTLR1_STOP; return;}
        	// Set START condition
        	I2C1->CTLR1 |= I2C_CTLR1_START;
        	// Wait for master mode select
            if(I2CTimeout(I2C_EVENT_MASTER_MODE_SELECT)) return;
        	// send 7-bit address + write flag
        	I2C1->DATAR = 0x40;
        	// Wait for transmit condition
            if(I2CTimeout(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return;
        	// Send LSB
        	I2C1->DATAR = (*numOnLine & 0xFF);
        	// Wait for transmit condition
            if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
        	// Send MSB
        	I2C1->DATAR = (*numOnLine >> 8);
        	// Wait for transmit condition
            if(I2CTimeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
        	// set STOP condition
        	I2C1->CTLR1 |= I2C_CTLR1_STOP;
          } 
      } else WriteErrMsg("Missing value!");
      break; // end I2CW
    // ...........................................................
    case 0xBA:     // I2CR
      RemoveNextWhtSP();
      if((numOnLine = isVariableNext()) != NULL) {
        if(compFlags & 0x40) 
          { 
            // Communication is similar to 24Cxxx - see above
        	// Wait for I2C not busy
        	uint32_t timeout = TIMEOUT_MAX;
        	while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
        	if(timeout==-1) {I2C1->CTLR1 |= I2C_CTLR1_STOP; return;}
        	// Set START condition
        	I2C1->CTLR1 |= I2C_CTLR1_START;
        	// Wait for master mode select
            if(I2CTimeout(I2C_EVENT_MASTER_MODE_SELECT)) return;
        	// send 7-bit address + write flag
        	I2C1->DATAR = 0x41;
        	// Wait for transmit condition
            if(I2CTimeout(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return;
            I2C1->CTLR1 |= I2C_CTLR1_ACK;
        	// Wait for transmit condition
            if(I2CTimeout(I2C_EVENT_MASTER_BYTE_RECEIVED)) return;
        	// Read LSB
        	int recI2Cval = I2C1->DATAR;
            if(I2CTimeout(I2C_EVENT_MASTER_BYTE_RECEIVED)) return;
        	// Read MSB
        	recI2Cval |= I2C1->DATAR << 8;
            I2C1->CTLR1 &= ~I2C_CTLR1_ACK;
        	// set STOP condition
        	I2C1->CTLR1 |= I2C_CTLR1_STOP;
            *numOnLine = recI2Cval;
          }
      } else WriteErrMsg("Missing variable!");
      break; // end I2CR
    // ...........................................................
    // ...........................................................

    default:
      WriteErrMsg("no such command!");
    }
}

/*********************************************************************
    Main program function with infinite loop
 *********************************************************************/
int main()
{
	SystemInit();

	funGpioInitAll();  // Enable GPIOs
    // Prepare speaker output
    funPinMode( PC0, GPIO_CFGLR_OUT_10Mhz_PP ); 
    funDigitalWrite( PC0, FUN_LOW );
    IIC_Init();     // Initialize I2C module
    // Initialize SysTick to trigger an IRQ with auto-reload, using HCLK/1 as clock source
	// Reset any pre-existing configuration
	SysTick->CTLR = 0x0000;
	// Set the compare register to trigger once per millisecond
	SysTick->CMP = 1503;
	// Reset the Count Register, and the global millis counter to 0
	SysTick->CNT = 0x00000000;
	// NOTE: By not setting SYSTICK_CTLR_STRE, we maintain compatibility with
	// busywait delay funtions used by ch32v003_fun.
	SysTick->CTLR |= SYSTICK_CTLR_STE   |  // Enable Counter
	                 SYSTICK_CTLR_STIE  |  // Enable Interrupts
	                 SYSTICK_CTLR_STCLK ;  // Set Clock Source to HCLK/1
	// Enable the SysTick IRQ
	NVIC_EnableIRQ(SysTicK_IRQn);

	My_Delay_Ms(100);   // For display stabilization after RESET
    // Check devices on I2C bus
    int FlagNoPassedDelay = 1;
    if(CheckI2CDevice(0xA2)) compFlags |= 0x80;      // Disk f - second EEPROM
    if(CheckI2CDevice(0x40)) compFlags |= 0x40;      // Expander PCF8575
    if(CheckI2CDevice(0x78)) compFlags |= 0x20;      // OLED display
    else {      // Short beep if no OLED
        for(int32_t i=0; i<200; i++ ) {
            GPIOC->BSHR = 0x00000001; My_Delay_Us(500);
            GPIOC->BSHR = 0x00010000; My_Delay_Us(500);
        }
        FlagNoPassedDelay = 0;
    }
    // Display init
    if(ISOLED) OLEDinit();
    WriteChar(0x0C);    // Clear display

    adc_init(); // Start A/D converter
    line_buffer[32] = (unsigned char)0xFF;  // Stop symbol at end of buffer

    WriteTextln("TinyBasic 1.0");  // Introductory message
    if(!ISDISKf) WriteTextln("No DISK f");
    if(!ISEXPANDER) WriteTextln("No PCF8575");
  
    // Test for autorun (Is statement RUN - 0xAA in the first byte of file 0?)  
    I2C_read_line(0);   // Read the first line of file
    if(line_buffer[0] == (unsigned char)0xAA) {   // Test first byte for RUN statement
        linePointer=1;    // First byte is bytecode of RUN, so move to next byte...
        KbdDataBuffer = 0x0A;   // ... and pretend that user pressed Enter after RUN
    }

    // Unfortunately - CardKB starts after about 1 sec blinking with LED, so we must delay for keyboard awakening
    if(FlagNoPassedDelay) My_Delay_Ms(900); else My_Delay_Ms(700);   // No delay if missing OLED, so wait more for CardKB
    if(CheckI2CKeyboard()) compFlags |= 0x10;      // CardKB is connected
    if(!ISKEYB) WriteTextln("No CardKB");

    char recChar;   // Processed character from keyboard
    // Program main loop starts here
    while(1)
    {
        // Get char to buffer (only to max size of buffer or end of line (Enter))
        recChar = GetKeyChar();
        // and return it to display
        if(recChar == 0x09) {  // Tab needs special cure 
          if(linePointer < 27) WriteText("    "); // Tab inserts only 4 spaces
        } else {  // Other characters
          if(linePointer < 32) WriteChar(recChar);
        }
        // Char must be saved to line_buffer
        // Is it char to start process line (Enter)?
        if (recChar == '\n') {
          // look for the newline. That's the end of your sentence:
          line_buffer[linePointer] = (unsigned char)0xFF;    // End of line is signalled with 0xFF
          currentLine = 0; linePointer=0;  // Prepare pointer to beginning,...
          // ... and process line
          int* numLine;
          RemoveWhtSP();
          if((numLine = isNumberNext()) != NULL) {    // If line starts with number ...
            RemoveWhtSP();  // ... discard number and white spaces ...
            if((*numLine >= 0) && (*numLine < 128)) I2C_write_line((unsigned char) *numLine); // ... and save line
            else WriteErrMsg("Error line number!");
          }
          else {  // No number at begin of line
            comENDTimer = 0;    // Prepare counter for test END command
            do{ // Is it time for test END command to break program?
                if(comENDTimer > 31914) {    // time of 1 sec
                    comENDTimer = 0;
                    recChar = readActKbd();
                    // Enter is 0x0D - change it to 0x0A
                    if(recChar == 0x0D) recChar = 0x0A;
                    if(recChar < 0xBF) KbdDataBuffer = recChar;
                    if(recChar == 0xAB) RunMode = 0;
                }
                // Execute statements while RunMode
                uint8_t auxLine = currentLine;    // Save line number
                // -----------------------------------------------------------
                Statement();
                // Test for the second Statement on the line
                RemoveWhtSP();  // IF, GOTO or GOSUB will discard second Statement
                if((auxLine == currentLine) && (line_buffer[linePointer] == ';')) {
                  RemoveNextWhtSP();
                  Statement();
                }
                // Test for the third Statement on the line
                RemoveWhtSP();  // IF, GOTO or GOSUB will discard third Statement
                if((auxLine == currentLine) && (line_buffer[linePointer] == ';')) {
                  RemoveNextWhtSP();
                  Statement();
                }
                if(RunMode && (auxLine == currentLine)) currentLine++; // No jump, so go to next line
                // Prepare next program line
                if(currentLine > 127) { RunMode = 0; currentLine = 0;} // Number of line step over maximum line
                else { I2C_read_line(currentLine); linePointer = 0; }   // Line number is OK, so prepare it
            } while(RunMode);
            WriteTextln("\nOK");
          }
          // ... line was proccessed so...
          linePointer=0;  // ... move pointer to beginning again.
        }
        else {       // No end of line - just process character
          if(recChar==0x08) {    // Is backspace, so return char(s) in the line
            if (linePointer > 0) {  // Edit can be only in current line
              linePointer--;    // Return pointer 1 char
              if(line_buffer[linePointer] < 0xA0) { dispPointer--;VideoRAM[dispPointer] = 0x00;} // Discard normal character
              else {  // It is keyword - that means rather difficult
                if(line_buffer[linePointer] < 0xBC) { // Just within keyword range
                  int pKeyWord = ((line_buffer[linePointer] - (uint8_t)0xA0)) << 3;  // Get lenght of keyword
                  for(int j=0; j<KeyWordTab[pKeyWord];j++){
                    VideoRAM[dispPointer] = 0x00; PlotChar(' ');    // Discard 1 letter of keyword
                    // Test pointer and stop roll display off
                    if(dispPointer>1) dispPointer--;
                  }
                  VideoRAM[dispPointer] = 0x00;     // And the discard last letter of keyword
                }
              } 
            }
          } else {
            if (linePointer < 32) {
              // Put normal char to line buffer
              if(recChar == 0x09) {    // Special char is Tab
                if(linePointer < 27) {  // Expand Tab to 4 spaces (for future backspace)
                  line_buffer[linePointer++] = ' '; line_buffer[linePointer++] = ' ';
                  line_buffer[linePointer++] = ' '; line_buffer[linePointer++] = ' '; 
                }
              } else {
              // Save real "normal" char finally
                line_buffer[linePointer] = recChar;
                linePointer++;  // max 32 chars in buffer
              }
            }
          }
        }
    }
}

