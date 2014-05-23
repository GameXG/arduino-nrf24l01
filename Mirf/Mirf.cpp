/**
 * Mirf
 *
 * Additional bug fixes and improvements
 *  11/03/2011:
 *   Switched spi library.
 *  07/13/2010:
 *   Added example to read a register
 *  11/12/2009:
 *   Fix dataReady() to work correctly
 *   Renamed keywords to keywords.txt ( for IDE ) and updated keyword list
 *   Fixed client example code to timeout after one second and try again
 *    when no response received from server
 * By: Nathan Isburgh <nathan@mrroot.net>
 * $Id: mirf.cpp 67 2010-07-13 13:25:53Z nisburgh $
 *
 *
 * An Ardunio port of:
 * http://www.tinkerer.eu/AVRLib/nRF24L01
 *
 * Significant changes to remove depencence on interupts and auto ack support.
 *
 * Aaron Shrimpton <aaronds@gmail.com>
 *
 */

/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.

    $Id: mirf.cpp 67 2010-07-13 13:25:53Z nisburgh $
*/

#include "Mirf.h"
// Defines for setting the MiRF registers for transmitting or receiving mode

Nrf24l Mirf = Nrf24l();

Nrf24l::Nrf24l(){
	cePin = 8;
	csnPin = 7;
	channel = 1;
	payload = 16;
	spi = NULL;
}

void Nrf24l::transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len){
	uint8_t i;
	for(i = 0;i < len;i++){
		datain[i] = spi->transfer(dataout[i]);
	}
}

void Nrf24l::transmitSync(uint8_t *dataout,uint8_t len){
	uint8_t i;
	for(i = 0;i < len;i++){
		spi->transfer(dataout[i]);
	}
}


void Nrf24l::init() 
// Initializes pins to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
{   
    pinMode(cePin,OUTPUT);
    pinMode(csnPin,OUTPUT);

    ceLow();
    csnHi();

    // Initialize spi module
    spi->begin();

}


void Nrf24l::config() 
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
// NB: channel and payload must be set now.
{
    // 配置无线信道
	configRegister(RF_CH,channel);

    // Set length of incoming payload 
	// 接收数据通道 有效数据宽度 最大32
	configRegister(RX_PW_P0, payload);
	configRegister(RX_PW_P1, payload);

    // Start receiver 
    powerUpRx();
    flushRx();
}

void Nrf24l::setRADDR(uint8_t * adr) 
// Sets the receiving address
{
	ceLow();
	writeRegister(RX_ADDR_P1,adr,mirf_ADDR_LEN);
	ceHi();
}

void Nrf24l::setTADDR(uint8_t * adr)
// Sets the transmitting address
{
	/*
	 * RX_ADDR_P0 must be set to the sending addr for auto ack to work.
	 */

	writeRegister(RX_ADDR_P0,adr,mirf_ADDR_LEN);
	writeRegister(TX_ADDR,adr,mirf_ADDR_LEN);
}

extern bool Nrf24l::dataReady() 
// Checks if data is available for reading
// 检测是否有可读的数据
{
    // See note in getData() function - just checking RX_DR isn't good enough
	// 查看 getData() 函数说明，
	// 只检查 RX_DR(接收数据中断，当接收到有效数据后置一) 并不是好办法。
	uint8_t status = getStatus();

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
	// 可以快速的检测 RX_DR ，但是如果没有检测到，还得检测FIFO是否空。
    if ( status & (1 << RX_DR) ) return 1;
    return !rxFifoEmpty();
}

extern bool Nrf24l::rxFifoEmpty(){
// 返回 RX FIFO 寄存器空标志。
// 1:RX FIFO 寄存器空
// 0: RX FIFO 寄存器非空
	uint8_t fifoStatus;

	readRegister(FIFO_STATUS,&fifoStatus,sizeof(fifoStatus));
	return (fifoStatus & (1 << RX_EMPTY));
}



extern void Nrf24l::getData(uint8_t * data) 
// Reads payload bytes into data array
//读RX 有效数据：1-32 字节。读操作全部从字节0 开始。当读RX
//有效数据完成后，FIFO 寄存器中有效数据被清除。
//应用于接收模式下。
{
    csnLow();                               // Pull down chip select
    spi->transfer( R_RX_PAYLOAD );            // Send cmd to read rx payload
    transferSync(data,data,payload); // Read payload
    csnHi();                               // Pull up chip select
    // NVI: per product spec, p 67, note c:
    //  "The RX_DR IRQ is asserted by a new packet arrival event. The procedure
    //  for handling this interrupt should be: 1) read payload through SPI,
    //  2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more 
    //  payloads available in RX FIFO, 4) if there are more data in RX FIFO,
    //  repeat from step 1)."
    // So if we're going to clear RX_DR here, we need to check the RX FIFO
    // in the dataReady() function
// NVI：每个产品的规格，第67页，注三： 
//“的RX_DR IRQ是断言一个新的数据包到达事件的过程 
//处理这个中断应该是：1）通过SPI读取有效负载， 
//2）明确RX_DR IRQ，3）读FIFO_STATUS，以检查是否有更多的 
//在RX FIFO中可用的有效载荷，4）如果在RX FIFO更多的数据， 
//重复从步骤1开始）。“
//所以，如果我们要在这里清除RX_DR，我们需要检查RX FIFO 
//在dataReady（）函数
    configRegister(STATUS,(1<<RX_DR));   // Reset status register
}

void Nrf24l::configRegister(uint8_t reg, uint8_t value)
// Clocks only one byte into the given MiRF register
{
    csnLow();
	// (REGISTER_MASK & reg) 
	// 寄存器地址只有5位，
	// 通过 REGISTER_MASK(0x1F = 11111) 把输入的超过5位的地址给过滤成0
	// 防止干扰到8-6位的操作寄存器指令
	
	//  W_REGISTER
    //  写寄存器命令  W_REGISTER = 0x20 = 00100000
    //  W_REGISTER 001A AAAA 写配置寄存器。AAAAA 指出写操作的寄存器地址
    //  只有在掉电模式和待机模式下可操作。
    spi->transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi->transfer(value);
    csnHi();
}

void Nrf24l::readRegister(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
    csnLow();
    spi->transfer(R_REGISTER | (REGISTER_MASK & reg));
    transferSync(value,value,len);
    csnHi();
}

void Nrf24l::writeRegister(uint8_t reg, uint8_t * value, uint8_t len) 
// Writes an array of bytes into inte the MiRF registers.
{
    csnLow();
    spi->transfer(W_REGISTER | (REGISTER_MASK & reg));
    transmitSync(value,len);
    csnHi();
}


void Nrf24l::send(uint8_t * value) 
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
    uint8_t status;
    status = getStatus();

    while (PTX) {
	    status = getStatus();
//TX_DS 5 0 R/W 数据发送完成中断。当数据发送完成后产生中
//断。如果工作在自动应答模式下，只有当接收到应答信号后此位置一。
//写‘1’清除中断。
//MAX_RT 4 0 R/W 达到最多次重发中断。
//写‘1’清除中断。
//如果MAX_RT 中断产生则必须清除后系统才
//能进行通讯。
	    if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
		    PTX = 0;
		    break;
	    }
    }                  // Wait until last paket is send

    ceLow();
    
    powerUpTx();       // Set to transmitter mode , Power up
    
    csnLow();                    // Pull down chip select
    spi->transfer( FLUSH_TX );     // Write cmd to flush tx fifo
    csnHi();                    // Pull up chip select
    
    csnLow();                    // Pull down chip select
	//W_RX_PAYLOAD  1010 0000 写TX 有效数据：1-32 字节。写操作从字节0 开始。
    //应用于发射模式下
    spi->transfer( W_TX_PAYLOAD ); // Write cmd to write payload
    transmitSync(value,payload);   // Write payload
    csnHi();                    // Pull up chip select

    ceHi();                     // Start transmission
}

void Nrf24l::send_head() 
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
    uint8_t status;
    status = getStatus();

    while (PTX) {
	    status = getStatus();
//TX_DS 5 0 R/W 数据发送完成中断。当数据发送完成后产生中
//断。如果工作在自动应答模式下，只有当接收到应答信号后此位置一。
//写‘1’清除中断。
//MAX_RT 4 0 R/W 达到最多次重发中断。
//写‘1’清除中断。
//如果MAX_RT 中断产生则必须清除后系统才
//能进行通讯。
	    if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
		    PTX = 0;
		    break;
	    }
    }                  // Wait until last paket is send

    ceLow();
    
    powerUpTx();       // Set to transmitter mode , Power up
    
    csnLow();                    // Pull down chip select
    spi->transfer( FLUSH_TX );     // Write cmd to flush tx fifo
    csnHi();                    // Pull up chip select
    
    csnLow();                    // Pull down chip select
	//W_RX_PAYLOAD  1010 0000 写TX 有效数据：1-32 字节。写操作从字节0 开始。
    //应用于发射模式下
    spi->transfer( W_TX_PAYLOAD ); // Write cmd to write payload
}

void Nrf24l::send_content(uint8_t * data,uint8_t length) 
{
    transmitSync(data,length);   // Write payload
}
void Nrf24l::send_end() 
{
    csnHi();                    // Pull up chip select

    ceHi();                     // Start transmission
}





/**
 * isSending.
 *
 * Test if chip is still sending.
 * When sending has finished return chip to listening.
 *

 */
/** 
?* isSending。 
?* 
?*如果测试芯片仍然是发送。 
?*当发送完成后返回芯片听。 
?* 
?*/
bool Nrf24l::isSending(){
	uint8_t status;
	if(PTX){
		status = getStatus();
	    	
		/*
		 *  if sending successful (TX_DS) or max retries exceded (MAX_RT).
		 如果发送完成，或者达到最大重试次数还是没发送成功。
		 */

		if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
			powerUpRx();
			return false; 
		}

		return true;
	}
	return false;
}

uint8_t Nrf24l::getStatus(){
// 获得状态寄存器
	uint8_t rv;
	readRegister(STATUS,&rv,1);
	return rv;
}

// 上电
void Nrf24l::powerUpRx(){
	PTX = 0;
	ceLow();
	// 使用 ‘0’-8 位CRC 校验模式，开启1:上电模式，开启接收模式
	configRegister(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) );
	ceHi();
	// 清除之前的数据发送完成中断。不是关闭中断
	// 当数据发送完成后产生中断。如果工作在自动应答模式下，
	// 只有当接收到应答信号后此位置一。写‘1’清除中断。
	
	// 清除之前的达到最多次重发中断。不是关闭中断
	// 写‘1’清除中断。如果MAX_RT 中断产生则必须清除后系统才能进行通讯。
	configRegister(STATUS,(1 << TX_DS) | (1 << MAX_RT)); 
}

void Nrf24l::flushRx(){
//清除RX FIFO 寄存器，应用于接收模式下。
//在传输应答信号过程中不应执行此指令。也就是说，若传输应答
//信号过程中执行此指令的话将使得应答信号不能被完整的传输。
    csnLow();
    spi->transfer( FLUSH_RX );
    csnHi();
}

void Nrf24l::powerUpTx(){
// 上电  发送模式 1 0 1 数据在TX FIFO 寄存器中
	PTX = 1;
	configRegister(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) );
}

void Nrf24l::ceHi(){
// 高电平：发送模式、接收模式；低电平：待机模式
// 待机模式是配置无线模块
	digitalWrite(cePin,HIGH);
}

void Nrf24l::ceLow(){
// 高电平：发送模式、接收模式；低电平：待机模式
// 待机模式是配置无线模块
	digitalWrite(cePin,LOW);
}

void Nrf24l::csnHi(){
//片选使能，低电平使能
	digitalWrite(csnPin,HIGH);
}

void Nrf24l::csnLow(){
//片选使能，低电平使能
	digitalWrite(csnPin,LOW);
}

void Nrf24l::powerDown(){
// 掉电模式
	ceLow();
	configRegister(CONFIG, mirf_CONFIG );
}
