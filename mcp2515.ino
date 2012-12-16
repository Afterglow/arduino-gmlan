/*
  1. Filename: MCP2515
 
 2. Version number: 0.1
 
 3. Creation date
 Feb 2010
 4. Last modification date
 December 16th, 2012
 - Split out source into functions
 5. Author's name
 Terry Kolody 
 Last modified by Paul Thomas
 
 6. Copyright notice
    This file is part of g8MediaNav.

    g8MediaNav is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    g8MediaNav is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
    
 7. Purpose of the program
 MCP2515 control and initialization
 
 8. Change history
 March 16th, 2010
 - Split out source into functions
 March 19th, 2011
 - Added GPL boilerplate
 December 12th, 2012
 - Rewrite send functions to accept variable length array of data to send
 
 9. Dependencies
 
 10.Special hardware requirements (e.g. A/D converters)     
 
 11. References
 Ejemplo transmisión trama CAN 2.0A
 Basado en el trabajo del Tutorial MCP2515 http://www.kreatives-chaos.com/
 Igor Real

 */

//-----------------------------------
//Write Register in MCP2515
//-----------------------------------
void mcp2515_write_register(uint8_t address,uint8_t data)
{
  digitalWrite(P_CS,LOW);
  SPI_ReadWrite(0x02);    //Instrucción de WRITE
  SPI_ReadWrite(address);
  SPI_ReadWrite(data);
  digitalWrite(P_CS,HIGH);  
}

//-----------------------------------
//Read Register in MCP2515
//-----------------------------------
uint8_t mcp2515_read_register(uint8_t address)
{
  uint8_t data;

  digitalWrite(P_CS,LOW);
  SPI_ReadWrite(0x03);     //READ Instruction
  SPI_ReadWrite(address);
  data=SPI_ReadWrite(0xff);
  digitalWrite(P_CS,HIGH);

  return data;
}

//-----------------------------------
//Modify Register in MCP2515
//-----------------------------------
void mcp2515_modifyRegister(const uint8_t address, 
const uint8_t mask, const uint8_t data)
{
  digitalWrite(P_CS,LOW);
  SPI_ReadWrite(0x05); // MCP_BITMOD;
  SPI_ReadWrite(address);
  SPI_ReadWrite(mask);
  SPI_ReadWrite(data);
  digitalWrite(P_CS,HIGH);
}

//-----------------------------------
//Initialize MCP2515
//-----------------------------------
void mcp2515_init(uint16_t speed, byte initFilter)
{
  digitalWrite(P_CS,LOW);
  SPI_ReadWrite(0xC0);     //Reset Instruction  
  digitalWrite(P_CS,HIGH);
  delay(10);  
  //-------------------------------------------------
  //CONFIGURAR BIT TIME
  //Fosc=16 MHz significa Tosc=62.5 nsec
  //Eligiendo BRP=1
  //BRP=1     PRESCALER
  //Tq=2*(BPR+1)/Fosc=250 nsec
  //Si velocidad=500 Khz, Veces de Tq=1/(500000*250e-9)=> El bit time tiene que ser 8*Tq
  //-------------------------------------------------
  //Usar el software Microchip CAN Bit Timing Calculator
  //-------------------------------------------------
  //Requerimientos:
  //Prop Seg + Phase Seg1 >=Phase Seg2  => 1+3 >= 3
  //Prop Seg + Phase Seg1 >=Tdelay      => 1+3 >= 2
  //Phase Seg2 > Sync Jump Width        => 3   > SJW  
  //Por lo que teniendo en cuenta los requerimientos
  //Sync Seg                  =1 Tq
  //Prop Seg=(PRSEG+1)*Tq     =1 Tq
  //Phase Seg1=(PHSEG1+1)*Tq  =3 Tq significa PHSEG1=2 (010b)
  //Phase Seg2=(PHSEG2+1)*Tq  =3 Tq significa PHSEG2=2 (010b)
  //t bit=1+1+3+3=8 <-OK
  //SJW=1
  //Por lo que para 500Kbits/s@16Mhz:
  //CNF1=0x01h(00000001b),CNF2=0x90h(10010000b),CNF3=0x02h(00000010b)
  //-------------------------------------------------
  //Para 1Mbit/s@16Mhz:
  //CNF1=0x00h(00000000b),CNF2=0x90h(10010000b),CNF3=0x02h(00000010b)
  //Para 125Kbit/s@16Mhz:
  //CNF1=0x07h(00000111b),CNF2=0x90h(10010000b),CNF3=0x02h(00000010b)  

  switch(speed){
  case 1:
    mcp2515_write_register(CNF1,0x00);
    mcp2515_write_register(CNF2,0x90);
    mcp2515_write_register(CNF3,0x02);
    //mySerial.println("Speed 1mbps");
    break;
  case 500:
    mcp2515_write_register(CNF1,0x01);
    mcp2515_write_register(CNF2,0x90);
    mcp2515_write_register(CNF3,0x02);
    //mySerial.println("Speed 500kbps");
    break;
  case 125:
    mcp2515_write_register(CNF1,0x07);
    mcp2515_write_register(CNF2,0x90);
    mcp2515_write_register(CNF3,0x02);
    //mySerial.println("Speed 125kbps");
    break;
  case 33:
    mcp2515_write_register(CNF1,0x09);
    mcp2515_write_register(CNF2,0xBE);
    mcp2515_write_register(CNF3,0x07);
 //   Serial.println("Speed 33kbps");
    break;      
  }

  //Activamos Interrupcion de RX
  mcp2515_write_register(CANINTE,(1<<1)|(1<<0)); //RX1IE and RX0IE (los dos buffers)
  //Filtros
  //Bufer 0: Todos los msjes
  mcp2515_write_register(RXB0CTRL,(1<<6)|(1<<5)|(1<<2)); //RXM1 and RXM0 para filter/mask off and rollover enabled
  //Bufer 1: Todos los msjes
  mcp2515_write_register(RXB1CTRL,(1<<6)|(1<<5)); //RXM1 and RXM0 para filter/mask off
  //Borrar bits de máscara de recepción
  mcp2515_write_register( RXM0SIDH, 0 );
  mcp2515_write_register( RXM0SIDL, 0 );
  mcp2515_write_register( RXM0EID8, 0 );
  mcp2515_write_register( RXM0EID0, 0 );
  mcp2515_write_register( RXM1SIDH, 0 );
  mcp2515_write_register( RXM1SIDL, 0 );
  mcp2515_write_register( RXM1EID8, 0 );
  mcp2515_write_register( RXM1EID0, 0 );

  if ( initFilter == 0x01 ) {
    mcp2515_initFilters();
  }
  
  //Set MCP2515 into normal. 
  // Or into loopback 
  //if (DEBUG) {
  mcp2515_write_register(CANCTRL, 0x40);
  Serial.println("Debug Enabled..");
  //} 
  //else {
  //mySerial.println("Normal..");     
  // mcp2515_write_register(CANCTRL, 0x00);
  //}

  delay(10);
}

void mcp2515_initFilters()
{
//  Serial.println("Init CAN Filters"); 
  // Enable filters. 
  mcp2515_write_register(RXB0CTRL,(1<<6)|(0<<5)|(1<<2)); //RXM1 and RXM0 para filter/mask on for extented IDs and rollover enabled
  //Bufer 1: Todos los msjes
  mcp2515_write_register(RXB1CTRL,(1<<6)|(0<<5)); //RXM1 and RXM0 para filter/mask off

  // Filter on the important ARBIDs. 
  // mcp2515_write_ext_can_id(RXF0SIDH, WHEEL_CONTROL);
  // mcp2515_write_ext_can_id(RXF2SIDH, RADIO_DIC);
  // mcp2515_write_ext_can_id(RXF3SIDH, RADIO_MODECHANGE);

  // Set Mask.. 100% match
  mcp2515_write_ext_can_id(RXM0SIDH, 0x1FFFFFFF);
  mcp2515_write_ext_can_id(RXM1SIDH, 0x1FFFFFFF);

}

//-----------------------------------
//Send Standard 11 bit Packet
//-----------------------------------
// byte D0,byte D1,byte D2,byte D3,byte D4,byte D5,byte D6,byte D7)
byte can_send_11bit_message(uint16_t id, int length, byte packetdata[])
{
  //See that buffer is free
  byte status;
  digitalWrite(P_CS,LOW);
  SPI_ReadWrite(0xA0);    //Read Status
  status=SPI_ReadWrite(0xFF);
  digitalWrite(P_CS,HIGH);
  //Read Status devuelve
  //Bit0.- CANINTF.RX0IF
  //BIT1.- CANINTF.RX1IF
  //BIT2.- TXB0CNTRL.TXREQ
  //BIT3.- CANINTF.TX0IF
  //BIT4.- TXB1CNTRL.TXREQ
  //BIT5.- CANINTF.TX1IF
  //BIT6.- TXB2CNTRL.TXREQ
  //BIT7.- CANINTF.TX2IF

  //Search for a free buffer
  byte buffer_free;
  byte address;
  if (bit_is_clear(status,2)){
    buffer_free=1;
    address=0x31;    //Registro Standard ID buffer 0
  }else if (bit_is_clear(status,4)){
    buffer_free=2;
    address=0x41;    //Registro Standard ID buffer 1
  }else if (bit_is_clear(status,6)){
    buffer_free=4;
    address=0x51;    //Registro Standard ID buffer 2
  }else{
    return 0;    //No hay buffer libre, no se ha transmitido msje  
  }

  //Set priorities on sending buffers
  //Is independent of the priorities of the bus can => (greater priority = lowest id)
  //It always puts the current buffer (final Msg) with the lowest priority
  
  switch (address){
    case 0x31:
      mcp2515_write_register(0x30,(0<<1)|(0<<0));  //Minor Priority
      mcp2515_write_register(0x40,(0<<1)|(1<<0));
      mcp2515_write_register(0x50,(1<<1)|(0<<0));  //Major Priotity
      break;

    case 0x41:
      mcp2515_write_register(0x30,(0<<1)|(1<<0));  
      mcp2515_write_register(0x40,(0<<1)|(0<<0));  //Minor Priority
      mcp2515_write_register(0x50,(1<<1)|(0<<0));  //Major Priotity
      break;

    case 0x51:
      mcp2515_write_register(0x30,(1<<1)|(0<<0));  //Major Priotity  
      mcp2515_write_register(0x40,(0<<1)|(1<<0));  
      mcp2515_write_register(0x50,(0<<1)|(0<<0));  //Minor Priority
      break;    

  }

  // Configurar ID
  mcp2515_write_register(address, (uint8_t) (id>>3));
  mcp2515_write_register(address+1, (uint8_t) (id<<5));
  mcp2515_write_register(address+2, 0x00);
  mcp2515_write_register(address+3, 0x00);
    
  // Configurar el largo del dato(8bytes) + RTR=0.
  mcp2515_write_register(address+4, length);     //Registro TXBnDLC
  // Bytes de Datos -> Registro TXBnDm
  int c = 0;
  while (c < length) {    
    mcp2515_write_register(address+c+5,packetdata[c]);
    Serial.print("Wrote byte ");
    Serial.print(packetdata[c]);
    Serial.print(" to address ");
    Serial.println(address);
    c++;
  }

  // Enviar mensaje CAN
  digitalWrite(P_CS,LOW);
  SPI_ReadWrite(0x80 | buffer_free);  //RTS(Message Request To Send)
  digitalWrite(P_CS,HIGH);
  return 1;
}

//-----------------------------------
//Send 29 bit message
//-----------------------------------
//byte can_send_29bit_message(uint32_t id, byte D0, byte D1 ,byte D2, byte D3,byte D4,byte D5,byte D6,byte D7)
byte can_send_29bit_message(uint32_t id, int length, byte packetdata[])
{
  // Truncate id just in case.. 
  id = id & EXMASK;   
  //  Serial.print("Header: ");
  //  Serial.println(id, HEX);
  //  

  //Miramos que buffer esta libre
  byte status;
  digitalWrite(P_CS,LOW);
  SPI_ReadWrite(0xA0);    //Read Status
  status=SPI_ReadWrite(0xFF);
  digitalWrite(P_CS,HIGH);
  //Read Status devuelve
  //Bit0.- CANINTF.RX0IF
  //BIT1.- CANINTF.RX1IF
  //BIT2.- TXB0CNTRL.TXREQ
  //BIT3.- CANINTF.TX0IF
  //BIT4.- TXB1CNTRL.TXREQ
  //BIT5.- CANINTF.TX1IF
  //BIT6.- TXB2CNTRL.TXREQ
  //BIT7.- CANINTF.TX2IF

  //Busco un buffer libre
  byte buffer_free;
  byte address;
  if (bit_is_clear(status,2)){
    buffer_free=1;
    address=0x31;    //Registro Standard ID buffer 0
  }
  else if (bit_is_clear(status,4)){
    buffer_free=2;
    address=0x41;    //Registro Standard ID buffer 1
  }
  else if (bit_is_clear(status,6)){
    buffer_free=4;
    address=0x51;    //Registro Standard ID buffer 2
  }
  else{
    return 0;    //No hay buffer libre, no se ha transmitido msje  
  }

  //Configuración de las prioridades de envio de buffers
  //Es independiente de las prioridades definidas del bus CAN =>(Mayor prioridad=Menor ID)
  //Se debe poner siempre el buffer actual(último msje)con la prioridad menor
  switch (address){
  case 0x31:
    mcp2515_write_register(0x30,(0<<1)|(0<<0));  //Menor prioridad
    mcp2515_write_register(0x40,(0<<1)|(1<<0));
    mcp2515_write_register(0x50,(1<<1)|(0<<0));  //Mayor prioridad
//    Serial.println("Buffer 0");
    break;
  case 0x41:
    mcp2515_write_register(0x30,(0<<1)|(1<<0));  
    mcp2515_write_register(0x40,(0<<1)|(0<<0));  //Menor prioridad
    mcp2515_write_register(0x50,(1<<1)|(0<<0));  //Mayor prioridad
//    Serial.println("Buffer 1");
    break;
  case 0x51:
    mcp2515_write_register(0x30,(1<<1)|(0<<0));  //Mayor prioridad  
    mcp2515_write_register(0x40,(0<<1)|(1<<0));  
    mcp2515_write_register(0x50,(0<<1)|(0<<0));  //Menor prioridad
//    Serial.println("Buffer 2");
    break;    
  }
  // write ID


  mcp2515_write_ext_can_id(address, id);

  // Configurar el largo del dato(8bytes) + RTR=0.
  mcp2515_write_register(address+4, length);     //Registro TXBnDLC
  // Bytes de Datos -> Registro TXBnDm
  int c = 0;
  while (c < length) {
    mcp2515_write_register(address+c+5,packetdata[c]);
    c++;
  }
  
  // Enviar mensaje CAN
  digitalWrite(P_CS,LOW);
  SPI_ReadWrite(0x80 | buffer_free);  //RTS(Message Request To Send)
  digitalWrite(P_CS,HIGH);
  return 1;
}

void mcp2515_read_can_id( const uint8_t mcp_addr, 
uint8_t* ext, uint32_t* can_id )
{
  uint8_t tbufdata[4];

  *ext = 0;
  *can_id = 0;

  tbufdata[0] = mcp2515_read_register(mcp_addr+MCP_SIDH);
  tbufdata[1] = mcp2515_read_register(mcp_addr+MCP_SIDL);
  tbufdata[2] = mcp2515_read_register(mcp_addr+MCP_EID8);
  tbufdata[3] = mcp2515_read_register(mcp_addr+MCP_EID0);
// if extended frame can_id is SID[10:0]EID[17:0]
  if ( (tbufdata[MCP_SIDL] & EXIDE) ==  EXIDE ) {

    *can_id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);
    *can_id <<= 18;
    *can_id |= ((uint32_t) (tbufdata[MCP_SIDL] & 0x03) << 16);
    *can_id |= (uint16_t) tbufdata[2] << 8;        
    *can_id |= tbufdata[3];


  }
// if not, can_id is SID[10:0] 
  else {

    *can_id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);
  }



}

void mcp2515_write_ext_can_id( const uint8_t mcp_addr, 
uint32_t can_id )
{
  mcp2515_write_register(mcp_addr, (uint8_t) (can_id>>21));
  mcp2515_write_register(mcp_addr+1, (((uint8_t) (can_id>>13)) & 0xE0) | EXIDE | (((uint8_t) (can_id >> 16)) & 0x03) );
  mcp2515_write_register(mcp_addr+2, (uint8_t) (can_id>>8));
  mcp2515_write_register(mcp_addr+3, (uint8_t) (can_id));
}

// Buffer can be MCP_RXBUF_0 or MCP_RXBUF_1
void mcp2515_read_canMsg( const uint8_t mcp_addr,
uint8_t* dlc,
uint8_t* msg)
{

  // Deal with remote Frame shit later.. 

  uint8_t i;

  *dlc = mcp2515_read_register( mcp_addr+4 );
  *dlc &= MCP_DLC_MASK;

  for (i=0; i < 8; i++ ) {
    *msg++  = mcp2515_read_register( mcp_addr + 5 + i );
  }

}



