/*
  1. Filename: SPI
 
 2. Version number: 0.1
 
 3. Creation date
 Feb 2010
 4. Last modification date
 March 16th, 2010
 - Split out source into functions
 5. Author's name
 Terry Kolody
 
 6. Copyright notice
    This file is part of g8MediaNav.

    Foobar is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    g8MediaNav is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with g8MediaNav.  If not, see <http://www.gnu.org/licenses/>.
    
 7. Purpose of the program
 SPI Interfacing
 
 8. Change history
 March 16th, 2010
 - Split out source into functions
 
 9. Dependencies
 
 10.Special hardware requirements (e.g. A/D converters)
 
 */


void initSPI()
{
  byte clr; 
  // Configure SPI
  // SPCR = 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate (fastest)
  SPCR = (1<<SPE)|(1<<MSTR);
  clr=SPSR;
  clr=SPDR;
  delay(10); 

}


//-----------------------------------
//Write-Read BYTE VIA SPI
//-----------------------------------
byte SPI_ReadWrite( byte data )
{
  SPDR = data;

  // Esperar hasta final de transmisión
  while( !( SPSR & (1<<SPIF) ) );

  return SPDR;
}

