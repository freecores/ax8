--
-- 90S2313 compatible microcontroller core
--
-- Version : 0220
--
-- Copyright (c) 2001-2002 Daniel Wallner (jesus@opencores.org)
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- Please report bugs to the author, but before you do so, please
-- make sure that this is not a derivative work and that
-- you have the latest version of this file.
--
-- The latest version of this file can be found at:
--	http://www.opencores.org/cvsweb.shtml/t51/
--
-- Limitations :
--
-- File history :
--

--Registers:												Comments:
--$3F SREG Status Register									Implemented in the AX8 core
--$3D SPL Stack Pointer Low									Implemented in the AX8 core
--$3B GIMSK General Interrupt Mask register
--$3A GIFR General Interrupt Flag Register
--$39 TIMSK Timer/Counter Interrupt Mask register
--$38 TIFR Timer/Counter Interrupt Flag register
--$35 MCUCR MCU General Control Register					No power down
--$33 TCCR0 Timer/Counter 0 Control Register
--$32 TCNT0 Timer/Counter 0 (8-bit)
--$2F TCCR1A Timer/Counter 1 Control Register A
--$2E TCCR1B Timer/Counter 1 Control Register B
--$2D TCNT1H Timer/Counter 1 High Byte
--$2C TCNT1L Timer/Counter 1 Low Byte
--$2B OCR1AH Output Compare Register 1 High Byte
--$2A OCR1AL Output Compare Register 1 Low Byte
--$25 ICR1H T/C 1 Input Capture Register High Byte
--$24 ICR1L T/C 1 Input Capture Register Low Byte
--$21 WDTCR Watchdog Timer Control Register					Not implemented
--$1E EEAR EEPROM Address Register							Not implemented
--$1D EEDR EEPROM Data Register								Not implemented
--$1C EECR EEPROM Control Register							Not implemented
--$18 PORTB Data Register, Port B							No pullup
--$17 DDRB Data Direction Register, Port B
--$16 PINB Input Pins, Port B
--$12 PORTD Data Register, Port D							No pullup
--$11 DDRD Data Direction Register, Port D
--$10 PIND Input Pins, Port D
--$0C UDR UART I/O Data Register
--$0B USR UART Status Register
--$0A UCR UART Control Register
--$09 UBRR UART Baud Rate Register
--$08 ACSR Analog Comparator Control and Status Register	Not implemented

library IEEE;
use IEEE.std_logic_1164.all;
use work.AX_Pack.all;

entity A90S2313 is
	port(
		Clk		: in std_logic;
		Reset_n	: in std_logic;
		INT0	: in std_logic;
		INT1	: in std_logic;
		T0		: in std_logic;
		T1		: in std_logic;
		ICP		: in std_logic;
		RXD		: in std_logic;
		TXD		: out std_logic;
		OC		: out std_logic;
		Port_B	: inout std_logic_vector(7 downto 0);
		Port_D	: inout std_logic_vector(7 downto 0)
	);
end A90S2313;

architecture rtl of A90S2313 is

	constant	ROMAddressWidth		: integer := 10;
	constant	RAMAddressWidth		: integer := 7;
	constant	BigISet				: boolean := true;

	component ROM2313
		port(
			Clk	: in std_logic;
			A	: in std_logic_vector(ROMAddressWidth - 1 downto 0);
			D	: out std_logic_vector(15 downto 0)
		);
	end component;

	signal	Reset_s_n	: std_logic;
	signal	ROM_Addr	: std_logic_vector(ROMAddressWidth - 1 downto 0);
	signal	ROM_Data	: std_logic_vector(15 downto 0);
	signal	IO_Rd		: std_logic;
	signal	IO_Wr		: std_logic;
	signal	IO_Addr		: std_logic_vector(5 downto 0);
	signal	IO_WData	: std_logic_vector(7 downto 0);
	signal	IO_RData	: std_logic_vector(7 downto 0);
	signal	TCCR0_Sel	: std_logic;
	signal	TCNT0_Sel	: std_logic;
	signal	TCCR1_Sel	: std_logic;
	signal	TCNT1_Sel	: std_logic;
	signal	OCR1_Sel	: std_logic;
	signal	ICR1_Sel	: std_logic;
	signal	UDR_Sel		: std_logic;
	signal	USR_Sel		: std_logic;
	signal	UCR_Sel		: std_logic;
	signal	UBRR_Sel	: std_logic;
	signal	PORTB_Sel	: std_logic;
	signal	DDRB_Sel	: std_logic;
	signal	PINB_Sel	: std_logic;
	signal	PORTD_Sel	: std_logic;
	signal	DDRD_Sel	: std_logic;
	signal	PIND_Sel	: std_logic;
	signal	Sleep_En	: std_logic;
	signal	ISC0		: std_logic_vector(1 downto 0);
	signal	ISC1		: std_logic_vector(1 downto 0);
	signal	Int_ET		: std_logic_vector(1 downto 0);
	signal	Int_En		: std_logic_vector(1 downto 0);
	signal	Int0_r		: std_logic_vector(1 downto 0);
	signal	Int1_r		: std_logic_vector(1 downto 0);
	signal	TC_Trig		: std_logic;
	signal	TO_Trig		: std_logic;
	signal	OC_Trig		: std_logic;
	signal	IC_Trig		: std_logic;
	signal	TOIE0		: std_logic;
	signal	TICIE1		: std_logic;
	signal	OCIE1		: std_logic;
	signal	TOIE1		: std_logic;
	signal	TOV0		: std_logic;
	signal	ICF1		: std_logic;
	signal	OCF1		: std_logic;
	signal	TOV1		: std_logic;
	signal	Int_Trig	: std_logic_vector(15 downto 1);
	signal	Int_Acc		: std_logic_vector(15 downto 1);

begin

	-- Synchronise reset
	process (Reset_n, Clk)
	begin
		if Reset_n = '0' then
			Reset_s_n <= '0';
		elsif Clk'event and Clk = '1' then
			Reset_s_n <= '1';
		end if;
	end process;

	-- Registers/Interrupts
	IO_RData <= "00" & Sleep_En & "0" & ISC1 & ISC0 when IO_Rd = '1' and IO_Addr = "110101" else "ZZZZZZZZ"; -- $35 MCUCR
	IO_RData <= Int_En & "000000" when IO_Rd = '1' and IO_Addr = "111011" else "ZZZZZZZZ"; -- $3B GIMSK
	IO_RData <= TOIE1 & OCIE1 & "00" & TICIE1 & "0" & TOIE0 & "0" when IO_Rd = '1' and IO_Addr = "111001" else "ZZZZZZZZ"; -- $39 TIMSK
	IO_RData <= TOV1 & OCF1 & "00" & ICF1 & "0" & TOV0 & "0" when IO_Rd = '1' and IO_Addr = "111000" else "ZZZZZZZZ"; -- $38 TIFR
	process (Reset_n, Clk)
	begin
		if Reset_n = '0' then
			Sleep_En <= '0';
			ISC0 <= "00";
			ISC1 <= "00";
			Int_ET <= "00";
			Int_En <= "00";
			Int0_r <= "11";
			Int1_r <= "11";
			TOIE0 <= '0';
			TICIE1 <= '0';
			OCIE1 <= '0';
			TOIE1 <= '0';
			TOV0 <= '0';
			ICF1 <= '0';
			OCF1 <= '0';
			TOV1 <= '0';
		elsif Clk'event and Clk = '1' then
			Int0_r(0) <= INT0;
			Int0_r(1) <= Int0_r(0);
			Int1_r(0) <= INT1;
			Int1_r(1) <= Int1_r(0);
			if IO_Wr = '1' and IO_Addr = "110101" then	-- $35 MCUCR
				Sleep_En <= IO_WData(5);
				ISC0 <= IO_WData(1 downto 0);
				ISC1 <= IO_WData(3 downto 2);
			end if;
			if IO_Wr = '1' and IO_Addr = "111011" then	-- $3B GIMSK
				Int_En <= IO_WData(7 downto 6);
			end if;
			if IO_Wr = '1' and IO_Addr = "111001" then	-- $39 TIMSK
				TOIE0 <= IO_WData(1);
				TICIE1 <= IO_WData(3);
				OCIE1 <= IO_WData(6);
				TOIE1 <= IO_WData(7);
			end if;
			if IO_Wr = '1' and IO_Addr = "111000" then	-- $38 TIFR
				if IO_WData(1) = '1' then
					TOV0 <= '0';
				end if;
				if IO_WData(3) = '1' then
					ICF1 <= '0';
				end if;
				if IO_WData(6) = '1' then
					OCF1 <= '0';
				end if;
				if IO_WData(7) = '1' then
					TOV1 <= '0';
				end if;
			end if;
			if Int_Acc(3) = '1' then
				ICF1 <= '0';
			end if;
			if Int_Acc(4) = '1' then
				OCF1 <= '0';
			end if;
			if Int_Acc(5) = '1' then
				TOV1 <= '0';
			end if;
			if Int_Acc(6) = '1' then
				TOV0 <= '0';
			end if;
			if TC_Trig = '1' then
				TOV0 <= '1';
			end if;
			if IC_Trig = '1' then
				ICF1 <= '1';
			end if;
			if OC_Trig = '1' then
				OCF1 <= '1';
			end if;
			if TO_Trig = '1' then
				TOV1 <= '1';
			end if;
			if Int_Acc(1) = '1' then
				Int_ET(0) <= '0';
			end if;
			if (ISC0 = "10" and Int0_r = "10") or (ISC0 = "11" and Int0_r = "01") then
				Int_ET(0) <= '1';
			end if;
			if Int_Acc(2) = '1' then
				Int_ET(1) <= '0';
			end if;
			if (ISC1 = "10" and Int1_r = "10") or (ISC1 = "11" and Int1_r = "01") then
				Int_ET(1) <= '1';
			end if;
		end if;
	end process;

	Int_Trig(1) <= '0' when Int_En(0) = '0' else not Int0_r(1) when ISC0 = "00" else Int_ET(0);
	Int_Trig(2) <= '0' when Int_En(1) = '0' else not Int1_r(1) when ISC1 = "00" else Int_ET(1);
	Int_Trig(3) <= '1' when TICIE1 = '1' and ICF1 = '0' else '0';
	Int_Trig(4) <= '1' when OCIE1 = '1' and OCF1 = '0' else '0';
	Int_Trig(5) <= '1' when TOIE1 = '1' and TOV1 = '0' else '0';
	Int_Trig(6) <= '1' when TOIE0 = '1' and TOV0 = '0' else '0';
	Int_Trig(15 downto 10) <= (others => '0');

	rom : ROM2313 port map(
			Clk => Clk,
			A => ROM_Addr,
			D => ROM_Data);

	ax : AX8
		generic map(
			ROMAddressWidth => ROMAddressWidth,
			RAMAddressWidth => RAMAddressWidth,
			BigIset => BigIset)
		port map(
			Clk => Clk,
			Reset_n => Reset_s_n,
			ROM_Addr => ROM_Addr,
			ROM_Data => ROM_Data,
			Sleep_En => Sleep_En,
			Int_Trig => Int_Trig,
			Int_Acc => Int_Acc,
			IO_Rd => IO_Rd,
			IO_Wr => IO_Wr,
			IO_Addr => IO_Addr,
			IO_WData => IO_WData,
			IO_RData => IO_RData);

	TCCR0_Sel <= '1' when IO_Addr = "110011" else '0';	-- $33 TCCR0
	TCNT0_Sel <= '1' when IO_Addr = "110010" else '0';	-- $32 TCNT0
	tc0 : AX_TC8 port map(
			Clk => Clk,
			Reset_n => Reset_n,
			T => T0,
			TCCR_Sel => TCCR0_Sel,
			TCNT_Sel => TCNT0_Sel,
			Rd => IO_Rd,
			Wr => IO_Wr,
			Data_In => IO_WData,
			Data_Out => IO_RData,
			Int  => TC_Trig);

	TCCR1_Sel <= '1' when IO_Addr(5 downto 1) = "10111" else '0';	-- $2E TCCR1
	TCNT1_Sel <= '1' when IO_Addr(5 downto 1) = "10110" else '0';	-- $2C TCNT1
	OCR1_Sel <= '1' when IO_Addr(5 downto 1) = "10101" else '0';	-- $2A OCR1
	ICR1_Sel <= '1' when IO_Addr(5 downto 1) = "10100" else '0';	-- $24 ICR1
	tc1 : AX_TC16 port map(
			Clk => Clk,
			Reset_n => Reset_n,
			T => T1,
			ICP => ICP,
			TCCR_Sel => TCCR1_Sel,
			TCNT_Sel => TCNT1_Sel,
			OCR_Sel => OCR1_Sel,
			ICR_Sel => ICR1_Sel,
			A0 => IO_Addr(0),
			Rd => IO_Rd,
			Wr => IO_Wr,
			Data_In => IO_WData,
			Data_Out => IO_RData,
			OC => OC,
			Int_TO => TO_Trig,
			Int_OC => OC_Trig,
			Int_IC => IC_Trig);

	UDR_Sel <= '1' when IO_Addr = "001100" else '0';
	USR_Sel <= '1' when IO_Addr = "001011" else '0';
	UCR_Sel <= '1' when IO_Addr = "001010" else '0';
	UBRR_Sel <= '1' when IO_Addr = "001001" else '0';
	uart : AX_UART port map(
			Clk => Clk,
			Reset_n => Reset_s_n,
			UDR_Sel => UDR_Sel,
			USR_Sel => USR_Sel,
			UCR_Sel => UCR_Sel,
			UBRR_Sel => UBRR_Sel,
			Rd => IO_Rd,
			Wr => IO_Wr,
			TXC_Clr => Int_Acc(9),
			Data_In => IO_WData,
			Data_Out => IO_RData,
			RXD => RXD,
			TXD => TXD,
			Int_RX => Int_Trig(7),
			Int_TR => Int_Trig(8),
			Int_TC => Int_Trig(9));

	PINB_Sel <= '1' when IO_Addr = "010101" else '0';
	DDRB_Sel <= '1' when IO_Addr = "010111" else '0';
	PORTB_Sel <= '1' when IO_Addr = "011000" else '0';
	PIND_Sel <= '1' when IO_Addr = "010000" else '0';
	DDRD_Sel <= '1' when IO_Addr = "010001" else '0';
	PORTD_Sel <= '1' when IO_Addr = "010010" else '0';
	porta : AX_Port port map(
			Clk => Clk,
			Reset_n => Reset_n,
			PORT_Sel => PORTB_Sel,
			DDR_Sel => DDRB_Sel,
			PIN_Sel => PINB_Sel,
			Rd => IO_Rd,
			Wr => IO_Wr,
			Data_In => IO_WData,
			Data_Out => IO_RData,
			IOPort  => Port_B);
	portb : AX_Port port map(
			Clk => Clk,
			Reset_n => Reset_n,
			PORT_Sel => PORTD_Sel,
			DDR_Sel => DDRD_Sel,
			PIN_Sel => PIND_Sel,
			Rd => IO_Rd,
			Wr => IO_Wr,
			Data_In => IO_WData,
			Data_Out => IO_RData,
			IOPort  => Port_D);

end;
