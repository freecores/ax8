--
-- 90S1200 compatible microcontroller core
--
-- Version : 0220b
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
--$3B GIMSK General Interrupt Mask register
--$39 TIMSK Timer/Counter Interrupt Mask register
--$38 TIFR Timer/Counter Interrupt Flag register
--$35 MCUCR MCU general Control Register					No power down
--$33 TCCR0 Timer/Counter 0 Control Register
--$32 TCNT0 Timer/Counter 0 (8-bit)
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
--$08 ACSR Analog Comparator Control and Status Register	Not implemented

library IEEE;
use IEEE.std_logic_1164.all;
use work.AX_Pack.all;

entity A90S1200 is
	generic(
		SyncReset : boolean := true
	);
	port(
		Clk		: in std_logic;
		Reset_n	: in std_logic;
		INT0	: in std_logic;
		T0		: in std_logic;
		Port_B	: inout std_logic_vector(7 downto 0);
		Port_D	: inout std_logic_vector(7 downto 0)
	);
end A90S1200;

architecture rtl of A90S1200 is

	constant	ROMAddressWidth		: integer := 9;
	constant	RAMAddressWidth		: integer := 0;
	constant	BigISet				: boolean := false;

	component ROM1200
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
	signal	TCCR_Sel	: std_logic;
	signal	TCNT_Sel	: std_logic;
	signal	PORTB_Sel	: std_logic;
	signal	DDRB_Sel	: std_logic;
	signal	PINB_Sel	: std_logic;
	signal	PORTD_Sel	: std_logic;
	signal	DDRD_Sel	: std_logic;
	signal	PIND_Sel	: std_logic;
	signal	Sleep_En	: std_logic;
	signal	ISC0		: std_logic_vector(1 downto 0);
	signal	Int0_ET		: std_logic;
	signal	Int0_En		: std_logic;
	signal	Int0_r		: std_logic_vector(1 downto 0);
	signal	TC_Trig		: std_logic;
	signal	TOIE0		: std_logic;
	signal	TOV0		: std_logic;
	signal	Int_Trig	: std_logic_vector(15 downto 1);
	signal	Int_Acc		: std_logic_vector(15 downto 1);

begin

	-- Synchronise reset
	process (Reset_n, Clk)
		variable Reset_v : std_logic;
	begin
		if Reset_n = '0' then
			if SyncReset then
				Reset_s_n <= '0';
				Reset_v := '0';
			end if;
		elsif Clk'event and Clk = '1' then
			if SyncReset then
				Reset_s_n <= Reset_v;
				Reset_v := '1';
			end if;
		end if;
	end process;

	g_reset : if not SyncReset generate
		Reset_s_n <= Reset_n;
	end generate;

	-- Registers/Interrupts
	IO_RData <= "00" & Sleep_En & "000" & ISC0 when IO_Rd = '1' and IO_Addr = "110101" else "ZZZZZZZZ";	-- $35 MCUCR
	IO_RData <= "0" & Int0_En & "000000" when IO_Rd = '1' and IO_Addr = "111011" else "ZZZZZZZZ";		-- $3B GIMSK
	IO_RData <= "000000" & TOIE0 & "0" when IO_Rd = '1' and IO_Addr = "111001" else "ZZZZZZZZ";		-- $39 TIMSK
	IO_RData <= "000000" & TOV0 & "0" when IO_Rd = '1' and IO_Addr = "111000" else "ZZZZZZZZ";		-- $38 TIFR
	process (Reset_s_n, Clk)
	begin
		if Reset_s_n = '0' then
			Sleep_En <= '0';
			ISC0 <= "00";
			Int0_ET <= '0';
			Int0_En <= '0';
			Int0_r <= "11";
			TOIE0 <= '0';
			TOV0 <= '0';
		elsif Clk'event and Clk = '1' then
			Int0_r(0) <= INT0;
			Int0_r(1) <= Int0_r(0);
			if IO_Wr = '1' and IO_Addr = "110101" then	-- $35 MCUCR
				Sleep_En <= IO_WData(5);
				ISC0 <= IO_WData(1 downto 0);
			end if;
			if IO_Wr = '1' and IO_Addr = "111011" then	-- $3B GIMSK
				Int0_En <= IO_WData(6);
			end if;
			if IO_Wr = '1' and IO_Addr = "111001" then	-- $39 TIMSK
				TOIE0 <= IO_WData(1);
			end if;
			if IO_Wr = '1' and IO_Addr = "111000" then	-- $38 TIFR
				if IO_WData(1) = '1' then
					TOV0 <= '0';
				end if;
			end if;
			if Int_Acc(2) = '1' then
				TOV0 <= '0';
			end if;
			if TC_Trig = '1' then
				TOV0 <= '1';
			end if;
			if Int_Acc(1) = '1' then
				Int0_ET <= '0';
			end if;
			if (ISC0 = "10" and Int0_r = "10") or (ISC0 = "11" and Int0_r = "01") then
				Int0_ET <= '1';
			end if;
		end if;
	end process;

	Int_Trig(1) <= '0' when Int0_En = '0' else not Int0_r(1) when ISC0 = "00" else Int0_ET;
	Int_Trig(2) <= '1' when TOIE0 = '1' and TOV0 = '0' else '0';
	Int_Trig(15 downto 3) <= (others => '0');

	rom : ROM1200 port map (
			Clk => Clk,
			A => ROM_Addr,
			D => ROM_Data);

	ax : AX8
		generic map(
			ROMAddressWidth => ROMAddressWidth,
			RAMAddressWidth => RAMAddressWidth,
			BigIset => BigIset)
		port map (
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

	TCCR_Sel <= '1' when IO_Addr = "110011" else '0';	-- $33 TCCR0
	TCNT_Sel <= '1' when IO_Addr = "110010" else '0';	-- $32 TCNT0
	tc : AX_TC8 port map(
			Clk => Clk,
			Reset_n => Reset_s_n,
			T => T0,
			TCCR_Sel => TCCR_Sel,
			TCNT_Sel => TCNT_Sel,
			Rd => IO_Rd,
			Wr => IO_Wr,
			Data_In => IO_WData,
			Data_Out => IO_RData,
			Int  => TC_Trig);

	PINB_Sel <= '1' when IO_Addr = "010101" else '0';
	DDRB_Sel <= '1' when IO_Addr = "010111" else '0';
	PORTB_Sel <= '1' when IO_Addr = "011000" else '0';
	PIND_Sel <= '1' when IO_Addr = "010000" else '0';
	DDRD_Sel <= '1' when IO_Addr = "010001" else '0';
	PORTD_Sel <= '1' when IO_Addr = "010010" else '0';
	porta : AX_Port port map(
			Clk => Clk,
			Reset_n => Reset_s_n,
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
			Reset_n => Reset_s_n,
			PORT_Sel => PORTD_Sel,
			DDR_Sel => DDRD_Sel,
			PIN_Sel => PIND_Sel,
			Rd => IO_Rd,
			Wr => IO_Wr,
			Data_In => IO_WData,
			Data_Out => IO_RData,
			IOPort  => Port_D);

end;
