--
-- AT90Sxxxx compatible microcontroller core
--
-- Version : 0146
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
--	No OC disconnect (separate output pins)
--
-- File history :
--

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity AX_TC16 is
	port(
		Clk			: in std_logic;
		Reset_n		: in std_logic;
		T			: in std_logic;
		ICP			: in std_logic;
		TCCR_Sel	: in std_logic;
		TCNT_Sel	: in std_logic;
		OCR_Sel		: in std_logic;
		ICR_Sel		: in std_logic;
		A0			: in std_logic;
		Rd			: in std_logic;
		Wr			: in std_logic;
		Data_In		: in std_logic_vector(7 downto 0);
		Data_Out	: out std_logic_vector(7 downto 0);
		OC			: out std_logic;
		Int_TO		: out std_logic;
		Int_OC		: out std_logic;
		Int_IC		: out std_logic
	);
end AX_TC16;

architecture rtl of AX_TC16 is

	signal	COM			: std_logic_vector(1 downto 0);
	signal	PWM			: std_logic_vector(1 downto 0);
	signal	CRBH		: std_logic_vector(1 downto 0); -- ICNC, ICES
	signal	CRBL		: std_logic_vector(3 downto 0); -- CTC, CS
	signal	TCNT		: std_logic_vector(15 downto 0);
	signal	IC			: std_logic_vector(15 downto 0);
	signal	OCR			: std_logic_vector(15 downto 0);
	signal	TEMP		: std_logic_vector(15 downto 0);

	signal	OC_i		: std_logic;
	signal	IC_Trig		: std_logic;
	signal	Tick		: std_logic;
	signal	PWM_Dn		: std_logic;
	signal	PWM_Load	: std_logic;

begin

	OC <= OC_i;
	Int_IC <= IC_Trig;

	-- Registers and counter
	Data_Out <= COM & "0000" & PWM when Rd = '1' and TCCR_Sel = '1' and A0 = '1' else "ZZZZZZZZ";
	Data_Out <= CRBH & "00" & CRBL when Rd = '1' and TCCR_Sel = '1' and A0 = '0' else "ZZZZZZZZ";
	Data_Out <= TCNT(7 downto 0) when Rd = '1' and TCNT_Sel = '1' and A0 = '0' else "ZZZZZZZZ";
	Data_Out <= OCR(7 downto 0) when Rd = '1' and OCR_Sel = '1' and A0 = '0' and PWM_Load = '0' else "ZZZZZZZZ";
	Data_Out <= Temp(7 downto 0) when Rd = '1' and OCR_Sel = '1' and A0 = '0' and PWM_Load = '1' else "ZZZZZZZZ";
	Data_Out <= IC(7 downto 0) when Rd = '1' and ICR_Sel = '1' and A0 = '0' else "ZZZZZZZZ";
	Data_Out <= TEMP(15 downto 8) when Rd = '1' and (TCNT_Sel = '1' or ICR_Sel = '1' or OCR_Sel = '1') and A0 = '1' else "ZZZZZZZZ";
	process (Reset_n, Clk)
		variable PWM_T : std_logic;
		variable PWM_B : std_logic;
	begin
		if Reset_n = '0' then
			COM <= "00";
			PWM <= "00";
			CRBH <= "00";
			CRBL <= "0000";
			TCNT <= (others => '0');
			OCR <= (others => '0');
			IC <= (others => '0');
			Temp <= (others => '0');
			OC_i <= '0';
			Int_TO <= '0';
			Int_OC <= '0';
			PWM_Dn <= '0';
			PWM_Load <= '0';
		elsif Clk'event and Clk = '1' then
			Int_TO <= '0';
			Int_OC <= '0';
			if Tick = '1' then
				TCNT <= std_logic_vector(unsigned(TCNT) + 1);
				if TCNT = "1111111111111111" then
					Int_TO <= '1';
				end if;
				if TCNT = OCR then
					if PWM = "00" then
						Int_OC <= '1';
						if CRBL(3) = '1' then
							TCNT <= (others => '0');
						end if;
						if COM = "01" then
							OC_i <= not OC_i;
						end if;
						if COM = "10" then
							OC_i <= '0';
						end if;
						if COM = "11" then
							OC_i <= '1';
						end if;
					end if;
				end if;
				if PWM /= "00" then
					PWM_T := '0';
					PWM_B := '0';
					if PWM_Dn = '0' then
						TCNT <= std_logic_vector(unsigned(TCNT) + 1);
					else
						TCNT <= std_logic_vector(unsigned(TCNT) - 1);
					end if;
					if PWM = "01" and TCNT(7 downto 0) = OCR(7 downto 0) then
						OC_i <= COM(0) xor PWM_Dn;
					end if;
					if PWM = "10" and TCNT(8 downto 0) = OCR(8 downto 0) then
						OC_i <= COM(0) xor PWM_Dn;
					end if;
					if PWM = "11" and TCNT(9 downto 0) = OCR(9 downto 0) then
						OC_i <= COM(0) xor PWM_Dn;
					end if;
					if PWM = "01" then
						if TCNT(7 downto 0) = "11111110" and PWM_Dn = '0' then
							PWM_T := '1';
						end if;
						if TCNT(7 downto 0) = "00000001" and PWM_Dn = '1' then
							PWM_B := '1';
						end if;
					end if;
					if PWM = "10" then
						if TCNT(8 downto 0) = "111111110" and PWM_Dn = '0' then
							PWM_T := '1';
						end if;
						if TCNT(8 downto 0) = "000000001" and PWM_Dn = '1' then
							PWM_B := '1';
						end if;
					end if;
					if PWM = "11" then
						if TCNT(9 downto 0) = "1111111110" and PWM_Dn = '0' then
							PWM_T := '1';
						end if;
						if TCNT(9 downto 0) = "0000000001" and PWM_Dn = '1' then
							PWM_B := '1';
						end if;
					end if;
					if PWM_T = '1' then
						if PWM_Load = '1' and COM(0) = '0' then
							OCR <= Temp;
							PWM_Load <= '0';
						end if;
						PWM_Dn <= '1';
					end if;
					if PWM_B = '1' then
						if PWM_Load = '1' and COM(0) = '1' then
							OCR <= Temp;
							PWM_Load <= '0';
						end if;
						PWM_Dn <= '0';
						Int_TO <= '1';
					end if;
				end if;
			end if;
			if IC_Trig = '1' then
				TCNT <= IC;
			end if;
			-- Register read with temp
			if Rd = '1' and TCNT_Sel = '1' and A0 = '0' then
				Temp(15 downto 8) <= TCNT(15 downto 8);
			end if;
			if Rd = '1' and OCR_Sel = '1' and A0 = '0' then
				Temp(15 downto 8) <= OCR(15 downto 8);
			end if;
			if Rd = '1' and ICR_Sel = '1' and A0 = '0' then
				Temp(15 downto 8) <= IC(15 downto 8);
			end if;
			-- Register write
			if TCNT_Sel = '1' and Wr = '1' then
				if A0 = '1' then
					Temp(15 downto 8) <= Data_In;
				else
					TCNT(7 downto 0) <= Data_In;
					TCNT(15 downto 8) <= Temp(15 downto 8);
				end if;
				Int_TO <= '0';
			end if;
			if OCR_Sel = '1' and Wr = '1' then
				if A0 = '1' then
					Temp(15 downto 8) <= Data_In;
				else
					Temp(7 downto 0) <= Data_In;
					if PWM = "00" then
						OCR(7 downto 0) <= Data_In;
						OCR(15 downto 8) <= Temp(15 downto 8);
					else
						PWM_Load <= '1';
					end if;
				end if;
			end if;
			if ICR_Sel = '1' and Wr = '1' then
				if A0 = '1' then
					Temp(15 downto 8) <= Data_In;
				else
					IC(7 downto 0) <= Data_In;
					IC(15 downto 8) <= Temp(15 downto 8);
				end if;
			end if;
			if TCCR_Sel = '1' and Wr = '1' and A0 = '1' then
				COM <= Data_In(7 downto 6);
				PWM <= Data_In(1 downto 0);
			end if;
			if TCCR_Sel = '1' and Wr = '1' and A0 = '0' then
				CRBH <= Data_In(7 downto 6);
				CRBL <= Data_In(3 downto 0);
			end if;
		end if;
	end process;

	-- Input capture filter
	process (Clk)
		variable Samples : std_logic_vector(4 downto 0);
	begin
		if Clk'event and Clk = '1' then
			IC_Trig <= '0';
			if CRBH(1) = '1' then
				if Samples = "10000" and CRBH(0) = '0' then
					IC_Trig <= '1';
				end if;
				if Samples = "01111" and CRBH(0) = '1' then
					IC_Trig <= '1';
				end if;
			else
				if Samples(1 downto 0) = "10" and CRBH(0) = '0' then
					IC_Trig <= '1';
				end if;
				if Samples(1 downto 0) = "01" and CRBH(0) = '1' then
					IC_Trig <= '1';
				end if;
			end if;
			Samples(3 downto 1) := Samples(2 downto 0);
			Samples(0) := ICP;
		end if;
	end process;

	-- Tick generator
	process (Clk, Reset_n)
		variable Prescaler : unsigned(9 downto 0);
		variable T_r : std_logic_vector(1 downto 0);
	begin
		if Reset_n = '0' then
			Prescaler := (others => '0');
			Tick <= '0';
			T_r := "00";
		elsif Clk'event and Clk='1' then
			Tick <= '0';
			case CRBL(2 downto 0) is
			when "000" =>
			when "001" =>
				Tick <= '1';
			when "010" =>
				if T_r(1) = '1' and T_r(0) = '0' then
					Tick <= '1';
				end if;
				T_r(1) := T_r(0);
				T_r(0) := Prescaler(2);
			when "011" =>
				if T_r(1) = '1' and T_r(0) = '0' then
					Tick <= '1';
				end if;
				T_r(1) := T_r(0);
				T_r(0) := Prescaler(5);
			when "100" =>
				if T_r(1) = '1' and T_r(0) = '0' then
					Tick <= '1';
				end if;
				T_r(1) := T_r(0);
				T_r(0) := Prescaler(7);
			when "101" =>
				if T_r(1) = '1' and T_r(0) = '0' then
					Tick <= '1';
				end if;
				T_r(1) := T_r(0);
				T_r(0) := Prescaler(9);
			when "110" =>
				if T_r(1) = '1' and T_r(0) = '0' then
					Tick <= '1';
				end if;
				T_r(1) := T_r(0);
				T_r(0) := T;
			when others =>
				if T_r(1) = '0' and T_r(0) = '1' then
					Tick <= '1';
				end if;
				T_r(1) := T_r(0);
				T_r(0) := T;
			end case;
			Prescaler := Prescaler + 1;
		end if;
	end process;

end;
