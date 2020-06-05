library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 

--
-- Reads one byte of data in from RxD based on generic baudPeriod
-- Assumes that there are 8 data bits, 1 start bit, 1 stop bit, and no parity bits
--
entity UART_Receiver is
	generic( baudPeriod	:	integer);								--Baud period in clock cycles
	
	port(	clk 			: 	in  	std_logic;							--Clock signal
			dataOut		:	out	std_logic_vector(7 downto 0);	--Output data
			byteReady	:	out	std_logic;							--Signal to register the complete read of a byte
			RxD			:	in		std_logic;							--Input RxD from a UART signal
			baudTickOut	:	out 	std_logic);							--Output baud tick, used for debugging
end UART_Receiver;

architecture Behavioral of UART_Receiver is

signal bitCount	:	integer range 0 to 8	:= 0;
signal state		:	integer range 0 to 3	:=	0;
signal syncRxD		:	std_logic_vector(3 downto 0) := (others => '1');
signal count		:	integer range 0 to baudPeriod		:= 0;

begin


ReceiveData: process(clk) is
begin
	if rising_edge(clk) then
		UART_FSM: case state is
			--
			-- Idling state.  Waits for 4 consecutive low values of
			-- the RxD signal before deciding that a signal is being
			-- transmitted.
			--
			when 0 =>
				baudTickOut <= '0';
				bitCount <= 0;
				byteReady <= '0';
				syncRxD <= syncRxD(2 downto 0) & RxD;	
				if syncRxD = X"0" then
					state <= 1;
					count <= 0;
				else
					state <= 0;
				end if;
				
			--
			-- Wait half a baud period so that the baud counter
			-- occurs in the middle of every data bit
			--
			when 1 =>
				if count < baudPeriod/2 then
					count <= count + 1;
					baudTickOut <= '0';
				else
					baudTickOut <= '1';
					count <= 0;
					state <= 2;
				end if;
				
			--
			-- Wait a full period. Baud ticks appear in the middle of bits
			--
			when 2 =>
				if count < baudPeriod then
					count <= count + 1;
					baudTickOut <= '0';
				else
					count <= 0;
					state <= 3;
					baudTickOut <= '1';
				end if;
				
			--
			-- Read bit
			--
			when 3 =>
				if bitCount < 8 then
					dataOut(bitCount) <= RxD;	--read bit in
					bitCount <= bitCount + 1;
					state <= 2;						--Return to delay loop
				else
					byteReady <= '1';				--signal that byte is ready
					bitCount <= 0;					--reset bit count
					state <= 0;						--return to start
					syncRxD <= X"F";				--reset RxD signal to be all '1's
				end if;
			
			when others => null;
		end case;
	
	end if;
end process;


end Behavioral;

