library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 

--
-- Sends a 32-bit word to PC using UART protocol using a baud rate set by generic constant
-- Uses an 8-bit, 1 stop bit, 1 start bit, no parity scheme, except for the very last stop bit
-- where two stop bits are used, as I have had some trouble with the PC side receiving data
--
entity UART_Transmitter is
	generic(	baudPeriod	:	integer);									--Baud period
	
	port(	clk 			: 	in 	std_logic;								--Clock signal
			dataIn		:	in		std_logic_vector(31 downto 0);	--32-bit word to be sent
			trigIn		:	in		std_logic;								--Trigger to send data
			TxD			:	out	std_logic;								--Serial transmit port
			baudTickOut	:	out	std_logic;								--Output for baud ticks for testing
			busy			:	out	std_logic);								--Busy signal is high when transmitting
end UART_Transmitter;

architecture Behavioral of UART_Transmitter is

constant NUM_BITS	:	integer	:=	41;

signal state		:	integer range 0 to 3	:=	0;
signal bitCount	:	integer range 0 to NUM_BITS-1	:=	0;
signal data			:	std_logic_vector(NUM_BITS-1 downto 0) := (others => '0');
signal count		:	integer range 0 to baudPeriod	:=	0;

begin

SendData: process(clk) is
begin
	if rising_edge(clk) then
		SendFSM: case state is
			--
			-- Idle state.  When a trigger is received, the busy signal is raised
			-- and the data to be sent is latched into an internal signal
			--
			when 0 =>
				if trigIn = '1' then
					bitCount <= 0;
					count <= 0;
					data <= "11" & dataIn(31 downto 24) & "01" & dataIn(23 downto 16) & "01" & dataIn(15 downto 8) & "01" & dataIn(7 downto 0) & "0";
					state <= 2;	--Immediately send the first bit
					busy <= '1';
				else
					TxD <= '1';	--Idle signal for UART TxD is high
					busy <= '0';
				end if;
				
			--
			-- Baud counter
			--
			when 1 =>
				if count < baudPeriod then
					count <= count + 1;
					baudTickOut <= '0';
				else
					baudTickOut <= '1';
					state <= 2;
					count <= 0;
				end if;
				
			--
			-- Send data
			--
			when 2 =>
				TxD <= data(bitCount);
				if bitCount >= (NUM_BITS - 1) then
					bitCount <= 0;
					state <= 0;
				else
					bitCount <= bitCount + 1;
					state <= 1;
				end if;
		
			when others => null;
		end case;
	end if;
end process;


end Behavioral;

