library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
--
-- Takes bytes of data from UART_Recevier and assembles them into either 32 bit command or numerical data
--
entity ReadData is
	generic(	baudPeriod	:	integer);												--Baud period in clock cycles
				
	port(	clk 			:	in std_logic;												--Clock
			dataIn		:	in	std_logic_vector(7 downto 0);						--1 byte of data from UART_receiver
			byteReady	:	in	std_logic;												--Signal to tell if byte is valid
			cmdDataOut	:	out std_logic_vector(31 downto 0);					--32 bit command word
			numDataOut	:	out std_logic_vector(31 downto 0);					--Numerical parameter
			dataFlag		:	in	std_logic;												--Indicates type of data cmd/num
			dataReady	:	out std_logic);											--Indicates data is ready
end ReadData;

architecture Behavioral of ReadData is

constant numBytes	:	integer	:=	4;
constant timeOut	:	integer	:=	100000*baudPeriod;	--100,000 baudPeriods until data assembly times out

signal state	:	integer range 0 to 3	:=	0;
signal byteCount	:	integer range 0 to 8	:=	0;	--Maximum number of bytes is 8 (64 bits)
signal countTimeOut	:	integer range 0 to timeOut	:=	0;
signal data	:	std_logic_vector(63 downto 0)	:= (others => '0');


begin

AssembleData: process(clk) is
begin
	if rising_edge(clk) then
		AssembleFSM: case state is
			--
			-- Idle state. Waits for a byte to be ready.
			--
			when 0 =>
				dataReady <= '0';
				if byteReady = '1' then
					countTimeOut <= 0;
					state <= 1;
				else
					dataReady <= '0';
					if countTimeOut < timeOut then
						countTimeOut <= countTimeOut + 1;
					else
						countTimeOut <= 0;
						byteCount <= 0;
					end if;
				end if;
				
			--
			-- Add new byte to data
			--
			when 1 =>
				data((byteCount+1)*8-1 downto byteCount*8) <= dataIn;
				byteCount <= byteCount + 1;
				state <= 2;
				
			--
			-- Check if total instruction is assembled.  Push data to
			-- either cmdDataOut or numDataOut depending on dataFlag
			--
			when 2 =>
				state <= 0;
				if byteCount = numBytes then
					dataReady <= '1';
					byteCount <= 0;
					if dataFlag = '0' then
						cmdDataOut <= data(8*numBytes-1 downto 0);
					else
						numDataOut <= data(8*numBytes-1 downto 0);
					end if;
				end if;
			when others => null;
		end case;
	end if;
end process;



end Behavioral;

