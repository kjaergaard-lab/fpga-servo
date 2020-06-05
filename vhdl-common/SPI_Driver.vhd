library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.Constants.all;
use work.Procedures.all;

--
-- Transfers and receives data using the SPI protocol.  
-- Behaviour can be changed with generics.
--
entity SPI_Driver is
	generic (
			CPOL				:	std_logic;	--Serial clock idle polarity
			CPHA				:	std_logic;	--Serial clock phase. '0': data valid on CPOL -> not(CPOL) transition. '1': data valid on not(CPOL) -> CPOL transition
			ORDER				:	std_logic;	--Bit order.  '0': LSB first. '1': MSB first
			SYNC_POL			:	std_logic;	--Active polarity of SYNC.  '0': active low. '1': active high
			TRIG_SYNC		:	std_logic;	--Use synchronous detection for trigger?
			TRIG_EDGE		:	std_logic;	--Edge of trigger to synchronize on. '0': falling edge. '1': rising edge.
			ASYNC_WIDTH		:	integer;		--Width of asynchronous update pulse
			ASYNC_POL		:	std_logic;	--Active polarity of asynchronous signal
			MAX_NUM_BITS	:	integer);	--Maximum number of bits to transfer
			
	port( clk				:	in	std_logic;		--Clock signal
			SPI_period		:	in	integer;			--SCLK period
			numBits			:	in integer;			--Number of bits in current data
			dataReceived	:	out std_logic_vector(MAX_NUM_BITS-1 downto 0);	--data that has been received
			dataReady		:	out	std_logic;	--Pulses high for one clock cycle to indicate new data is valid on dataReceived
			SDIN				:	in		std_logic;	--To be connected to device SDOUT
			SDOUT				:	out	std_logic;	--To be connected to device SDIN
			SYNC				:	out	std_logic;	--SYNC frame
			SCLK				:	out	std_logic;	--Serial clock
			ASYNC				:	out	std_logic;	--Asynchronous update signal
			syncDelay		:	in	integer range 0 to 255;
			dataToSend		:	in	std_logic_vector(MAX_NUM_BITS-1 downto 0);	--data to be sent
			trigIn			:	in	std_logic;		--Start trigger
			enable			:	in std_logic;		--Enable bit
			busy				:	out std_logic);	--Transmission in progress
end SPI_Driver;

architecture Behavioral of SPI_Driver is

signal SPI_cnt	:	integer	:=	0;	--Counts clock edges
signal bit_cnt	:	integer range 0 to (MAX_NUM_BITS-1)	:=	0;	--counts bits.
signal trigSync	:	std_logic_vector(1 downto 0)	:=	"00";
signal trig	:	std_logic	:=	'0';
signal dataIn, dataOut	:	std_logic_vector(MAX_NUM_BITS-1 downto 0)	:=	(others => '0');

signal delayCount	:	integer range 0 to 255	:=	0;
signal state	:	integer range 0 to 3	:=	0;

begin

--Synchronous trigger detection
startAcq: process(clk,trigIn) is
	begin
		if rising_edge(clk) then
			trigSync <= (trigSync(0),trigIn);
		end if;
end process;

--Use appropriate trigger depending on options TRIG_SYNC and TRIG_EDGE
trig <= 	trigIn when TRIG_SYNC = '0' else
			'1' when (TRIG_EDGE = '0' and trigSYNC = "10") or (TRIG_EDGE = '1' and trigSYNC = "01") else
			'0';
			
SPI_Process: process(clk) is
begin
	if rising_edge(clk) then
		SPI_Case: case state is
			--Idle/wait for trigger state
			when 0 =>
				dataIn <= (others => '0');
				dataReady <= '0';
				SCLK <= CPOL;
				SDOUT <= '0';
				delayCount <= 0;
				ASYNC <= not(ASYNC_POL);
				if trig = '1' and enable = '1' then
					state <= state + 1;
					SPI_cnt <= 1;
					dataOut(numBits-1 downto 0) <= dataToSend;
					SYNC <= SYNC_POL;
					busy <= '1';
					if ORDER = '0' then
						bit_cnt <= 0;
					else
						bit_cnt <= numBits - 1;
					end if;
				else
					SYNC <= not(SYNC_POL);
					busy <= '0';
				end if;
				
			--Send/receive data state
			when 1 =>
				if SPI_cnt <= SPI_period/2 then
					if CPHA = '0' then
						SDOUT <= dataOut(bit_cnt);
						dataIn(bit_cnt) <= SDIN;
					end if;
					SCLK <= CPOL;
					SPI_cnt <= SPI_cnt + 1;
				elsif SPI_cnt < SPI_period then
					if CPHA = '1' then
						SDOUT <= dataOut(bit_cnt);
						dataIn(bit_cnt) <= SDIN;
					end if;
					SCLK <= not(CPOL);
					SPI_cnt <= SPI_cnt + 1;
				else
					if ORDER = '0' then
						if bit_cnt = numBits - 1 then
							bit_cnt <= 0;
							SPI_cnt <= 0;
							dataReceived <= dataIn;
							dataReady <= '1';
							state <= state + 1;
						else
							bit_cnt <= bit_cnt + 1;
							SPI_cnt <= 1;
						end if;
					else
						if bit_cnt = 0 then
							bit_cnt <= numBits - 1;
							SPI_cnt <= 0;
							dataReceived <= dataIn;
							dataReady <= '1';
							state <= state +1;
						else
							bit_cnt <= bit_cnt - 1;
							SPI_cnt <= 1;
						end if;
					end if;
				end if;
			
			--SYNC delay state
			when 2 =>
				dataReady <= '0';
				SCLK <= CPOL;
				SDOUT <= '0';
				if delayCount < syncDelay then
					delayCount <= delayCount + 1;
				else
					SYNC <= not(SYNC_POL);
					delayCount <= 0;
					state <= 3;
				end if;
				
			--ASYNC pulse state
			when 3 =>
				if delayCount < ASYNC_WIDTH then
					ASYNC <= ASYNC_POL;
					delayCount <= delayCount + 1;
				else
					state <= 0;
				end if;
				
			when others => null;
		end case;	--end SPI_case
	end if;	--end rising_edge
end process;
			

end Behavioral;

