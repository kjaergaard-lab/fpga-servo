library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
--
--This entity handles serial communication using the UART protocol.  Data
--is received on the RxD pin and transmitted on the TxD pin.  Data transmission
--starts on receipt of a high transmitTrig signal, and a transmitBusy signal is
--asserted while data is transmitted.
--
--Received data is pushed onto the cmdDataOut signal or the numDataOut signal depending
--on the value of dataFlag.  If '0', then cmdDataOut is used, and if '1' then numDataOut
--is used.
--
entity SerialCommunication is
	generic (baudPeriod	:	integer);									--Baud period appropriate for clk					
	port(	clk 				: 	in  std_logic;								--Clock signal

			--
			-- Signals for reading from serial port
			--
			RxD				:	in	std_logic;								--Input RxD from a UART signal
			cmdDataOut		:	out std_logic_vector(31 downto 0);	--32 bit command word
			numDataOut		:	out std_logic_vector(31 downto 0);	--Numerical parameter
			dataFlag			:	in	std_logic;								--Indicates type of data cmd/num
			dataReady		:	out 	std_logic;							--Flag to indicate that data is valid

			--
			-- Signals for transmitting on serial port
			--
			TxD				:	out std_logic;								--Serial transmit pin
			dataIn			:	in  std_logic_vector(31 downto 0);	--Data to transmit
			transmitTrig	:	in  std_logic;								--Trigger to start transmitting data
			transmitBusy	:	out std_logic);							--Flag to indicate that a transmission is in progress
end SerialCommunication;

architecture Behavioral of SerialCommunication is

component UART_Receiver
	generic( baudPeriod	:	integer);								--Baud period in clock cycles
	
	port(	clk 			: 	in  	std_logic;							--Clock signal
			dataOut		:	out	std_logic_vector(7 downto 0);	--Output data
			byteReady	:	out	std_logic;							--Signal to register the complete read of a byte
			RxD			:	in		std_logic;							--Input RxD from a UART signal
			baudTickOut	:	out 	std_logic);							--Output baud tick, used for debugging
end component;

component UART_Transmitter
	generic(	baudPeriod	:	integer);									--Baud period
	
	port(	clk 			: 	in 	std_logic;								--Clock signal
			dataIn		:	in		std_logic_vector(31 downto 0);	--32-bit word to be sent
			trigIn		:	in		std_logic;								--Trigger to send data
			TxD			:	out	std_logic;								--Serial transmit port
			baudTickOut	:	out	std_logic;								--Output for baud ticks for testing
			busy			:	out	std_logic);								--Busy signal is high when transmitting
end component;

component ReadData
	generic(	baudPeriod	:	integer);												--Baud period in clock cycles
				
	port(	clk 			:	in std_logic;												--Clock
			dataIn		:	in	std_logic_vector(7 downto 0);						--1 byte of data from UART_receiver
			byteReady	:	in	std_logic;												--Signal to tell if byte is valid
			cmdDataOut	:	out std_logic_vector(31 downto 0);					--32 bit command word
			numDataOut	:	out std_logic_vector(31 downto 0);					--Numerical parameter
			dataFlag		:	in	std_logic;												--Indicates type of data cmd/num
			dataReady	:	out std_logic);											--Indicates data is ready
end component;


signal 	serialData			: std_logic_vector(7 downto 0)	:= (others => '0');	--data from UART_Receiver
signal	serialDataReady	:	std_logic	:= '0';	--byte ready signal from UART_Receiver

begin

uart_receive: UART_Receiver 
generic map( 	baudPeriod => baudPeriod)
port map(
	clk => clk,
	dataOut => serialData,
	byteReady => serialDataReady,
	RxD => RxD,
	baudTickOut => open);

uart_transmit: UART_Transmitter 
generic map(	baudPeriod => baudPeriod)
port map(
	clk => clk,
	dataIn => dataIn,
	trigIn => transmitTrig,
	TxD => TxD,
	baudTickOut => open,
	busy => transmitBusy);
	
assemble_data: ReadData 
generic map(	baudPeriod => baudPeriod)
port map(
	clk => clk,
	dataIn => serialData,
	byteReady => serialDataReady,
	cmdDataOut => cmdDataOut,
	numDataOut => numDataOut,
	dataFlag => dataFlag,
	dataReady => dataReady);





end Behavioral;

