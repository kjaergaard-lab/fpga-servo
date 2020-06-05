library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use work.Constants.all;
use work.Procedures.all;

--
--This top-level entity implements a single servo controller.
--
entity topmod is
	port (	
				clk100x	:	in	std_logic;				--External clock on board
				--
				-- Serial connections
				--
				RxD	:	in	std_logic;					--Receiver pin
				TxD	:	out std_logic;					--Transmission pin
				
				extStartTrig		:	in		std_logic_vector(N-1 downto 0);		--Externally applied start trigger
				EXT_SWITCH			:	in		std_logic_vector(N-1 downto 0);		--Externally applied switch signal
				
				ADC_SCLK				:	out	std_logic_vector(N-1 downto 0);		--ADC serial clock
				ADC_SDIN				:	in		std_logic_vector(N-1 downto 0);		--DOUT on ADC eval board
				ADC_SDOUT			:	out	std_logic_vector(N-1 downto 0);		--DIN on ADC eval board
				ADC_SYNC				:	out	std_logic_vector(N-1 downto 0);		--SYNC signal on ADC eval board
				ADC_DRDY				:	in		std_logic_vector(N-1 downto 0);		--DRDY on ADC eval board
				ADC_START			:	out	std_logic_vector(N-1 downto 0);		--START on ADC chip
				
				DAC_SDOUT			:	out	std_logic_vector(N-1 downto 0);	--SDIN on DAC Eval Board
				DAC_SYNC				:	out	std_logic_vector(N-1 downto 0);	--SYNC on DAC Eval Board
				DAC_SCLK				:	out	std_logic_vector(N-1 downto 0);	--SCLK on DAC Eval Board
				DAC_LDAC				:	out	std_logic_vector(N-1 downto 0);	--LDAC on DAC Eval Board
				DAC_SWITCH			:	out	std_logic_vector(N-1 downto 0);	--Output for an external switch
			
				--
				-- LPDDR memory signals
				--
				mcb3_dram_dq                            : inout  std_logic_vector(DDR_NUM_DQ_PINS-1 downto 0);
				mcb3_dram_a                             : out std_logic_vector(DDR_ADDR_WIDTH-1 downto 0);
				mcb3_dram_ba                            : out std_logic_vector(DDR_BANK_WIDTH-1 downto 0);
				mcb3_dram_cke                           : out std_logic;
				mcb3_dram_ras_n                         : out std_logic;
				mcb3_dram_cas_n                         : out std_logic;
				mcb3_dram_we_n                          : out std_logic;
				mcb3_dram_dm                            : out std_logic;
				mcb3_dram_udqs                          : inout  std_logic;
				mcb3_rzq                                : inout  std_logic;
				mcb3_dram_udm                           : out std_logic;
--				c3_sys_clk                              : in  std_logic;
--				c3_sys_rst_n                            : in  std_logic;
--				c3_calib_done                           : out std_logic;
--				c3_clk0                                 : out std_logic;
--				c3_rst0                                 : out std_logic;
				mcb3_dram_dqs                           : inout  std_logic;
				mcb3_dram_ck                            : out std_logic;
				mcb3_dram_ck_n                          : out std_logic
				
--				calib_done	:	out std_logic
				
			);
end topmod;

architecture Behavioral of topmod is

------------------------------------------------------------
-------------   IP Components  -----------------------------
------------------------------------------------------------
component DCM1
	port
	 (-- Clock in ports
	  CLK_IN1	: 	in	std_logic;
	  -- Clock out ports
	  CLK_OUT1	: 	out	std_logic;
	  CLK_OUT2	:	out	std_logic				
	 );
end component;


------------------------------------------------------------
-------------   PC Communication Components  ---------------
------------------------------------------------------------
component SerialCommunication
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
end component;

------------------------------------------------------------
---------------   Servo Control Components  ----------------
------------------------------------------------------------
component Analog_Digital_Control
	generic(	ID				:	unsigned(MEM_ADDR_ID_WIDTH-1 downto 0));								--Controller ID
	port(	clk				:	in	std_logic;											--Input clock
			trigIn			:	in std_logic;											--Start trigger in.  Can be overridden by a software start
			--
			-- Serial communication signals
			--
			cmdData			:	in std_logic_vector(31 downto 0);				--Command data from serial
			dataReady		:	in	std_logic;											--Indicates that new data is valid
			numData			:	in	std_logic_vector(31 downto 0);				--Numerical data from serial
			dataFlag			:	inout	std_logic	:=	'0';							--Flag that indicates the kind of data available
			
			--
			-- Memory signals
			--
			memReadTrig			:	out std_logic;										--Trigger to start memory read
			memReadAddr			:	out mem_addr;										--Address to read from
			memDataReady		:	in	std_logic;										--Signal indicates memData is valid
			memReadData			:	in mem_data;										--Data returned from memory	
			
			memWriteTrig		:	out std_logic;										--Trigger to start memory read
			memWriteAddr		:	out mem_addr;										--Address to read from
			memWriteData		:	out mem_data;										--Data returned from memory
			
			--
			-- DAC SPI signals
			--
			DAC_SDOUT	:	out	std_logic;	--SDIN on DAC
			DAC_SYNC		:	out	std_logic;	--SYNC on DAC
			DAC_SCLK		:	out	std_logic;	--SCLK on DAC
			DAC_LDAC		:	out	std_logic;	--LDAC on DAC
			DAC_SWITCH	:	out	std_logic;	--Analog switch
			EXT_SWITCH	:	in		std_logic;	--External switch
			
			--
			-- ADC SPI signals (for ADS127L01)
			--
			ADC_SCLK		:	out	std_logic;	--SCLK on ADC
			ADC_SYNC		:	out	std_logic;	--SYNC on ADC
			ADC_SDOUT	:	out	std_logic;	--SDIN on ADC 
			ADC_SDIN		:	in		std_logic;	--SDOUT on ADC 
			ADC_DRDY		:	in		std_logic;	--DRDY on ADC
			ADC_START	:	out	std_logic;	--START signal on ADS127L01
			
			--
			-- Transmission signals
			--
			transmitBusy	:	in		std_logic;								--Signal indicating that transmission line is busy
			dataToTransmit	:	out	std_logic_vector(31 downto 0);	--Data to transmit to PC
			transmitTrig	:	out	std_logic;								--Trigger for data transmission
			
			--
			-- Other signals
			--
			debug	:	out	std_logic_vector(3 downto 0));
end component;

------------------------------------------------------------
-------------------   Memory Components  -------------------
------------------------------------------------------------
component LPDDRControl_2channel is
port
  (
	--
	-- Physical memory signals
	--
   mcb3_dram_dq                            : inout  std_logic_vector(DDR_NUM_DQ_PINS-1 downto 0);
   mcb3_dram_a                             : out std_logic_vector(DDR_ADDR_WIDTH-1 downto 0);
   mcb3_dram_ba                            : out std_logic_vector(DDR_BANK_WIDTH-1 downto 0);
   mcb3_dram_cke                           : out std_logic;
   mcb3_dram_ras_n                         : out std_logic;
   mcb3_dram_cas_n                         : out std_logic;
   mcb3_dram_we_n                          : out std_logic;
   mcb3_dram_dm                            : out std_logic;
   mcb3_dram_udqs                          : inout  std_logic;
   mcb3_rzq                                : inout  std_logic;
   mcb3_dram_udm                           : out std_logic;
   c3_sys_clk                              : in  std_logic;
   c3_sys_rst_n                            : in  std_logic;
   c3_calib_done                           : out std_logic;
   c3_clk0                                 : out std_logic;
   c3_rst0                                 : out std_logic;
   mcb3_dram_dqs                           : inout  std_logic;
   mcb3_dram_ck                            : out std_logic;
   mcb3_dram_ck_n                          : out std_logic;
	
	--
	-- Clock for user logic
	--
	clk			:	in	std_logic;
	--
	-- User signals
	--
	trigWrite	:	in std_logic_vector(NM-1 downto 0);
	trigRead		:	in std_logic_vector(NM-1 downto 0);
	dataIn		:	in mem_data_array(NM-1 downto 0);
	dataOut		:	out mem_data_array(NM-1 downto 0);
	dataReady	:	out std_logic_vector(NM-1 downto 0);
	addrRead		:	in	mem_addr_array(NM-1 downto 0);
	addrWrite	:	in	mem_addr_array(NM-1 downto 0);
	
	rd_error	:	out	std_logic_vector(NM-1 downto 0);
	wr_error	:	out	std_logic_vector(NM-1 downto 0)
	);
end component;

------------------------------------------------------------------------------------
-----------------------  Clock and timing signals   --------------------------------
------------------------------------------------------------------------------------
signal clk100, clk50	:	std_logic;


------------------------------------------------------------------------------------
----------------------  Serial interface signals  ----------------------------------
------------------------------------------------------------------------------------
signal cmdData, numData, dataToSendTop	:	serial_data	:=	(others => '0');
signal dataFlagTop, dataReady, transmitTrigTop, transmitBusy	:	std_logic	:=	'0';

signal dataToSend		:	serial_data_array(N_MAX-1 downto 0)	:=	(others => (others => '0'));
signal dataFlag		:	std_logic_vector(N-1 downto 0)	:=	(others => '0');
signal transmitTrig	:	std_logic_vector(N_MAX-1 downto 0)	:=	(others => '0');
signal transmitCount	:	natural range 0 to N-1				:=	0;

-------------------------------------------------------------------------------------
-----------------------   Memory signals  -------------------------------------------
-------------------------------------------------------------------------------------
signal readTrig, writeTrig, memDataReady	:	std_logic_vector(NM-1 downto 0)	:=	(others => '0');
signal readAddr, writeAddr						:	mem_addr_array(NM-1 downto 0);
signal readData, writeData						:	mem_data_array(NM-1 downto 0);

begin

-------------------------------------------------------------------------
----------------------  IP Components  ----------------------------------
-------------------------------------------------------------------------

Inst_dcm1: DCM1 port map (
	CLK_IN1 => clk100x,
	CLK_OUT1 => clk100,
	CLK_OUT2 => clk50);

  
-------------------------------------------------------------------------
-----------------  PC Communication Components  -------------------------
-------------------------------------------------------------------------
	
SerialCommunication_inst: SerialCommunication 
generic map(
	baudPeriod => BAUD_PERIOD)
port map(
	clk => clk50,
	
	RxD => RxD,
	cmdDataOut => cmdData,
	numDataOut => numData,
	dataFlag => dataFlagTop,
	dataReady => dataReady,
	
	TxD => TxD,
	dataIn => dataToSendTop,
	transmitTrig => transmitTrigTop,
	transmitBusy => transmitBusy);
	
dataFlagTop <= unary_or(dataFlag);
--dataFlagTop <= dataFlag(0);

--dataToSendTop <= dataToSend(0);
--transmitTrigTop <= transmitTrig(0);

TransmitRouting: process(clk50) is
begin
	if rising_edge(clk50) then
		if unary_or(transmitTrig) = '1' then
			transmitTrigTop <= '1';
			TransmitCase: case(transmitTrig) is
				when "01" => dataToSendTop <= dataToSend(0);
				when "10" => dataToSendTop <= dataToSend(1);
				when others => null;
			end case;
		else
			transmitTrigTop <= '0';
		end if;
	end if;
end process;
	
------------------------------------------------------------
---------------   Servo Control Components  ----------------
------------------------------------------------------------

Servo_Gen:
for I in 0 to N-1 generate
	ServoX: Analog_Digital_Control
	generic map(
		ID => to_unsigned(I,MEM_ADDR_ID_WIDTH))
	port map(
		clk 				=> clk50,
		trigIn 			=> extStartTrig(I),
		cmdData 			=> cmdData,
		dataReady 		=> dataReady,
		numData 			=> numData,
		dataFlag 		=> dataFlag(I),
		
		memReadTrig 	=> readTrig(I),
		memReadAddr 	=> readAddr(I),
		memDataReady 	=> memDataReady(I),
		memReadData 	=> readData(I),
		
		memWriteTrig 	=> writeTrig(I),
		memWriteAddr 	=> writeAddr(I),
		memWriteData 	=> writeData(I),
		
		DAC_SDOUT 		=> DAC_SDOUT(I),
		DAC_SYNC 		=> DAC_SYNC(I),
		DAC_SCLK 		=> DAC_SCLK(I),
		DAC_LDAC 		=> DAC_LDAC(I),
		DAC_SWITCH 		=> DAC_SWITCH(I),
		EXT_SWITCH 		=> EXT_SWITCH(I),
		
		ADC_SCLK 		=> ADC_SCLK(I),
		ADC_SYNC 		=> ADC_SYNC(I),
		ADC_SDIN 		=> ADC_SDIN(I),
		ADC_SDOUT 		=> ADC_SDOUT(I),
		ADC_DRDY 		=> ADC_DRDY(I),
		ADC_START 		=> ADC_START(I),
		
		debug 			=> open,
		
		transmitBusy 	=> transmitBusy,
		dataToTransmit => dataToSend(I),
		transmitTrig 	=> transmitTrig(I)
		);
end generate Servo_Gen;
		
------------------------------------------------------------
-------------------   Memory Components  -------------------
------------------------------------------------------------
MemController: LPDDRControl_2channel
	port map(
		mcb3_dram_dq => mcb3_dram_dq,
		mcb3_dram_a => mcb3_dram_a,
		mcb3_dram_ba => mcb3_dram_ba,
		mcb3_dram_cke => mcb3_dram_cke,
		mcb3_dram_ras_n => mcb3_dram_ras_n,
		mcb3_dram_cas_n => mcb3_dram_cas_n,
		mcb3_dram_we_n => mcb3_dram_we_n,
		mcb3_dram_dm => mcb3_dram_dm,
		mcb3_dram_udqs => mcb3_dram_udqs,
		mcb3_rzq => mcb3_rzq,
		mcb3_dram_udm => mcb3_dram_udm,
		c3_sys_clk => clk100,
		c3_sys_rst_n => '0',
		c3_calib_done => open,
		c3_clk0 => open,
		c3_rst0 => open,
		mcb3_dram_dqs => mcb3_dram_dqs,
		mcb3_dram_ck => mcb3_dram_ck,
		mcb3_dram_ck_n => mcb3_dram_ck_n,
		
		--User signals
		clk => clk50,
		trigWrite => writeTrig,
		trigRead => readTrig,
		dataIn => writeData,
		dataOut => readData,
		dataReady => memDataReady,
		addrRead => readAddr,
		addrWrite => writeAddr,
		
		wr_error => open,
		rd_error => open);
		

end Behavioral;

