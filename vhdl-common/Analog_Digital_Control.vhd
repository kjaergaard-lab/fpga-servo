library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.Constants.all;
use work.Procedures.all;

--
-- This entity is used to control both an ADC and a DAC for use in a digital PID controller.
-- Multiple controllers can be instantiated in the top-level component as long as they have
-- distinct ID values.  The behaviour of the controller can be changed with the generic values.
--
entity Analog_Digital_Control is	
	generic(	ID				:	unsigned(MEM_ADDR_ID_WIDTH-1 downto 0));			--Controller ID
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
end Analog_Digital_Control;

architecture Behavioral of Analog_Digital_Control is

--
-- Define components
--
component SPI_Driver
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
end component;

component SampleGenerator
	generic(	ID							:	unsigned(MEM_ADDR_ID_WIDTH-1 downto 0);		--Controller ID
				SAMPLE_LOCATION		:	unsigned(1 downto 0);		--Location of samples
				LOOP1_LOCATION			:	unsigned(1 downto 0);		--Location of first loop register
				LOOP2_LOCATION			:	unsigned(1 downto 0)			--Location of second loop register
				);		
	port (	clk				:	in	std_logic;	--Input clock
				trigIn			:	in	std_logic;	--Input trigger
				reset				:	in std_logic;	--Input reset signal
				numSamples		:	in	integer;		--Number of samples to use
				updateTime		:	in	integer;		--Number of clk cycles between sample updates
				sampleSource	:	in	std_logic;	--Internal ramp generator ('0') or read from memory ('1')
				
				rampValues			:	in	int_array(0 to MAX_NUM_RAMPS-1);	--Values at boundary times
				rampBndyTimes		:	in	int_array(0 to MAX_NUM_RAMPS-1);	--Boundary times at which the voltage ramps change
				rampRates			:	in	int_array(0 to MAX_NUM_RAMPS-2);	--Derivatives (code value/update time) for each ramp
				rampCodes			:	in	int_array(0 to MAX_NUM_RAMPS-1);	--Codes at boundary times
				
				memReadTrig		:	out std_logic;									--Trigger to start memory read
				memDataReady	:	in	std_logic;									--Signal to indicates memData is valid
				memAddr			:	out mem_addr;									--Address to read from
				memData			:	in mem_data;									--Data returned from memory		
								
				sampleOut	:	out integer;											--Output sample
				sampleCode	:	out std_logic_vector(7 downto 0);				--Output code
				loopReg		:	out std_logic_vector(4*K_WIDTH-1 downto 0);	--Output loop parameters
				sampleTrig	:	out std_logic;											--Trigger indicating valid sampleOut
				genEnabled	:	out std_logic											--Generator enabled?
			);
end component;

component PID_Controller is
	generic(	MEAS_WIDTH	:	integer	:=	24;							--Width of the measurement signal
				CNTL_WIDTH	:	integer	:= 24;							--Width of the control signal
				OUT_WIDTH	:	integer	:=	20;							--Width of the output signal
				K_WIDTH		:	integer	:=	16);							--Width of the PID gain parameters
	port(	clk			:	in	std_logic;									--Clock signal
			enable		:	in	std_logic;									--Enables PID algorithm
			measureIn	:	in	signed(MEAS_WIDTH-1 downto 0);		--Measurement signal
			controlIn	:	in signed(CNTL_WIDTH-1 downto 0);		--Control signal
			measReady	:	in	std_logic;									--Signal that new measurement is valid.
			
			polarity			:	in std_logic := '0';						--Negative ('0') or positive ('1') sign for error signal
			useFixed			:	in std_logic := '1';						--Use fixed loop parameters ('1') or varying ones ('0')
			loopRegFixed	:	in std_logic_vector(4*K_WIDTH-1 downto 0);	--Combination of loop parameters for fixed loop parameters
			loopRegVary		:	in std_logic_vector(4*K_WIDTH-1 downto 0);	--Combination of loop parameters for variable loop parameters
			
			dacZero		:	in	unsigned(OUT_WIDTH-1 downto 0);		--Code indicating when the DAC outputs 0 V
			minValueIn	:	in	unsigned(OUT_WIDTH-1 downto 0);		--Maximum output value for DAC
			maxValueIn	:	in	unsigned(OUT_WIDTH-1 downto 0);		--Minimum output value for DAC
			
			errorOut		:	out std_logic_vector(31 downto 0);		--Error signal out - used for transmission of signal to PC
			pidOut		:	out unsigned(OUT_WIDTH-1 downto 0);		--Output value for DAC
			trigOut		:	out std_logic);
end component;

component ADCMemRWControl
	generic(	ID							:	unsigned(MEM_ADDR_ID_WIDTH-1 downto 0);						--Controller ID
				DATA_LOCATION			:	unsigned(1 downto 0));						--Data location		
	port(	clk			:	in	std_logic;													--Clock
			dataIn		:	in	mem_data;													--Input data to be written to memory
			dataReady	:	in	std_logic;													--Input data is valid
			numSamples	:	in	integer;														--Number of samples for manual triggering
			numMemSamp	:	in integer;														--Number of samples requested when reading back sample data from the PC
			trigManual	:	in	std_logic;													--Manual trigger
			enableIn		:	in	std_logic;													--Externally supplied enable bit
			enableOut	:	out	std_logic;												--Output enable bit
			
			sampleStep			:	in	integer;												--The step size
			useSampleLimits	:	std_logic;												--Use limits on the sample count to limit the amount of data?
			minSample			:	in	integer;												--Minimum value of sampleCount that can be saved
			maxSample			:	in	integer;												--Maximum value of sampleCount that can be saved
			numReqSamples		:	out integer;											--Last sample written
			
			trigWrite	:	out	std_logic;												--Write trigger for memory
			addrWrite	:	out	mem_addr;												--Address for memory write
			dataWrite	:	out	mem_data;												--Data for memory write
			trigRead		:	out	std_logic;												--Read trigger for memory
			addrRead		:	out	mem_addr;												--Address for memory read
			dataRead		:	in		mem_data;												--Data from memory read
			dataValid	:	in		std_logic;												--Data from memory is valid
			
			memReadTypeIn	:	in		unsigned(1 downto 0);							--What data to read out and transmit
			startTransmit	:	in		std_logic;											--Trigger to start transmission of data
			dataToTransmit	:	out	std_logic_vector(31 downto 0);				--Data to transmit
			trigTransmit	:	out	std_logic;											--Trigger for transmission
			transmitBusy	:	in		std_logic);											--Indicates that transmitter is busy
end component;

component Count_Edges is
	generic(	EDGE_TYPE	:	std_logic;	--'0' for falling edge, '1' for rising edge
				MAX_COUNT	:	integer);	--The maximum count expected, used for setting the count integer range
	port(	clk		:	in std_logic;		--Input clock used for counting
			edgeIn	:	in	std_logic;		--Input signal whose edges we wish to count
			countOut	:	out integer);		--The number of clock cycles between edges
end component;

-------------------------------------------------------------
-------------------   Memory constants   --------------------
-------------------------------------------------------------
constant SAMPLE_LOCATION			:	unsigned(1 downto 0)	:=	"00";	--Location of samples for the sample generator
constant LOOP1_LOCATION				:	unsigned(1 downto 0)	:=	"01";	--Location of the first two loop parameters
constant LOOP2_LOCATION				:	unsigned(1 downto 0)	:=	"10";	--Location of the second two loop parameters
constant DATA_LOCATION				:	unsigned(1 downto 0)	:=	"11";	--Location of saved data

-------------------------------------------------------------
-------------------   Global Signals   ----------------------
-------------------------------------------------------------
signal autoFlag						:	std_logic							:=	'1';								--Indicates that the controller is in auto ('1') or manual ('0') mode
signal trigSync, extSync			:	std_logic_vector(1 downto 0)	:=	"00";								--Used for synchronizing external signals
signal trig								:	std_logic							:=	'0';								--Internal trigger signal
signal enableTrig						:	std_logic							:=	'0';								--Default is zero so that on power-up the system MUST be programmed via PC to work
signal trigCount						:	unsigned(31 downto 0)			:= to_unsigned(0,32);			--Counter used sequence timing - see offTime
constant trigHoldOff					:	unsigned(31 downto 0)			:=	to_unsigned(100000000,32);	--1 s hold off with 100 MHz clock

-------------------------------------------------------------
----------------------   Parameters   -----------------------
-------------------------------------------------------------
signal numSamples						:	integer								:=	1000;								--Number of samples to generate/read for current sequence
signal updateTime						:	integer								:=	25000;							--Delay between successive samples
signal spiPeriod						:	integer								:= 20;								--Period for for ADC and DAC SPI clocks
signal sampleSource					:	std_logic							:=	'0';								--Sample source - internal ('0') or memory ('1')
signal offTime							:	unsigned(31 downto 0)			:=	to_unsigned(50000000,32);	--Time after trigger at which controller switches off
signal transmitType					:	integer range 0 to 7				:=	0;									--Type of data to save and later transmit

signal Kp, Ki, Kd, divisorPID		:	std_logic_vector(K_WIDTH-1 downto 0)			:=	X"0000";				--PID loop parameters
signal loopRegFixed, loopRegVary	:	std_logic_vector(4*K_WIDTH-1 downto 0)			:=	(others => '0');	--Combinations of loop parameters as (divisorPID,Kd,Ki,Kp)
signal polarity, useFixed			:	std_logic												:=	'0';					--PID polarity and whether or not to use fixed loop parameters

signal rampValues						:	int_array(0 to MAX_NUM_RAMPS-1)					:=	(others => 0);	--Ramp values
signal rampBndyTimes					:	int_array(0 to MAX_NUM_RAMPS-1)					:=	(others => 0);	--Ramp boundary times
signal rampRates						:	int_array(0 to MAX_NUM_RAMPS-2)					:=	(others => 0);	--Ramp derivatives
signal rampCodes						:	int_array(0 to MAX_NUM_RAMPS-1)					:=	(others => 0);	--Ramp values

-------------------------------------------------------------
---------------------   PID Signals   -----------------------
-------------------------------------------------------------
signal control, measurement		:	signed(ADC_DATA_WIDTH-1 downto 0)	:=	(others => '0');	--Control and measurement signals for PID algorithm
signal enablePID						:	std_logic									:=	'0';					--Signal to enable PID controller
signal outPID							:	unsigned(19 downto 0)					:=	(others => '0');	--Value to send to DAC
signal trigPID							:	std_logic									:=	'0';					--Trigger from PID controller
signal errorOut						:	std_logic_vector(31 downto 0)			:=	(others => '0');	--Error signal from PID controller as 32-bit value
signal dacZeroForPID					:	unsigned(19 downto 0);												--DAC code indicating 0 V for PID controller

-------------------------------------------------------------
--------------   Sample Generator Signals   -----------------
-------------------------------------------------------------
signal sampleOut										:	integer								:=	0;						--Sample out of sample generator
signal sampleCode										:	std_logic_vector(7 downto 0)	:= X"00";				--Sample code out of sample generator
signal sampleTrig										:	std_logic							:=	'0';					--Sample trigger from sample generator
signal sampleGenFinished							:	std_logic							:=	'0';					--Indicates that the sample generator has finished
signal sampleGenEnabled								:	std_logic							:= '0';					--Indicates that the sample generator is running

signal memSampleReadTrig, memSampleDataReady	:	std_logic							:=	'0';					--Memory read trigger from sample generator and corresponding data ready signal
signal memSampleReadAddr							:	mem_addr								:=	(others => '0');	--Read address corresponding to the sample generator
signal memSampleReadData							:	mem_data								:=	(others => '0');	--Read data for sample generator

-------------------------------------------------------------
---------------------   DAC Signals   -----------------------
-------------------------------------------------------------
constant DAC_ZERO_UNI								:	unsigned(19 downto 0) 			:= X"00000";	--DAC code indicating 0 V in unipolar mode
constant DAC_ZERO_BI									:	unsigned(19 downto 0) 			:= X"80000";	--DAC code indicating 0 V in bipolar mode
signal dacZero											:	unsigned(19 downto 0)			:=	X"00000";	--DAC code used to represent 0 V

signal outDAC1, outDAC2, outDAC3					:	std_logic_vector(19 downto 0)	:=	X"00000";	--Various stages of DAC signals with different limits applied
signal outDAC, dataDAC								:	std_logic_vector(23 downto 0)	:=	X"000000";	--24-bit representations of the final DAC signal
signal busyDAC											:	std_logic;												--Signal from SPI controller indicating that the DAC is current being written to
signal trigDAC1, trigDAC2, trigDAC				:	std_logic							:=	'0';			--Various triggers for the DAC at different stages of routing
signal autoSwitchDAC, manSwitchDAC				:	std_logic							:=	'0';			--Switch signals for the DAC in different modes
signal useExternalSwitch, dacMode				:	std_logic							:=	'0';			--Use external switch signal and the dacMode switch

signal minValueDAC									:	unsigned(19 downto 0)			:=	X"00000";	--Minimum DAC value
signal maxValueDAC									:	unsigned(19 downto 0)			:=	X"FFFFF";	--Maximum DAC value
signal syncDelay										:	integer range 0 to 255			:=	0;				--Delay between final data bit and the rising edge of the SYNC signal in the SPI module

signal overrideTrigDAC, overrideDAC				:	std_logic							:=	'0';			--Override signals for the DAC which lets a specified zero value be written to the DAC at the offTime
signal overrideStateDAC								:	integer range 0 to 3				:=	0;				--Counter for the override state machine
signal dacOffValue									:	std_logic_vector(19 downto 0)	:=	X"00000";	--Value to write to DAC in override process

signal sampleOutDAC									:	integer								:=	0;				--Sample code for DAC output as an integer - used for limiting the values

-------------------------------------------------------------
---------------------   ADC Signals   -----------------------
-------------------------------------------------------------
constant MAX_COUNT					:	integer												:=	50000;				--Maximum number of edges for the ADC DRDY pin
signal samplePeriodADC				:	integer range 0 to MAX_COUNT-1				:=	0;						--Number of clock cycles taken by the ADC DRDY pin

signal dataADC							:	std_logic_vector(23 downto 0)					:= X"000000";			--Data read from ADC
signal dataReadyADC, enableADC	:	std_logic											:=	'0';					--Signals indicating that ADC data is ready and that the ADC should be enabled

signal trigWriteADC, trigReadADC : 	std_logic											:=	'0';					--Triggers for writing and reading data from memory for the ADC
signal addrWriteADC, addrReadADC	:	mem_addr												:=	(others => '0');	--Read/write addresses for data from the ADC
signal dataWriteADC					:	mem_data												:=	(others => '0');	--Data to write to the ADC

-------------------------------------------------------------
------------------   Memory Signals   -----------------------
-------------------------------------------------------------
signal dataToSave											:	std_logic_vector(31 downto 0)						:= (others => '0');	--Data to save to memory
signal memTrig, reset, resetMan, resetAuto		:	std_logic												:=	'0';					--Memory triggers and reset signals
signal numMemSamples										:	integer													:=	0;						--Number of samples to read from memory in case of reading sample data back (debugging)
signal memAddrWriteSerial								:	mem_addr													:=	(others => '0');	--Write address from serial parser
signal memTrigWriteSerial								:	std_logic												:=	'0';					--Write trigger from serial parser
signal memDataWriteSerial								:	mem_data													:=	(others => '0');	--Write data from serial parser

signal transmitTrigADC, transmitTrigSerial		:	std_logic												:=	'0';					--UART transmission triggers
signal dataToTransmitADC, dataToTransmitSerial	:	std_logic_vector(31 downto 0)						:=	(others => '0');	--UART data signals

signal useSampleLimits									:	std_logic												:=	'0';					--Use sample limits?
signal minSample, maxSample							:	integer													:=	0;						--Min and max samples to read back
signal sampleStep											:	integer													:=	1;						--Step size when reading back
signal numReqSamples										:	integer													:=	0;						--Counts the number of samples that should be sent back, used for checking for transmission errors

signal startTransmit										:	std_logic												:=	'0';					--Start transmitting data from memory?
signal memReadType										:	unsigned(1 downto 0)									:=	"11";					--What data to read from memory - "11" is measured data

-------------------------------------------------------------
------------------   Manual Signals   -----------------------
-------------------------------------------------------------
signal manTrigADC, manTrigRamp, manTrigDAC		:	std_logic												:=	'0';
signal manDataDAC											:	std_logic_vector(DAC_DATA_WIDTH-1 downto 0)	:=	(others => '0');


begin

--
-- These are debugging signals
--
debug(0) <= enableADC;
debug(1) <= dataFlag;
debug(2) <= sampleSource;
debug(3) <= dataDAC(23);

-----------------------------------------------------------------------------------------------------
--------------------------------   DAC Control   ----------------------------------------------------
-----------------------------------------------------------------------------------------------------
AD5791_SPI: SPI_Driver
generic map(
		CPOL => '1',
		CPHA => '0',
		ORDER => '1',
		SYNC_POL => '0',
		TRIG_SYNC => '0',
		TRIG_EDGE => '0',
		ASYNC_WIDTH => 2,
		ASYNC_POL => '0',
		MAX_NUM_BITS => DAC_DATA_WIDTH)
port map( 
		clk => clk,
		SPI_period => spiPeriod,
		numBits => DAC_DATA_WIDTH,
		dataReceived => open,	--data that has been received
		dataReady => open,
		SDOUT => DAC_SDOUT,--To be connected to DAC SDIN
		SDIN => '0',			
		SYNC => DAC_SYNC,--SYNC frame
		SCLK => DAC_SCLK,	--serial clock
		ASYNC => open,
		syncDelay => syncDelay,
		dataToSend => dataDAC,
		trigIn => trigDAC,
		enable => '1',
		busy => busyDAC);
			
--
-- The LDAC signal is not currently used
--
DAC_LDAC <= '0';

--
-- Allows for the use of an external signal to control the switching action if a switch is used
--
DAC_SWITCH <= (autoSwitchDAC and (not(useExternalSwitch) or EXT_SWITCH)) when autoFlag = '1' else manSwitchDAC;	

			
--
-- The following statements determines what data should be sent to the DAC based on the sample code.
-- If the LSB of the sample code is 0, then the PID output is sent to the DAC.  Otherwise, the sample
-- value from the sample generator is sent directly to the DAC.
--
sampleOutDAC <= 	sampleOut when (sampleOut >= 0 and sampleOut <= (2**20-1)) else
						0 when sampleOut < 0 else
						(2**20-1) when sampleOut > (2**20-1);
outDAC1 <= std_logic_vector(outPID(19 downto 0)) when sampleCode(0) = '0' else 
			  std_logic_vector(to_unsigned(sampleOutDAC,20));			  
trigDAC1 <= sampleTrig when sampleCode(0) = '1' else trigPID;

--
-- This statement limits the output of the DAC to the min and max values specified by the user
--
outDAC2 <= 	outDAC1 when (unsigned(outDAC1) <= maxValueDAC) and (unsigned(outDAC1) >= minValueDAC) else
				std_logic_vector(maxValueDAC) when (unsigned(outDAC1) > maxValueDAC) else
				std_logic_vector(minValueDAC);
					

--
-- Encodes the voltage data correctly for an AD5791 DAC
--				
outDAC <= X"1" & outDAC2 when overrideStateDAC = 0 else X"1" & dacOffValue;
	
--
-- Data is finally sent to the DAC depending on whether or not the controller  is in
-- automatic or manual mode
--
dataDAC <= outDAC when autoFlag = '1' else manDataDAC;
trigDAC <= (trigDAC1 or manTrigDAC) when overrideStateDAC = 0 else overrideTrigDAC;
			
--
-- This process is used to override the usual DAC output value in order to shut off
-- the MOSFETs used to control the DAC.  On either the overrideDAC signal or a falling
-- edge of the external switch (if using), the process issues a write trigger as soon as
-- possible.  When overrideStateDAC /= 0 this trigger supersedes all others.
--
DAC_Override: process(clk) is
begin
	if rising_edge(clk) then
		OV: case(overrideStateDAC) is
			--
			-- Idle state
			--
			when 0 =>
				if overrideDAC = '1' or (extSync = "10" and useExternalSwitch = '1') then
					overrideStateDAC <= 1;
				else
					overrideTrigDAC <= '0';
				end if;
				
			--
			-- Wait for DAC to be available for sending data
			--
			when 1 =>
				if busyDAC = '0' then
					overrideTrigDAC <= '1';
					overrideStateDAC <= 2;
				end if;
				
			--
			-- Wait one clock cycle for trigger to be issued
			--
			when 2 =>
				overrideTrigDAC <= '0';
				overrideStateDAC <= overrideStateDAC + 1;
				
			--
			-- Wait for reset signal to be issued
			--
			when 3 =>
				if reset = '1' then
					overrideStateDAC <= 0;
				end if;
		end case;
	end if;
end process;			
			
			
-----------------------------------------------------------------------------------------------------
--------------------------------   ADC Control   ----------------------------------------------------
-----------------------------------------------------------------------------------------------------			
ADS127L01_SPI: SPI_Driver
	generic map(
			CPOL => '0',
			CPHA => '1',
			ORDER => '1',
			SYNC_POL => '0',
			TRIG_SYNC => '1',
			TRIG_EDGE => '0',
			ASYNC_WIDTH => 0,
			ASYNC_POL => '0',
			MAX_NUM_BITS => ADC_DATA_WIDTH)
	port map( 
			clk => clk,
			SPI_period => spiPeriod,
			numBits => ADC_DATA_WIDTH,
			dataReceived => dataADC,
			dataReady => dataReadyADC,
			SDOUT => ADC_SDOUT,
			SDIN => ADC_SDIN,			
			SYNC => ADC_SYNC,
			SCLK => ADC_SCLK,
			ASYNC => open,
			syncDelay => 0,
			dataToSend => X"000000",
			trigIn => ADC_DRDY,
			enable => enableADC,
			busy => open);
			
CountSampleRate: Count_Edges
	generic map(
		EDGE_TYPE => '0',
		MAX_COUNT => MAX_COUNT)
	port map(
		clk => clk,
		edgeIn => ADC_DRDY,
		countOut => samplePeriodADC);
-----------------------------------------------------------------------------------------------------
--------------------------------   PID Control   ----------------------------------------------------
-----------------------------------------------------------------------------------------------------
--
-- Measurement and control signals are signed values
--
measurement <= signed(dataADC);
control <= to_signed(sampleOut,ADC_DATA_WIDTH);
--
-- The PID controller is enabled only when the LSB of the sample code is '0'
--
enablePID <= sampleGenEnabled when sampleCode(0) = '0' else '0';
--
-- Note the encoding of the loop register
--
loopRegFixed <= divisorPID & Kd & Ki & Kp;
--
-- This is the code corresponding to 0 V and changes depending on the DAC output mode
--
dacZero <= DAC_ZERO_UNI when dacMode = '0' else DAC_ZERO_BI;



PID: PID_Controller
generic map(
	MEAS_WIDTH => ADC_DATA_WIDTH,
	CNTL_WIDTH => ADC_DATA_WIDTH,
	OUT_WIDTH => 20,
	K_WIDTH => K_WIDTH)
port map(
	clk => clk,
	enable => enablePID,
	measureIn => measurement,
	controlIn => control,
	measReady => dataReadyADC,
	
	polarity => polarity,
	useFixed => useFixed,
	loopRegFixed => loopRegFixed,
	loopRegVary => loopRegVary,
	
	dacZero => dacZero,
	minValueIn => minValueDAC,
	maxValueIn => maxValueDAC,

	errorOut => errorOut,
	pidOut => outPID,
	trigOut => trigPID);


-----------------------------------------------------------------------------------------------------
------------------------------   Handle Memory Signals   --------------------------------------------
-----------------------------------------------------------------------------------------------------
--
-- These two statements determine what data should be saved depending on the transmission type.
--
dataToSave <= 	(X"00" & dataADC) 						when transmitType = 0 else
					(X"00" & outDAC) 							when transmitType = 1 else
					(X"00" & std_logic_vector(control)) when transmitType = 2 else
					errorOut 									when transmitType = 3 else
					(X"00" & dataADC);
memTrig <= 		dataReadyADC 								when transmitType = 0 else
					trigDAC1 									when transmitType = 1 else
					sampleTrig 									when transmitType = 2 else
					dataReadyADC;
					
--
-- This process is used simply to route the memory signals depending on what process
-- is asking for/writing data.
--					
MemRoute: process(clk) is
begin
	if rising_edge(clk) then
		if memSampleReadTrig = '1' then
			memReadTrig <= '1';
			memReadAddr <= memSampleReadAddr;
		elsif trigReadADC = '1' then
			memReadTrig <= '1';
			memReadAddr <= addrReadADC;
		else
			memReadTrig <= '0';
		end if;	--end memSampleReadTrig
		
		if memTrigWriteSerial = '1' then
			memWriteTrig <= '1';
			memWriteAddr <= memAddrWriteSerial;
			memWriteData <= memDataWriteSerial;
		elsif trigWriteADC = '1' then
			memWriteTrig <= '1';
			memWriteAddr <= addrWriteADC;
			memWriteData <= dataWriteADC;
		else
			memWriteTrig <= '0';
		end if;	--end memTrigWriteSerial
		
		if transmitTrigADC = '1' then
			dataToTransmit <= dataToTransmitADC;
			transmitTrig <= '1';
		elsif transmitTrigSerial = '1' then
			dataToTransmit <= dataToTransmitSerial;
			transmitTrig <= '1';
		else
			transmitTrig <= '0';
		end if;
		
	end if;	--end rising_edge(clk)
end process;
					

ADC_MemRW: ADCMemRWControl
	generic map(
		ID => ID,
		DATA_LOCATION => DATA_LOCATION)
	port map(
		clk => clk,
		dataIn => dataToSave,
		dataReady => memTrig,
		numSamples => numSamples,
		numMemSamp => numMemSamples,
		trigManual => manTrigADC,
		enableIn => sampleGenEnabled,
		enableOut => enableADC,
		
		sampleStep => sampleStep,
		useSampleLimits => useSampleLimits,
		minSample => minSample,
		maxSample => maxSample,
		numReqSamples => numReqSamples,
		
		trigWrite => trigWriteADC,
		addrWrite => addrWriteADC,
		dataWrite => dataWriteADC,
		trigRead => trigReadADC,
		addrRead => addrReadADC,
		dataRead => memReadData,
		dataValid => memDataReady,
		
		memReadTypeIn => memReadType,
		startTransmit => startTransmit,
		dataToTransmit => dataToTransmitADC,
		trigTransmit => transmitTrigADC,
		transmitBusy => transmitBusy);
		
-----------------------------------------------------------------------------------------------------
-------------------   Trigger detection and global timing   -----------------------------------------
-----------------------------------------------------------------------------------------------------
--
-- Synchronously detects external start trigger
--
StartTrigDetect: process(clk) is
begin
	if rising_edge(clk) then
		trigSync <= (trigSync(0),trigIn);
		extSync <= (extSync(0),EXT_SWITCH);
	end if;
end process;

reset <= resetMan or resetAuto;

--
-- This process counts clock edges and ultimately controls when the DAC output is switched off.
-- At offTime the DAC override process is initiated.
--
SeqCounter: process(clk) is
begin
	if rising_edge(clk) then
		if (trigSync = "01" or manTrigRamp = '1') and trigCount = 0 and enableTrig = '1' then		--Synchronous start of sequence
			trig <= '1';
			trigCount <= trigCount + 1;
			autoSwitchDAC <= '1';
		elsif trigCount > 0 and trigCount < offTime then
			trigCount <= trigCount + 1;
			trig <= '0';
		elsif trigCount = offTime then
			trigCount <= trigCount + 1;
			autoSwitchDAC <= '0';
			overrideDAC <= '1';
		elsif trigCount > offTime and trigCount < (offTime + trigHoldOff) then
			trigCount <= trigCount + 1;
			overrideDAC <= '0';
		elsif trigCount = (offTime + trigHoldOff) then
			trigCount <= (others => '0');
			resetAuto <= '1';
		else
			trig <= '0';
			trigCount <= (others => '0');
			autoSwitchDAC <= '0';
			resetAuto <= '0';
		end if;
	end if;
end process;

SampleGen: SampleGenerator 
	generic map(
		ID => ID,
		SAMPLE_LOCATION => SAMPLE_LOCATION,
		LOOP1_LOCATION => LOOP1_LOCATION,
		LOOP2_LOCATION => LOOP2_LOCATION)
	port map(
		clk => clk,
		trigIn => trig,
		reset => reset,
		numSamples => numSamples,
		updateTime => updateTime,
		sampleSource => sampleSource,
		
		rampValues => rampValues,
		rampBndyTimes => rampBndyTimes,
		rampRates => rampRates,
		rampCodes => rampCodes,
		
		memReadTrig => memSampleReadTrig,
		memDataReady => memDataReady,
		memAddr => memSampleReadAddr,
		memData => memReadData,
		
		sampleOut => sampleOut,
		sampleCode => sampleCode,
		loopReg => loopRegVary,
		sampleTrig => sampleTrig,
		genEnabled => sampleGenEnabled);

-----------------------------------------------------------------------------------------------------
------------------------------   Parse Serial Data   ------------------------------------------------
-----------------------------------------------------------------------------------------------------
--
-- cmdData instructs the following process on what to do with the incoming serial data.
--
-- Bits 31 downto 30 are used as an address and distinguish between different controllers.
--
-- Bits 29 downto 28 are used to determine whether or not to read/write to a parameter or to
-- memory.  Values are: "00" => write parameter, "01" => write to memory, "10" => read parameter,
-- "11" => not used.  The remaining bits differ depending on whether or not we are addressing the
-- memory or parameters.
--
-- MEMORY
-- Bits 27 downto 26 select a memory write mode.  "00" => write to sample location,
-- "01" => write to the first loop parameter location, "10" => write to the second loop parameter location,
-- "11" => not used here, but is reserved for writing to and retrieving data from memory.
--
-- Bits 25 downto 24 are not used.
--
-- The remaining bits 23 downto 0 are used directly as a memory address.
--
-- PARAMETER 
-- Bits 27 downto 24 are used to separate parameters into sensible categories.
--
-- Bits 23 downto 16 are currently not used.
--
-- Bits 15 downto 8 are used to select the parameter to address.  For array-valued parameters
-- bits 7 downto 0 are interpreted as the array index
--
-- NOTE ON RW procedure and friends
-- The RW/RWS/RWU procedures are used to provide a unified framework for writing to and reading from
-- various parameters in the controller.  If numFlag = '0' and cmdData(29) = '0', then RW raises numFlag.  This tells the serial
-- controller process in the top-module to parse the next 32 bits of incoming serial data as a numerical parameter.
-- On the next dataReady flag, this numerical data is interpreted by RW and friends as a std_logic_vector, an unsigned
-- value, or a signed value.  When cmdData(29) is '1', then the parameter is loaded into dataToTransmitSerial and the 
-- serial transmission trigger is raised to transmit this data back to the PC via serial.
--
ProcessSerialData: process(clk) is
begin
	if rising_edge(clk) then
		if dataReady = '1' and ((ENABLE_ID and cmdData(31 downto 30) = std_logic_vector(ID)) or (not ENABLE_ID)) then
			if cmdData(28) = '0' then
				--
				-- Parse parameters
				--
				Destination: case cmdData(27 downto 24) is
					--
					-- Manual parameters
					--
					when X"0" =>
						autoFlag <= '0';
						ManDestination: case cmdData(15 downto 8) is
							when X"00" => rw(cmdData(29),dataFlag,numData,manDataDAC,dataToTransmitSerial,transmitTrigSerial);
							when X"01" => manTrigDAC <= '1';	
							when X"02" => manSwitchDAC <= cmdData(0);
							when X"03" => manTrigADC <= '1';
							when others => null;
						end case;	--End ManDestination
					--
					-- Software triggers
					--
					when X"1" =>
						autoFlag <= '1';
						SoftwareTriggers: case cmdData(15 downto 8) is
							when X"00" => resetMan <= '1';
							when X"01" => manTrigRamp <= '1';
							when X"02" => startTransmit <= '1';memReadType <= unsigned(cmdData(1 downto 0));
							when others => null;
						end case;	--End SoftwareTriggers
						
					--
					-- Global settings
					--
					when X"2" =>
						autoFlag <= '1';
						GlobalParams: case cmdData(15 downto 8) is
							when X"00" => rwu(cmdData(29),dataFlag,numData,spiPeriod,dataToTransmitSerial,transmitTrigSerial);						--SPI period
							when X"01" => rwu(cmdData(29),dataFlag,numData,transmitType,dataToTransmitSerial,transmitTrigSerial);					--Type of data to save and transmit
							when X"02" => rw(cmdData(29),dataFlag,numData,enableTrig,dataToTransmitSerial,transmitTrigSerial);						--Allow trigger to start sequence?
							when X"03" => rwu(cmdData(29),dataFlag,numData,offTime,dataToTransmitSerial,transmitTrigSerial);						--Off time
							when X"04" => ADC_START <= cmdData(0);																										--Signal to send to ADC_START pin
							when X"05" => dataToTransmitSerial <= std_logic_vector(to_unsigned(samplePeriodADC,32));transmitTrigSerial <= '1';--Sends ADC sample period back to PC
							when others => null;
						end case;	--End GlobalParams
						
					--
					-- DAC settings
					--
					when X"3" =>
						autoFlag <= '1';
						DACParams: case cmdData(15 downto 8) is
							when X"00" => rwu(cmdData(29),dataFlag,numData,minValueDAC,dataToTransmitSerial,transmitTrigSerial);			--Minimum value that DAC can write
							when X"01" => rwu(cmdData(29),dataFlag,numData,maxValueDAC,dataToTransmitSerial,transmitTrigSerial);			--Maximum value that DAC can write
							when X"02" => rwu(cmdData(29),dataFlag,numData,syncDelay,dataToTransmitSerial,transmitTrigSerial);				--SYNC delay after data
							when X"03" => rw(cmdData(29),dataFlag,numData,dacMode,dataToTransmitSerial,transmitTrigSerial);					--DAC mode uni- or bi-polar
							when X"04" => rw(cmdData(29),dataFlag,numData,dacOffValue,dataToTransmitSerial,transmitTrigSerial);			--Value to write to DAC on falling edge of external switch
							when X"05" => rw(cmdData(29),dataFlag,numData,useExternalSwitch,dataToTransmitSerial,transmitTrigSerial);	--Use external switch?
							when others => null;
						end case;	--End DACParams
						
					--
					-- Ramp parameters
					--
					when X"4" =>
						autoFlag <= '1';
						RampParams: case cmdData(15 downto 8) is
							when X"00" => rw(cmdData(29),dataFlag,numData,sampleSource,dataToTransmitSerial,transmitTrigSerial);			--Sample source
							when X"01" => rwu(cmdData(29),dataFlag,numData,updateTime,dataToTransmitSerial,transmitTrigSerial);			--Sample update time
							when X"02" => rwu(cmdData(29),dataFlag,numData,numSamples,dataToTransmitSerial,transmitTrigSerial);			--Number of samples
							when others => null;
						end case;	--End RampParams
					
					--
					-- Linear ramp array-valued parameters
					--
					when X"5" =>
						autoFlag <= '1';
						LinearRamps: case cmdData(15 downto 8) is
							when X"00" => rws(cmdData(29),dataFlag,numData,rampValues,cmdData(7 downto 0),dataToTransmitSerial,transmitTrigSerial);		--Ramp values
							when X"01" => rwu(cmdData(29),dataFlag,numData,rampBndyTimes,cmdData(7 downto 0),dataToTransmitSerial,transmitTrigSerial);	--Ramp boundary times
							when X"02" => rws(cmdData(29),dataFlag,numData,rampRates,cmdData(7 downto 0),dataToTransmitSerial,transmitTrigSerial);		--Ramp rates (derivatives)
							when X"03" => rwu(cmdData(29),dataFlag,numData,rampCodes,cmdData(7 downto 0),dataToTransmitSerial,transmitTrigSerial);		--Ramp codes
							when others => null;
						end case;	--End LinearRamps
					
					--
					-- PID controller parameters
					--
					when X"6" =>
						autoFlag <= '1';
						PIDValues: case cmdData(15 downto 8) is
							when X"00" => rw(cmdData(29),dataFlag,numData,polarity,dataToTransmitSerial,transmitTrigSerial);	--PID polarity
							when X"01" => rw(cmdData(29),dataFlag,numData,Kp,dataToTransmitSerial,transmitTrigSerial);			--Proportional gain coefficient
							when X"02" => rw(cmdData(29),dataFlag,numData,Ki,dataToTransmitSerial,transmitTrigSerial);			--Integral gain coefficient
							when X"03" => rw(cmdData(29),dataFlag,numData,Kd,dataToTransmitSerial,transmitTrigSerial);			--Derivative gain coefficient
							when X"04" => rw(cmdData(29),dataFlag,numData,divisorPID,dataToTransmitSerial,transmitTrigSerial);	--Overall PID divisor		
							when X"05" => rw(cmdData(29),dataFlag,numData,useFixed,dataToTransmitSerial,transmitTrigSerial);	--Use fixed PID gains or use varying ones
							when others => null;
						end case;	--End PIDValues
						
					--
					-- Memory parameters
					--
					when X"7" =>
						autoFlag <= '1';
						OtherSignals: case cmdData(15 downto 8) is
							when X"00" => rwu(cmdData(29),dataFlag,numData,numMemSamples,dataToTransmitSerial,transmitTrigSerial);				--Maximum number of samples to read
							when X"01" => rw(cmdData(29),dataFlag,numData,useSampleLimits,dataToTransmitSerial,transmitTrigSerial);				--Use sample limits when reading memory?
							when X"02" => rwu(cmdData(29),dataFlag,numData,minSample,dataToTransmitSerial,transmitTrigSerial);						--Starting sample for reading
							when X"03" => rwu(cmdData(29),dataFlag,numData,maxSample,dataToTransmitSerial,transmitTrigSerial);						--Ending sample for reading
							when X"04" => rwu(cmdData(29),dataFlag,numData,sampleStep,dataToTransmitSerial,transmitTrigSerial);					--Step size when reading
							when X"05" => dataToTransmitSerial <= std_logic_vector(to_unsigned(numReqSamples,32));transmitTrigSerial <= '1';	--Sends number of requested samples back to PC
							when others => null;
						end case;
					when others => null;
				end case;	--End Destination
			elsif cmdData(29 downto 28) = "01" then
				if dataFlag = '0' then
					dataFlag <= '1';
					MemCase: case cmdData(27 downto 26) is
						--
						-- Write to sample location
						--
						when "00" => 
							if ENABLE_ID then
								memAddrWriteSerial <= MEM_ADDR_PAD_VALUE & ID & SAMPLE_LOCATION & unsigned(cmdData(MEM_ADDR_USER_WIDTH-1 downto 0));
							else
								memAddrWriteSerial <= MEM_ADDR_PAD_VALUE & SAMPLE_LOCATION & unsigned(cmdData(MEM_ADDR_USER_WIDTH-1 downto 0));
							end if;
						--
						-- Write to first loop location
						--
						when "01" => 
							if ENABLE_ID then
								memAddrWriteSerial <= MEM_ADDR_PAD_VALUE & ID & LOOP1_LOCATION & unsigned(cmdData(MEM_ADDR_USER_WIDTH-1 downto 0));
							else
								memAddrWriteSerial <= MEM_ADDR_PAD_VALUE & LOOP1_LOCATION & unsigned(cmdData(MEM_ADDR_USER_WIDTH-1 downto 0));
							end if;
						--
						-- Write to second loop location
						--
						when "10" => 
							if ENABLE_ID then
								memAddrWriteSerial <= MEM_ADDR_PAD_VALUE & ID & LOOP2_LOCATION & unsigned(cmdData(MEM_ADDR_USER_WIDTH-1 downto 0));
							else
								memAddrWriteSerial <= MEM_ADDR_PAD_VALUE & LOOP2_LOCATION & unsigned(cmdData(MEM_ADDR_USER_WIDTH-1 downto 0));
							end if;
						
						--
						-- Never write to data location "11"
						--
						when others => null;
					end case;
				else
					memTrigWriteSerial <= '1';
					memDataWriteSerial <= numData;
					dataFlag <= '0';
				end if;

			end if;	--End manual/automatic if
		else
			--
			-- All trigger signals need to be lowered
			--
			manTrigDAC <= '0';
			manTrigADC <= '0';
			manTrigRamp <= '0';
			
			startTransmit <= '0';
			memTrigWriteSerial <= '0';
			resetMan <= '0';
			
			transmitTrigSerial <= '0';
		end if;	--End dataReady = '1'	
	end if;	--End rising_edge(clk)
end process;



end Behavioral;

