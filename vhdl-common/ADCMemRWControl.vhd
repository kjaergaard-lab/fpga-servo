library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.Constants.all;
use work.Procedures.all;

--
--This component handles reading and writing data to memory for later
--retrieval by the PC.  It keeps track of how many samples are written to the memory
--so that the correct number of samples can be read back out by the PC.
--
entity ADCMemRWControl is
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
end ADCMemRWControl;

architecture Behavioral of ADCMemRWControl is

signal state	:	integer range 0 to 7 := 0;
signal enable, enableManual	:	std_logic	:=	'0';
signal sampleCount, sampleCountMax, sampleReadMax	:	integer	:=	0;

signal memReadType	:	unsigned(1 downto 0)	:=	"00";
signal data	:	mem_data	:=	(others => '0');

begin

--
-- numReqSamples is used to figure out if the number of samples that the PC receives is 
-- the number of samples that it should have received.  There is occasionally a difference
-- and it is probably due to lost words because of the fast UART transmission rate.
--
numReqSamples <= sampleReadMax - minSample when useSampleLimits = '1' else sampleReadMax;

--
-- This detects manual triggers and disables the write when the correct number of samples
-- has been written
--
manualTrigDetect: process(clk) is
begin
	if rising_edge(clk) then
		if trigManual = '1' then
			enableManual <= '1';
		elsif sampleCount >= numSamples then
			enableManual <= '0';
		end if;
	end if;
end process;

enable <= enableIn or enableManual;
enableOut <= enable;
--
-- This is the main process for the component.  It both writes data collected by the FPGA
-- to memory.  It also reads data for sending to the PID controller or the sample generator 
-- OR when requested by the PC.
--
SampleCollection: process(clk) is
begin
	if rising_edge(clk) then
		if enable = '1' then
			--
			-- This is the main write stage.  Data is written to the correct address determined by
			-- sample count and the given DATA_LOCATION.  
			--
			if dataReady = '1' then
				trigWrite <= '1';
				if ENABLE_ID then
					addrWrite <= MEM_ADDR_PAD_VALUE & ID & DATA_LOCATION & to_unsigned(sampleCount,MEM_ADDR_USER_WIDTH);
				else
					addrWrite <= MEM_ADDR_PAD_VALUE & DATA_LOCATION & to_unsigned(sampleCount,MEM_ADDR_USER_WIDTH);
				end if;
				dataWrite <= dataIn;
				sampleCountMax <= sampleCount + 1;
				sampleCount <= sampleCount + 1;
			else
				trigWrite <= '0';
			end if;	--end dataReady = '1'
		else
			-- 
			-- This is the read case which occurs when the PC requests data
			-- from the FPGA.  Memory is read as fast as the UART transmission
			-- allows and is streamed at once - there is no specification of the
			-- address by the PC for each request.  This makes it unsuitable for
			-- control by certain protocols like SPI without significant modification.
			--
			trigWrite <= '0';
			ReadTransmit: case state is
				when 0 =>
					--
					-- This is the wait-for-trigger stage, where the module waits for
					-- the PC to signal that it wants data.
					--
					if startTransmit = '0' or sampleStep = 0 then
						--
						-- If no startTransmit signal is sent, or the sample step is 0,
						-- then don't transmit data and reset the sampleCount
						--
						sampleCount <= 0;
					else
						if useSampleLimits = '0' then
							--
							-- If we're not using sample limits, then set the maximum read
							-- sample to the maximum sample collected OR numMemSamp
							--
							sampleCount <= 0;
							if memReadTypeIn = DATA_LOCATION then
								sampleReadMax <= sampleCountMax;
							else
								sampleReadMax <= numMemSamp;
							end if;
						else
							-- 
							-- If using sample limits, then set the minimum and maximum sample
							-- limits.
							--
							sampleCount <= minSample;
							if memReadTypeIn = DATA_LOCATION then
								if (maxSample < sampleCountMax) then
									sampleReadMax <= maxSample;
								else
									sampleReadMax <= sampleCountMax;
								end if;
							else
								if (maxSample < numMemSamp) then
									sampleReadMax <= maxSample;
								else
									sampleReadMax <= numMemSamp;
								end if;
							end if;
						end if;
						
						memReadType <= memReadTypeIn;
						state <= 1;
					end if;
					
				when 1 =>
					--
					-- Set the read address.
					--
					if ENABLE_ID then
						addrRead <= MEM_ADDR_PAD_VALUE & ID & memReadType & to_unsigned(sampleCount,MEM_ADDR_USER_WIDTH);
					else
						addrRead <= MEM_ADDR_PAD_VALUE & memReadType & to_unsigned(sampleCount,MEM_ADDR_USER_WIDTH);
					end if;
					trigRead <= '1';
					state <= state + 1;
				
				when 2 =>
					--
					-- Set the read trigger to 0
					--
					trigRead <= '0';
					state <= state + 1;
					
				when 3 =>
					--
					-- When the memory indicates that the read data is valid, latch
					-- that data into a local register
					--
					if dataValid = '1' then
						data <= dataRead;
						state <= state + 1;
					end if;
					
				when 4 =>
					--
					-- When the UART transmitter is no longer busy, send the data and
					-- increment the sampleCount
					--
					if transmitBusy = '0' then
						trigTransmit <= '1';
						dataToTransmit <= data;
						state <= state + 1;
						sampleCount <= sampleCount + sampleStep;
					end if;
					
				when 5 =>
					--
					-- Once the sampleCount exceeds the maximum read count, stop
					-- the read-back process
					--
					trigTransmit <= '0';
					if sampleCount < sampleReadMax then
						state <= 1;
					else
						state <= 0;
					end if;

				when others => null;
			end case;	--end ReadTransmit
		end if;	--end enable = '1'	
	end if;	--end rising_edge(clk)
end process;



end Behavioral;

