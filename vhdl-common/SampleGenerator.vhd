library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.Constants.all;
use work.Procedures.all;

--
-- This component generates samples to be used by the PID controller either
-- from memory or from an internal ramp generator.
--
entity SampleGenerator is
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
end SampleGenerator;

architecture Behavioral of SampleGenerator is

signal state				:	integer range 0 to 3	:=	0;
signal updateCount		:	integer	:=	0;
signal updateTrig			:	std_logic	:=	'0';
signal sampleCount		:	integer	:=	0;

signal rampSampleOut		:	integer	:=	0;
signal rampSampleCode	:	std_logic_vector(7 downto 0)	:=	X"00";
signal rampIdx				:	integer range 0 to MAX_NUM_RAMPS-1 :=	0;


signal memState			:	integer range 0 to 15	:=	0;
signal memSampleOut		:	integer	:=	0;
signal memSampleCode		:	std_logic_vector(7 downto 0)	:=	X"00";
signal memDelay			:	integer	:=	0;
signal memDelayCount		:	integer	:=	0;
signal memDelayEnable	:	std_logic	:=	'0';

signal loopReg1, loopReg2	:	std_logic_vector(31 downto 0)	:=	(others => '0');

begin

--
-- The Update process outs new samples and new sample codes
-- each time that the updateCount counter reaches updateTime
--
Update: process(clk,trigIn) is
begin
	if rising_edge(clk) then
		SM: case state is
			--Idle state
			when 0 =>
				sampleCount <= 0;
				sampleTrig <= '0';
				if reset = '1' then
--					updateTrig <= '1';
					genEnabled <= '0';
				elsif trigIn = '1' then
					state <= state + 1;
					updateCount <= 1;
					updateTrig <= '0';
					genEnabled <= '1';
				else
					updateCount <= 0;
					updateTrig <= '0';
					genEnabled <= '0';
				end if;
				
			--Shift data state
			when 1 =>
				state <= state + 1;
				sampleTrig <= '1';
				
				if sampleSource = '0' then
					sampleOut <= rampSampleOut;
					sampleCode <= rampSampleCode;
					sampleCount <= sampleCount + 1;
				elsif memSampleCode(7) = '0' and memDelayEnable = '0' then
					sampleOut <= memSampleOut;
					sampleCode <= memSampleCode;
					loopReg <= loopReg2 & loopReg1;
					memDelayEnable <= '0';
					sampleCount <= sampleCount + 1;
				elsif memSampleCode(7) = '1' then
					memDelay <= memSampleOut;
					sampleCount <= sampleCount + 1;
				end if;
				
			--Wait for updateTime
			when 2 =>
				sampleTrig <= '0';
				if updateCount = 1 then
					updateCount <= updateCount + 1;
					if memDelayEnable = '0' then
						updateTrig <= '1';
						if memSampleCode(7) = '1' then
							memDelayEnable <= '1';
						end if;
					end if;
					
				elsif updateCount < (updateTime - 1) then
					updateCount <= updateCount + 1;
					updateTrig <= '0';
				else
					updateCount <= 1;
					if sampleCount < numSamples then
						state <= 1;
					else
						state <= 0;
					end if;
					
					if memDelayEnable = '1' and memDelayCount < (memDelay - 2) then
						memDelayCount <= memDelayCount + 1;
					else
						memDelayEnable <= '0';
						memDelayCount <= 0;
					end if;
				end if;
				
			when others => null;
		end case;	--end case SM
	end if;	--end rising_edge(clk)
end process;


--
-- The LinearRamp process calculates linear ramps
--
LinearRamp: process(clk) is
begin
	if rising_edge(clk) then	
		if reset = '1' then
			rampIdx <= 0;
			rampSampleOut <= rampValues(0);
			rampSampleCode <= std_logic_vector(to_unsigned(rampCodes(0),8));
		elsif updateTrig = '1' and sampleSource = '0' then
			if sampleCount = rampBndyTimes(rampIdx) then
				rampSampleOut <= rampValues(rampIdx);
				rampSampleCode <= std_logic_vector(to_unsigned(rampCodes(rampIdx),8));
			elsif sampleCount > rampBndyTimes(rampIdx) and sampleCount < (rampBndyTimes(rampIdx+1)-1) then
				rampSampleOut <= rampSampleOut + rampRates(rampIdx);
				rampSampleCode <= std_logic_vector(to_unsigned(rampCodes(rampIdx),8));
			elsif sampleCount = (rampBndyTimes(rampIdx+1)-1) then
				rampSampleOut <= rampSampleOut + rampRates(rampIdx);
				rampIdx <= rampIdx + 1;
			else
				rampSampleOut <= rampValues(0);
				rampSampleCode <= std_logic_vector(to_unsigned(rampCodes(0),8));
				rampIdx <= 0;
			end if;	--end sampleCount
		end if;	--end updateTrig
	end if;	--end rising_edge(clk)
end process;

--
-- This process issues read triggers to the correct memory address and fetches new samples from memory
--
MemoryRamp: process(clk) is
begin
	if rising_edge(clk) then
		MemoryStateMachine: case memState is
			--
			-- Idle state
			--
			when 0 =>
				if (updateTrig = '1' and sampleSource = '1') or reset = '1' then
					--
					-- Read new sample/control signal
					--
					memReadTrig <= '1';
					memState <= memState + 1;
					if ENABLE_ID then
						memAddr <= MEM_ADDR_PAD_VALUE & ID & SAMPLE_LOCATION & to_unsigned(sampleCount,MEM_ADDR_USER_WIDTH);
					else
						memAddr <= MEM_ADDR_PAD_VALUE & SAMPLE_LOCATION & to_unsigned(sampleCount,MEM_ADDR_USER_WIDTH);
					end if;
				else
					memReadTrig <= '0';
				end if;
				
			--
			-- Reset read trigger
			--
			when 1|4|7 =>
				memReadTrig <= '0';
				memState <= memState + 1;
			
			--
			-- Assign data to sample out and sample code
			-- If variable loops are enabled, then need to read out loop registers
			--
			when 2 =>
				if memDataReady = '1' then
					memSampleOut <= to_integer(signed(memData(23 downto 0)));
					memSampleCode <= memData(MEM_DATA_WIDTH-1 downto 24);
					memState <= memState + 1;
				end if;
				
			--
			-- Read new value for loop register 1
			--
			when 3 =>
				memReadTrig <= '1';
				memState <= memState + 1;
				if ENABLE_ID then
					memAddr <= MEM_ADDR_PAD_VALUE & ID & LOOP1_LOCATION & to_unsigned(sampleCount,MEM_ADDR_USER_WIDTH);
				else
					memAddr <= MEM_ADDR_PAD_VALUE & LOOP1_LOCATION & to_unsigned(sampleCount,MEM_ADDR_USER_WIDTH);
				end if;
				
			--
			-- Assign data to loop register 1
			--
			when 5 =>
				if memDataReady = '1' then
					loopReg1 <= memData;
					memState <= memState + 1;
				end if;
				
			--
			-- Read new value for loop register 2
			--
			when 6 =>
				memReadTrig <= '1';
				memState <= memState + 1;
				if ENABLE_ID then
					memAddr <= MEM_ADDR_PAD_VALUE & ID & LOOP2_LOCATION & to_unsigned(sampleCount,MEM_ADDR_USER_WIDTH);
				else
					memAddr <= MEM_ADDR_PAD_VALUE & LOOP2_LOCATION & to_unsigned(sampleCount,MEM_ADDR_USER_WIDTH);
				end if;
				
			--
			-- Assign data to loop register 2
			--
			when 8 =>
				if memDataReady = '1' then
					loopReg2 <= memData;
					memState <= 0;
				end if;
				

			when others => null;
		end case;
	end if;
end process;

end Behavioral;

