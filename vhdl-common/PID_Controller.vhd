library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.Constants.all;
use work.Procedures.all;

--
--Uses measurement and control values to implement a PID controller
--by calculating a correction to the actuator (DAC) value at each
--time step.
--
entity PID_Controller is
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
			trigOut		:	out std_logic);								--Trigger for DAC, indicating pidOut is valid
end PID_Controller;

architecture Behavioral of PID_Controller is

--
-- Define constant widths
--
constant EXP_WIDTH	:	integer	:=	40;							--Expanded width of signals
constant MULT_WIDTH	:	integer	:=	K_WIDTH + EXP_WIDTH;		--Width of multiplied signals
constant MULT_DELAY	:	integer	:=	7;								--Latency of the multiplication blocks

--
-- Multiplies a 40-bit signed value with a 16-bit unsigned value.
-- The widths of these signals must match the width of their appropriate parameters.
-- a'length = EXP_WIDTH, b'length = K_WIDTH, p'length = MULT_WIDTH
--
COMPONENT K_Multiplier
  PORT (
	 clk : in std_logic;
    a : IN STD_LOGIC_VECTOR(EXP_WIDTH-1 DOWNTO 0);
    b : IN STD_LOGIC_VECTOR(K_WIDTH-1 DOWNTO 0);
    p : OUT STD_LOGIC_VECTOR(MULT_WIDTH-1 DOWNTO 0)
  );
END COMPONENT;


signal measurement, control, propTerm, integralTerm, derivativeTerm	:	signed(EXP_WIDTH-1 downto 0)	:=	(others => '0');
signal errorSig, errorSig1, errorSig2	:	signed(EXP_WIDTH-1 downto 0)	:=	(others => '0');
signal pidProp, pidInt, pidDeriv	:	std_logic_vector(MULT_WIDTH-1 downto 0)	:=	(others => '0');
signal pidSum, pidDivide	:	signed(MULT_WIDTH-1 downto 0)	:=	(others => '0');
signal minValue, maxValue	:	signed(MULT_WIDTH-1 downto 0)	:=	(others => '0');

signal multCount	:	integer	range 0 to 7	:=	0;
signal state	:	integer range 0 to 3	:=	0;

signal Kp, Ki, Kd	:	std_logic_vector(K_WIDTH-1 downto 0)	:=	(others => '0');	--Proportional, integral, and derivative gain and divisor
signal divisor	:	integer range 0 to 2**(K_WIDTH-1)	:=	0;

begin	
	
--
-- Resize inputs to EXP_WIDTH
--
measurement <= resize(measureIn,measurement'length);
control <= resize(controlIn,control'length);

--
-- Convert the min and max values to signed values and shift them left by the divisor value.
-- As the divisor value is continuously updated in the PID process before these min and max
-- values are used, these min and max values are always up-to-date when they are called
--
minValue <= shift_left(u2s(minValueIn,dacZero,MULT_WIDTH),divisor);
maxValue <= shift_left(u2s(maxValueIn,dacZero,MULT_WIDTH),divisor);			

--
-- Calculate error signal
--
errorSig <= control - measurement when polarity = '0' else measurement - control;
errorOut <= std_logic_vector(resize(errorSig,32));

--
-- Calculate actuator stages
--
MultProp: K_Multiplier
port map (
	clk => clk,
	a => std_logic_vector(propTerm),
	b => Kp,
	p => pidProp);
	
MultInt: K_Multiplier
port map (
	clk => clk,
	a => std_logic_vector(integralTerm),
	b => Ki,
	p => pidInt);	

MultDeriv: K_Multiplier
port map (
	clk => clk,
	a => std_logic_vector(derivativeTerm),
	b => Kd,
	p => pidDeriv);

pidSum <= signed(pidProp) + signed(pidInt) + signed(pidDeriv);

--
-- This is the main PID process and provides parsing of the loop registers as well
-- as handling the timing.
--
PID_Process: process(clk) is
begin
	if rising_edge(clk) then
		PID_FSM: case state is
			--
			-- Wait-for-measurement state
			--
			when 0 =>
				multCount <= 0;
				trigOut <= '0';
				if enable = '1' and measReady = '1' then
					--
					-- Calculate the various PID terms from current and previous error signals
					--
					propTerm <= errorSig - errorSig1;
					integralTerm <= shift_right(errorSig + errorSig1,1);
					derivativeTerm <= errorSig - shift_left(errorSig1,1) + errorSig2;
					errorSig2 <= errorSig1;
					errorSig1 <= errorSig;
					
					--
					-- Decide which loop register to use and parse the values
					--
					if useFixed = '1' then
						Kp <= loopRegFixed(K_WIDTH-1 downto 0);
						Ki <= loopRegFixed(2*K_WIDTH-1 downto K_WIDTH);
						Kd <= loopRegFixed(3*K_WIDTH-1 downto 2*K_WIDTH);
						divisor <= slvToInt(loopRegFixed(4*K_WIDTH-1 downto 3*K_WIDTH));
					else
						Kp <= loopRegVary(K_WIDTH-1 downto 0);
						Ki <= loopRegVary(2*K_WIDTH-1 downto K_WIDTH);
						Kd <= loopRegVary(3*K_WIDTH-1 downto 2*K_WIDTH);
						divisor <= slvToInt(loopRegVary(4*K_WIDTH-1 downto 3*K_WIDTH));
					end if;
					state <= state + 1;	
					
				elsif enable = '0' then
					--
					-- If the PID controller is not enabled, then set all signals to zero
					--
					propTerm <= (others => '0');
					integralTerm <= (others => '0');
					derivativeTerm <= (others => '0');
					pidDivide <= (others => '0');
					pidOut <= dacZero;
					errorSig1 <= (others => '0');
					errorSig2 <= (others => '0');
				end if;
				
			--
			-- Insert delay for multipliers
			--
			when 1 =>
				if multCount < MULT_DELAY then
					multCount <= multCount + 1;
				else
					--
					-- Add the actuator correction pidSum to the old value of the actuator pidDivide.
					-- Note that this addition takes place BEFORE the right-shift by divisor bits
					--
					pidDivide <= pidDivide + pidSum;
					multCount <= 0;
					state <= state + 1;
				end if;
			
			--
			-- Check limits and produce output
			--
			when 2 =>
				state <= state + 1;
				if pidDivide < minValue then
					pidDivide <= minValue;
					pidOut <= minValueIn;
				elsif pidDivide > maxValue then
					pidDivide <= maxValue;
					pidOut <= maxValueIn;
				else
					pidOut <= s2u(shift_right(pidDivide,divisor),dacZero,OUT_WIDTH);
				end if;
			
			--
			-- Generate trigger indicating a valid output
			--
			when 3 =>
				trigOut <= '1';
				state <= 0;
					
			when others => null;
		end case;	--end PID_FSM
	end if;
end process;


end Behavioral;

