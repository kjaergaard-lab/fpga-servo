library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.Constants.all;
use work.Procedures.all;

--
-- This is a simple module to count the number of clock cycles
-- between edges of a signal.  Useful for determining an input
-- clock rate
--
entity Count_Edges is
	generic(	EDGE_TYPE	:	std_logic;	--'0' for falling edge, '1' for rising edge
				MAX_COUNT	:	integer);	--The maximum count expected, used for setting the count integer range
	port(	clk		:	in std_logic;		--Input clock used for counting
			edgeIn	:	in	std_logic;		--Input signal whose edges we wish to count
			countOut	:	out integer);		--The number of clock cycles between edges
end Count_Edges;

architecture Behavioral of Count_Edges is

signal edgeSync	:	std_logic_vector(1 downto 0)	:=	"00";
signal count	:	integer range 0 to MAX_COUNT-1 	:=	0;

begin

--
-- This creates a two-bit signal that indicates the presence of an edge
--
SyncGen: process(clk) is
begin
	if rising_edge(clk) then
		edgeSync <= edgeSync(0) & edgeIn;
	end if;
end process;

--
-- This process counts the number of clock cycle between edges
-- Note that the output count is incremented by one before being
-- written out to ensure the correct value
--
CountProcess: process(clk) is
begin
	if rising_edge(clk) then
		if (EDGE_TYPE = '0' and edgeSync = "10") or (EDGE_TYPE = '1' and edgeSync = "01") then
			countOut <= count + 1;
			count <= 0;
		else
			count <= count + 1;
		end if;
	end if;	--end rising_edge
end process;


end Behavioral;

