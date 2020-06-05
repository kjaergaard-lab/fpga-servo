library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 

--
-- This package contains both constants and functions used for
-- servo control.
--
package Constants is

--
-- Type int_array is used for defining parameters for linear ramps
--
type int_array is array (integer range <>) of integer;
--
-- Type serial_data and serial_data_Array are used for sending and receiving serial data
--
subtype serial_data is std_logic_vector(31 downto 0);
type serial_data_array is array(natural range <>) of serial_data;

subtype mem_instr is std_logic_vector(2 downto 0);
type mem_instr_array is array(natural range <>) of mem_instr;

subtype burst_length is std_logic_vector(5 downto 0);
type burst_length_array is array(natural range <>) of burst_length;

subtype mem_count is std_logic_vector(6 downto 0);
type mem_count_array is array(natural range <>) of mem_count;

subtype mem_mask is std_logic_vector(3 downto 0);
type mem_mask_array is array(natural range <>) of mem_mask;

subtype mem_state is unsigned(3 downto 0);
type mem_state_array is array(natural range <>) of mem_state;

--
-- This is the baud period for the UART protocol.  I have found that
-- a 1 MHz rate is as fast as the Saturn board can handle
--
constant BAUD_PERIOD		:	integer	:= 50;	--With a 50 MHz clock, corresponds to 1 MHz
--
-- The maximum number of piece-wise linear ramps
--
constant MAX_NUM_RAMPS	:	integer	:=	8;
--
-- The maximum number of controllers to use
--
constant N_MAX				:	natural	:=	1;
--
-- The number of controllers to use
--
constant N					:	natural	:=	1;
--
-- Number of memory ports
--
constant NM					:	natural	:=	1;
--
-- Enable or disable use of ID tags in memory retrieval
--
constant ENABLE_ID		:	boolean	:=	false;

--
-- These constants are used for the DDR interface
--
constant DDR_NUM_DQ_PINS	:	integer	:=	16;
constant DDR_ADDR_WIDTH		:	integer	:=	13;
constant DDR_BANK_WIDTH		:	integer	:= 2;
constant MEM_ADDR_WIDTH		:	integer	:=	28;
constant MEM_MASK_SIZE		:	integer	:= 4;
constant NUM_MEM_BYTES		:	integer	:=	4;
constant MEM_DATA_WIDTH		:	integer	:=	8*NUM_MEM_BYTES;

--
-- These constants are for the user when addressing the memory.
-- Note that the memory is 512 Mb, and if each word stored in memory
-- is 32 bits long, that gives 16e6 addresses, which corresponds to
-- 24 bits of accessible memory
--
constant MEM_ADDR_USER_WIDTH	:	integer	:=	22;		--The width of the address field that the user can access when using variable loops
constant MEM_ADDR_TYPE_WIDTH	:	integer	:=	2;			--The width of the type field indicating what kind of data is present in the memory
constant MEM_ADDR_ID_WIDTH		:	integer	:=	2;			--The width of the ID field indicating which controller's data to access
--
-- These are used to pad the memory addresses to get the right size for the memory controller
--
constant MEM_ADDR_PAD_WIDTH	:	integer	:=	MEM_ADDR_WIDTH - MEM_ADDR_USER_WIDTH - MEM_ADDR_TYPE_WIDTH;
constant MEM_ADDR_PAD_VALUE	:	unsigned(MEM_ADDR_PAD_WIDTH-1 downto 0)	:=	(others => '0');

--
-- Type mem_data and mem_data_array are used for handling data sent to and from the DDR memory
--
subtype mem_data is std_logic_vector(31 downto 0);
type mem_data_array is array(natural range <>) of mem_data;
--
-- Type mem_addr and mem_addr_array are used for handling addresses sent to and from the DDR memory
--
subtype mem_addr is unsigned(MEM_ADDR_WIDTH-1 downto 0);
type mem_addr_array is array(natural range <>) of mem_addr;

subtype mem_addr_full is std_logic_vector(29 downto 0);
type mem_addr_full_array is array(natural range <>) of mem_addr_full;

--
-- This signals define input and output widths for the PID controller
--
constant DAC_DATA_WIDTH	:	integer	:=	24;
constant ADC_DATA_WIDTH	:	integer	:=	24;
constant K_WIDTH	:	integer	:=	16;

--
-- These functions are useful short-hands for converting between different
-- signal types
--

--
-- Converts a standard logic vector signal to an integer
--
function slvToInt(
	signal vecIn	: std_logic_vector)
	return integer;
	
--
-- Converts an unsigned value to a signed value using the given zero-value (zVal)
--
function u2s(
	uIn	:	unsigned;
	zVal	:	unsigned;
	len	:	integer)
	return signed;
	
--
-- Converts a signed value to an unsigned value using the given zero-value (zVal)
--
function s2u(
	sIn	:	signed;
	zVal	:	unsigned;
	len	:	integer)
	return unsigned;
	
function unary_and(slv : in std_logic_vector) return std_logic;
function unary_or(slv : in std_logic_vector) return std_logic;
	
end Constants;

--------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------
package body Constants is

function slvToInt(
	signal vecIn	: std_logic_vector)
	return integer is
begin
	return to_integer(unsigned(vecIn));
end slvToInt;


function u2s(
	uIn	:	unsigned;
	zVal	:	unsigned;
	len	:	integer)
	return signed is
begin
	return signed(std_logic_vector(resize(uIn,len))) - signed(std_logic_vector(resize(zVal,len)));
end u2s;


function s2u(
	sIn	:	signed;
	zVal	:	unsigned;
	len	:	integer)
	return unsigned is
begin
	return resize(unsigned(std_logic_vector(sIn + signed(std_logic_vector(resize(zVal,sIn'length))))),len);
end s2u;


function unary_and(slv : in std_logic_vector) return std_logic is
  variable res_v : std_logic := '1';  -- Null slv vector will also return '1'
begin
  for i in slv'range loop
    res_v := res_v and slv(i);
  end loop;
  return res_v;
end function;


function unary_or(slv : in std_logic_vector) return std_logic is
	variable res_v : std_logic;  -- Null slv vector will also return '1'
begin
	res_v := '0';
	for i in slv'range loop
	 res_v := res_v or slv(i);
	end loop;
	return res_v;
end function;

end Constants;
