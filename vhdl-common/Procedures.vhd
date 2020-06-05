library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.Constants.all;

--
-- This package defines procedures that are used to allow for
-- simple and compact writing of statements that handle reading
-- and writing of parameters to and from the PC using the SerialCommunication
-- component.
--
package Procedures is

--
-- Unsigned and signed integer parameter parsing
--
procedure rwu(
	signal readFlag	:	in std_logic;	
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout integer;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic);
	
procedure rws(
	signal readFlag	:	in std_logic;	
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout integer;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic);
	
--
-- Unsigned and signed integer array parameter parsing
--
procedure rwu(
	signal readFlag	:	in std_logic;	
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout int_array;
	signal idx			:	in std_logic_vector;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic);
	
procedure rws(
	signal readFlag	:	in std_logic;	
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout int_array;
	signal idx			:	in std_logic_vector;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic);
	
--
-- std_logic_vector parameter parsing
--
procedure rw(
	signal readFlag	:	in std_logic;	
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout std_logic_vector;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic);

--
-- std_logic parameter parsing
--
procedure rw(
	signal readFlag	:	in std_logic;	
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout std_logic;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic);

--
-- Unsigned unsigned parameter parsing
--	
procedure rwu(
	signal readFlag	:	in std_logic;	
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout unsigned;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic);

--
-- Signed signed parameter parsing
--	
procedure rws(
	signal readFlag	:	in std_logic;	
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout signed;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic);
	

end Procedures;

--------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------

package body Procedures is

--
-- Unsigned integer parameter parsing
--
procedure rwu(
	signal readFlag	:	in std_logic;
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout integer;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic) is
begin
	if readFlag = '0' then
		if flagIn = '0' then
			flagIn <= '1';
		else
			flagIn <= '0';
			param <= to_integer(unsigned(dataIn));
		end if;
	else
		dataOut <= std_logic_vector(to_unsigned(param,dataOut'length));
		trigOut <= '1';
	end if;
end rwu;

--
-- Signed integer parameter parsing
--
procedure rws(
	signal readFlag	:	in std_logic;
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout integer;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic) is
begin
	if readFlag = '0' then
		if flagIn = '0' then
			flagIn <= '1';
		else
			flagIn <= '0';
			param <= to_integer(signed(dataIn));
		end if;
	else
		dataOut <= std_logic_vector(to_signed(param,dataOut'length));
		trigOut <= '1';
	end if;
end rws;

--
-- Unsigned integer array parameter parsing
--
procedure rwu(
	signal readFlag	:	in std_logic;
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout int_array;
	signal idx			:	in std_logic_vector;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic) is
begin
	if readFlag = '0' then
		if flagIn = '0' then
			flagIn <= '1';
		else
			flagIn <= '0';
			param(slvToInt(idx)) <= to_integer(unsigned(dataIn));
		end if;
	else
		dataOut <= std_logic_vector(to_unsigned(param(slvToInt(idx)),dataOut'length));
		trigOut <= '1';
	end if;
end rwu;

--
-- Signed integer parameter parsing
--
procedure rws(
	signal readFlag	:	in std_logic;
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout int_array;
	signal idx			:	in std_logic_vector;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic) is
begin
	if readFlag = '0' then
		if flagIn = '0' then
			flagIn <= '1';
		else
			flagIn <= '0';
			param(slvToInt(idx)) <= to_integer(signed(dataIn));
		end if;
	else
		dataOut <= std_logic_vector(to_signed(param(slvToInt(idx)),dataOut'length));
		trigOut <= '1';
	end if;
end rws;

--
-- std_logic_vector parameter parsing
--
procedure rw(
	signal readFlag	:	in std_logic;
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout std_logic_vector;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic) is
begin
	if readFlag = '0' then
		if flagIn = '0' then
			flagIn <= '1';
		else
			flagIn <= '0';
			param <= dataIn(param'length-1 downto 0);
		end if;
	else
		dataOut <= (dataOut'length-1 downto param'length => '0') & param;
		trigOut <= '1';
	end if;
end rw;

--
-- std_logic parameter parsing
--
procedure rw(
	signal readFlag	:	in std_logic;
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout std_logic;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic) is
begin
	if readFlag = '0' then
		if flagIn = '0' then
			flagIn <= '1';
		else
			flagIn <= '0';
			param <= dataIn(0);
		end if;
	else
		dataOut <= (0 => param, others => '0');
		trigOut <= '1';
	end if;
end rw;

--
-- Unsigned parameter parsing
--
procedure rwu(
	signal readFlag	:	in std_logic;
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout unsigned;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic) is
begin
	if readFlag = '0' then
		if flagIn = '0' then
			flagIn <= '1';
		else
			flagIn <= '0';
			param <= unsigned(dataIn(param'length-1 downto 0));
		end if;
	else
		dataOut <= (dataOut'length-1 downto param'length => '0') & std_logic_vector(param);
		trigOut <= '1';
	end if;
end rwu;

--
-- Signed parameter parsing
--
procedure rws(
	signal readFlag	:	in std_logic;
	signal flagIn		:	inout	std_logic;
	signal dataIn		:	in	std_logic_vector;
	signal param		:	inout signed;
	signal dataOut		:	out std_logic_vector;
	signal trigOut		:	out std_logic) is
begin
	if readFlag = '0' then
		if flagIn = '0' then
			flagIn <= '1';
		else
			flagIn <= '0';
			param <= signed(dataIn(param'length-1 downto 0));
		end if;
	else
		dataOut <= (dataOut'length-1 downto param'length => param(param'length-1)) & std_logic_vector(param);
		trigOut <= '1';
	end if;
end rws;
 
end Procedures;
