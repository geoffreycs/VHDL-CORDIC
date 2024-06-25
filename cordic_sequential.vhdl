-- Written by Geoffrey Stentiford for ECE383 Final project
-- April-May 2024
-- 
-- How to use:
-- Supply an active-high clock on "clk". Put angle in radians in
-- signed Q10.22 fixed point on angle signal and set "start" to true.
-- Bring "start" back to false after a clock cycle and then wait for
-- "done" to become true. At that point, "sinB" and "cosB" will
-- contain the output values in sQ10.22. The outputs will stay valid
-- and "done" will stay true until you set "start" to true. Technically,
-- the output values are not updated until the next computation completes,
-- so if you wanted to gamble, you could theoretically start a new
-- calculation and still read out values from the previous one. This
-- module uses 21 CORDIC iterations, each taking three clock cycles, so
-- adding in the pre-processing and setup, you have 66 cycles from
-- start to finish (I think) where old output values are still
-- nominally valid.
--
-- This design is quite cheap to implement compared to the concurrent
-- CORDIC module, but at the cost of taking many clock cycles. Yes,
-- this makes it slower than Xilinx's CORDIC IP core. However, that
-- one requires you to deal with the awful AXI4 bus. This does not.
--
-- Loosely based off of the example CORDIC script on Wikipedia:
-- https://en.wikipedia.org/wiki/CORDIC?useskin=vector#Software_Example_(Python)
--
-- This HDL code is multi-licensed to provide the greatest possible
-- usage. You may apply any of the following licenses to usages of this:
-- APSL-2.0, Apache-2.0, AFL-3.0, CC-BY-SA-4.0, GPL-3.0, LGPL, MPL-2.0
-- NCSA, OSL-3.0, MS-PL, EUPL-1.1, AGPL-3.0, CC0-1.0, EPL-2.0, 0BSD,
-- BSD-4-Clause, BSD-3-Clause, BSD-2-Clause, Artistic-2.0, or MIT.

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY cordic_sequential IS
    PORT (
        start : IN BOOLEAN;
        angle : IN signed(31 DOWNTO 0);
        clk : IN STD_LOGIC;
        sinB : OUT signed(31 DOWNTO 0);
        cosB : OUT signed(31 DOWNTO 0);
        done : OUT BOOLEAN
    );
END cordic_sequential;

ARCHITECTURE Behavioral OF cordic_sequential IS

    TYPE lut IS ARRAY(0 TO 20) OF NATURAL RANGE 0 TO 4194304;
    CONSTANT table : lut := (3294198, 1944679, 1027514, 521582, 261803,
    131029, 65530, 32767, 16383, 8191, 4095, 2047, 1023, 511, 255, 127,
    63, 31, 15, 7, 3);
    CONSTANT P2i : lut := (4194304, 2097152, 1048576, 524288, 262144,
    131072, 65536, 32768, 16384, 8192, 4096, 2048, 1024, 512, 256, 128,
    64, 32, 16, 8, 4);
    CONSTANT Kn : INTEGER := 2547003;
    CONSTANT ONE : INTEGER := 4194304;
    CONSTANT NGO : INTEGER := (-4194304);
    CONSTANT RPD : INTEGER := 73204;
    CONSTANT DPR : INTEGER := 240315917;
    CONSTANT HALFPI : INTEGER := 6588397;
    CONSTANT FRAC : INTEGER := 22;

    -- Guaranteed to work for all Q10.22
    FUNCTION multfix(a : INTEGER; b : INTEGER) RETURN INTEGER IS
    BEGIN
        RETURN to_integer(resize(shift_right(to_signed(a, 32) * to_signed(b, 32), FRAC), 32));
    END FUNCTION;
    FUNCTION multfix(a : signed; b : INTEGER) RETURN signed IS
    BEGIN
        RETURN resize(shift_right(resize(a, 64) * b, FRAC), 32);
    END FUNCTION;

    SIGNAL flip3, rev, sigma : INTEGER RANGE -1 TO 1 := 1;
    SIGNAL flip1, flip2 : INTEGER RANGE -1 TO 1;
    SIGNAL alpha1, alpha2, theta, x, y, n, a, b, temp1, temp2, cosF, sinF : INTEGER := 0;
    SIGNAL finished : BOOLEAN := false;
    SIGNAL i : NATURAL RANGE 0 TO 20 := 0;
    SIGNAL state : NATURAL RANGE 0 TO 6 := 0;

BEGIN

    cosB <= to_signed(NGO, cosB'length) WHEN (cosF < NGO) ELSE
        to_signed(ONE, cosB'length) WHEN (cosF > ONE) ELSE
        to_signed(cosF, cosB'length);
    sinB <= to_signed(NGO, sinB'length) WHEN (sinF < NGO) ELSE
        to_signed(ONE, sinB'length) WHEN (sinF > ONE) ELSE
        to_signed(sinF, sinB'length);
    done <= finished;

    n <= to_integer(shift_right(multfix(alpha1, DPR), FRAC)) REM 360;
    a <= n REM 180;
    flip1 <= (-1) WHEN a < (-90) ELSE
        1;
    b <= a + 180 WHEN a < (-90) ELSE
        a;
    flip2 <= (-flip1) WHEN ((-n) / 180) > 0 ELSE
        flip1;

    PROCESS
    BEGIN
        WAIT UNTIL rising_edge(clk);
        CASE state IS
            WHEN 0 =>
                IF start THEN
                    IF angle > 0 THEN -- negative angles are more accurage
                        rev <= (-1);
                        alpha1 <= to_integer(signed((NOT unsigned(angle)) + 1));
                    ELSE
                        alpha1 <= to_integer(angle);
                        rev <= 1;
                    END IF;
                    flip3 <= 1;
                    state <= 1;
                    finished <= false;
                END IF;
            WHEN 1 =>
                state <= 2;
            WHEN 2 =>
                IF alpha1 < (-HALFPI) THEN -- compensate for angles outside of -90° to +90°
                    alpha2 <= multfix(to_integer(shift_left(to_signed(b, 32), FRAC)), RPD);
                    flip3 <= flip2;
                ELSE
                    alpha2 <= alpha1;
                END IF;
                x <= ONE; -- do set up for CORDIC iterations
                theta <= 0;
                y <= 0;
                state <= 3;
                i <= 0;
            WHEN 3 =>
                IF theta < alpha2 THEN
                    sigma <= 1;
                ELSE
                    sigma <= (-1);
                END IF;
                temp1 <= x;
                temp2 <= y;
                state <= 4;
            WHEN 4 =>
                theta <= theta + (sigma * table(i));
                x <= x - multfix(temp2 * sigma, P2i(i));
                y <= y + multfix(P2i(i) * sigma, temp1);
                state <= 5;
            WHEN 5 =>
                IF i < 20 THEN -- loop governor
                    i <= i + 1;
                    state <= 3;
                ELSE
                    state <= 6;
                END IF;
            WHEN 6 =>
                cosF <= flip3 * multfix(Kn, x);
                sinF <= (rev * flip3) * multfix(Kn, y);
                finished <= true;
                state <= 0;
        END CASE;
    END PROCESS;

END Behavioral;