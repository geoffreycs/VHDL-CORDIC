-- Written by Geoffrey Stentiford for ECE383 Final project
-- April-May 2024
-- 
-- How to use:
-- Just put angle in radians in signed Q10.22 fixed point on angle signal
-- and get sine and cosine value out on sinB and cosB, also sQ10.22
-- Angles of any size representable in Q10.22 are acceptable. You
-- do not need to limit yourself to +/-90°.
--
-- This design is intensive in DSP and LUT usage, but is entirely
-- implemented as concurrent assignments. No clocking or control logic is
-- needed. As far as the FPGA is concerned, this module produces answers
-- instantly, which is faster than the Xilinx parallel CORDIC IP core,
-- which would take 21 clock cycles to achieve the same level of precision
-- and also requires one to work through the AXI4 bus (yuck).
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

ENTITY cordic_fast IS
    PORT (
        angle : IN signed(31 DOWNTO 0);
        sinB : OUT signed(31 DOWNTO 0);
        cosB : OUT signed(31 DOWNTO 0)
    );
END cordic_fast;

ARCHITECTURE Behavioral OF cordic_fast IS

    -- 21 iterations
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
    TYPE int_array IS ARRAY(0 TO 21) OF INTEGER;
    TYPE temp IS ARRAY(1 TO 21) OF INTEGER RANGE -1 TO 1;
    TYPE dyntbl IS ARRAY(0 TO 20) OF INTEGER;

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

    -- Less resource-intensive but only possible for smaller multiplications
    FUNCTION mult32f(a : INTEGER; b : INTEGER) RETURN INTEGER IS
    BEGIN
        RETURN to_integer(shift_right(to_signed(a * b, 32), FRAC));
    END FUNCTION;

    SIGNAL alpha1 : signed(31 DOWNTO 0);
    SIGNAL x, y : int_array;
    SIGNAL theta : dyntbl;
    SIGNAL sigma : temp;
    SIGNAL alpha2, cosF, sinF, x_n, x_a, x_b : INTEGER;
    SIGNAL rev, flip1, flip2, flip3 : INTEGER RANGE -1 TO 1;

BEGIN
    -- outputs
    cosF <= flip3 * multfix(Kn, x(21));
    sinF <= (rev * flip3) * multfix(Kn, y(21));
    cosB <= to_signed(NGO, cosB'length) WHEN (cosF < NGO) ELSE
        to_signed(ONE, cosB'length) WHEN (cosF > ONE) ELSE
        to_signed(cosF, cosB'length);
    sinB <= to_signed(NGO, sinB'length) WHEN (sinF < NGO) ELSE
        to_signed(ONE, sinB'length) WHEN (sinF > ONE) ELSE
        to_signed(sinF, sinB'length);

    -- negative angles more accurate and sin/cos are symmetrical
    rev <= (-1) WHEN angle > 0 ELSE
        1;
    alpha1 <= angle WHEN rev = 1 ELSE
        signed((NOT unsigned(angle)) + 1);

    -- compensate for angles outside of -90° to +90°
    x_n <= to_integer(shift_right(multfix(alpha1, DPR), FRAC)) REM 360;
    x_a <= x_n REM 180;
    flip1 <= (-1) WHEN x_a < (-90) ELSE
        1;
    x_b <= x_a + 180 WHEN x_a < (-90) ELSE
        x_a;
    flip2 <= (-flip1) WHEN ((-x_n) / 180) > 0 ELSE
        flip1;
    flip3 <= flip2 WHEN alpha1 < (-HALFPI) ELSE
        1;
    alpha2 <= multfix(to_integer(shift_left(to_signed(x_b, 32), FRAC)), RPD) WHEN alpha1 < (-HALFPI) ELSE
        to_integer(alpha1);

    -- initial inputs
    theta(0) <= 0;
    x(0) <= ONE;
    y(0) <= 0;

    -- BEGIN TEMPLATE-GENERATED CODE

    -- i = 0
    sigma(1) <= 1 WHEN (theta(0) < alpha2) ELSE
    (-1);
    theta(1) <= theta(0) + (sigma(1) * table(0));
    x(1) <= x(0) - multfix(sigma(1) * y(0), P2i(0));
    y(1) <= y(0) + multfix(sigma(1) * P2i(0), x(0));

    -- i = 1
    sigma(2) <= 1 WHEN (theta(1) < alpha2) ELSE
    (-1);
    theta(2) <= theta(1) + (sigma(2) * table(1));
    x(2) <= x(1) - multfix(sigma(2) * y(1), P2i(1));
    y(2) <= y(1) + multfix(sigma(2) * P2i(1), x(1));

    -- i = 2
    sigma(3) <= 1 WHEN (theta(2) < alpha2) ELSE
    (-1);
    theta(3) <= theta(2) + (sigma(3) * table(2));
    x(3) <= x(2) - multfix(sigma(3) * y(2), P2i(2));
    y(3) <= y(2) + multfix(sigma(3) * P2i(2), x(2));

    -- i = 3
    sigma(4) <= 1 WHEN (theta(3) < alpha2) ELSE
    (-1);
    theta(4) <= theta(3) + (sigma(4) * table(3));
    x(4) <= x(3) - multfix(sigma(4) * y(3), P2i(3));
    y(4) <= y(3) + multfix(sigma(4) * P2i(3), x(3));

    -- i = 4
    sigma(5) <= 1 WHEN (theta(4) < alpha2) ELSE
    (-1);
    theta(5) <= theta(4) + (sigma(5) * table(4));
    x(5) <= x(4) - multfix(sigma(5) * y(4), P2i(4));
    y(5) <= y(4) + multfix(sigma(5) * P2i(4), x(4));

    -- i = 5
    sigma(6) <= 1 WHEN (theta(5) < alpha2) ELSE
    (-1);
    theta(6) <= theta(5) + (sigma(6) * table(5));
    x(6) <= x(5) - multfix(sigma(6) * y(5), P2i(5));
    y(6) <= y(5) + multfix(sigma(6) * P2i(5), x(5));

    -- i = 6
    sigma(7) <= 1 WHEN (theta(6) < alpha2) ELSE
    (-1);
    theta(7) <= theta(6) + (sigma(7) * table(6));
    x(7) <= x(6) - multfix(sigma(7) * y(6), P2i(6));
    y(7) <= y(6) + multfix(sigma(7) * P2i(6), x(6));

    -- i = 7
    sigma(8) <= 1 WHEN (theta(7) < alpha2) ELSE
    (-1);
    theta(8) <= theta(7) + (sigma(8) * table(7));
    x(8) <= x(7) - multfix(sigma(8) * y(7), P2i(7));
    y(8) <= y(7) + multfix(sigma(8) * P2i(7), x(7));

    -- i = 8
    sigma(9) <= 1 WHEN (theta(8) < alpha2) ELSE
    (-1);
    theta(9) <= theta(8) + (sigma(9) * table(8));
    x(9) <= x(8) - multfix(sigma(9) * y(8), P2i(8));
    y(9) <= y(8) + multfix(sigma(9) * P2i(8), x(8));

    -- i = 9
    sigma(10) <= 1 WHEN (theta(9) < alpha2) ELSE
    (-1);
    theta(10) <= theta(9) + (sigma(10) * table(9));
    x(10) <= x(9) - multfix(sigma(10) * y(9), P2i(9));
    y(10) <= y(9) + multfix(sigma(10) * P2i(9), x(9));

    -- i = 10
    sigma(11) <= 1 WHEN (theta(10) < alpha2) ELSE
    (-1);
    theta(11) <= theta(10) + (sigma(11) * table(10));
    x(11) <= x(10) - multfix(sigma(11) * y(10), P2i(10));
    y(11) <= y(10) + multfix(sigma(11) * P2i(10), x(10));

    -- i = 11
    sigma(12) <= 1 WHEN (theta(11) < alpha2) ELSE
    (-1);
    theta(12) <= theta(11) + (sigma(12) * table(11));
    x(12) <= x(11) - multfix(sigma(12) * y(11), P2i(11));
    y(12) <= y(11) + multfix(sigma(12) * P2i(11), x(11));

    -- i = 12
    sigma(13) <= 1 WHEN (theta(12) < alpha2) ELSE
    (-1);
    theta(13) <= theta(12) + (sigma(13) * table(12));
    x(13) <= x(12) - multfix(sigma(13) * y(12), P2i(12));
    y(13) <= y(12) + multfix(sigma(13) * P2i(12), x(12));

    -- i = 13
    sigma(14) <= 1 WHEN (theta(13) < alpha2) ELSE
    (-1);
    theta(14) <= theta(13) + (sigma(14) * table(13));
    x(14) <= x(13) - multfix(sigma(14) * y(13), P2i(13));
    y(14) <= y(13) + multfix(sigma(14) * P2i(13), x(13));

    -- i = 14
    sigma(15) <= 1 WHEN (theta(14) < alpha2) ELSE
    (-1);
    theta(15) <= theta(14) + (sigma(15) * table(14));
    x(15) <= x(14) - mult32f(sigma(15) * y(14), P2i(14));
    y(15) <= y(14) + mult32f(sigma(15) * P2i(14), x(14));

    -- i = 15
    sigma(16) <= 1 WHEN (theta(15) < alpha2) ELSE
    (-1);
    theta(16) <= theta(15) + (sigma(16) * table(15));
    x(16) <= x(15) - mult32f(sigma(16) * y(15), P2i(15));
    y(16) <= y(15) + mult32f(sigma(16) * P2i(15), x(15));

    -- i = 16
    sigma(17) <= 1 WHEN (theta(16) < alpha2) ELSE
    (-1);
    theta(17) <= theta(16) + (sigma(17) * table(16));
    x(17) <= x(16) - mult32f(sigma(17) * y(16), P2i(16));
    y(17) <= y(16) + mult32f(sigma(17) * P2i(16), x(16));

    -- i = 17
    sigma(18) <= 1 WHEN (theta(17) < alpha2) ELSE
    (-1);
    theta(18) <= theta(17) + (sigma(18) * table(17));
    x(18) <= x(17) - mult32f(sigma(18) * y(17), P2i(17));
    y(18) <= y(17) + mult32f(sigma(18) * P2i(17), x(17));

    -- i = 18
    sigma(19) <= 1 WHEN (theta(18) < alpha2) ELSE
    (-1);
    theta(19) <= theta(18) + (sigma(19) * table(18));
    x(19) <= x(18) - mult32f(sigma(19) * y(18), P2i(18));
    y(19) <= y(18) + mult32f(sigma(19) * P2i(18), x(18));

    -- i = 19
    sigma(20) <= 1 WHEN (theta(19) < alpha2) ELSE
    (-1);
    theta(20) <= theta(19) + (sigma(20) * table(19));
    x(20) <= x(19) - mult32f(sigma(20) * y(19), P2i(19));
    y(20) <= y(19) + mult32f(sigma(20) * P2i(19), x(19));

    -- i = 20
    sigma(21) <= 1 WHEN (theta(20) < alpha2) ELSE
    (-1);
    x(21) <= x(20) - mult32f(sigma(21) * y(20), P2i(20));
    y(21) <= y(20) + mult32f(sigma(21) * P2i(20), x(20));

END Behavioral;