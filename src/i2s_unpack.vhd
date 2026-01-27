----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 01/26/2026 06:40:22 PM
-- Design Name: 
-- Module Name: i2s_unpack - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity i2s_unpack is
    port(
        aclk    : in  std_logic;
        aresetn : in  std_logic;
        -- From FIR Compiler (data-only)
        fir_tdata  : in  std_logic_vector(23 downto 0);
        fir_tvalid : in  std_logic;
        fir_tready : out std_logic;
        -- From Metadata FIFO (data-only, 16-bit)
        meta_tdata  : in  std_logic_vector(15 downto 0);
        meta_tvalid : in  std_logic;
        meta_tready : out std_logic;
        -- To I2S TX
        tx_tdata  : out std_logic_vector(31 downto 0);
        tx_tvalid : out std_logic;
        tx_tready : in  std_logic;
        tx_tid    : out std_logic_vector(2 downto 0)
    );
end i2s_unpack;

architecture Behavioral of i2s_unpack is
begin
    -- Output data only when BOTH FIR and FIFO are valid 
    tx_tvalid <= fir_tvalid and meta_tvalid;
    
    -- Only let one advance when the other is present and TX is ready
    fir_tready  <= tx_tready and meta_tvalid;
    meta_tready <= tx_tready and fir_tvalid;
    
    -- Repack data into 32-bit I2S vector
    p_repack : process(fir_tdata, meta_tdata)
        variable y24     : signed(23 downto 0);
        variable w       : std_logic_vector(31 downto 0);
    begin
        y24     := signed(fir_tdata);
        w := (others => '0');
        w(31 downto 28) := meta_tdata(15 downto 12); -- misc
        w(27 downto 4)  := std_logic_vector(y24);    -- sample
        w(3 downto 0)   := meta_tdata(11 downto 8);  -- preamble
        tx_tdata <= w;
        tx_tid   <= meta_tdata(7 downto 5);
    end process;
end Behavioral;
