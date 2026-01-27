----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 01/26/2026 06:29:29 PM
-- Design Name: 
-- Module Name: i2s_pack - Behavioral
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

entity i2s_pack is
  port(
    aclk    : in  std_logic;
    aresetn : in  std_logic;
    -- From I2S RX
    rx_tdata  : in  std_logic_vector(31 downto 0);
    rx_tvalid : in  std_logic;
    rx_tready : out std_logic;
    rx_tid    : in  std_logic_vector(2 downto 0);
    -- To FIR Compiler
    fir_tdata  : out std_logic_vector(23 downto 0);
    fir_tvalid : out std_logic;
    fir_tready : in  std_logic;
    -- To Metadata FIFO 
    meta_tdata  : out std_logic_vector(15 downto 0);
    meta_tvalid : out std_logic;
    meta_tready : in  std_logic
  );
end entity;

architecture Behavioral of i2s_pack is
begin
    -- Pack metadata:
    -- [15:12]=misc (TDATA[31:28]), [11:8]=preamble (TDATA[3:0]),
    -- [7:5]=TID, [4:0]=0
    meta_tdata(15 downto 12) <= rx_tdata(31 downto 28);
    meta_tdata(11 downto 8)  <= rx_tdata(3 downto 0);
    meta_tdata(7 downto 5)   <= rx_tid;
    meta_tdata(4 downto 0)   <= (others => '0');
    -- Extract audio sample from [27:4]
    p_sample : process(rx_tdata)
        variable s24 : signed(23 downto 0);
    begin
        s24 := signed(rx_tdata(27 downto 4));
        fir_tdata <= std_logic_vector(s24);
    end process;
    -- Accept data only when both downstreams can accept
    rx_tready  <= fir_tready and meta_tready;
    fir_tvalid <= rx_tvalid and meta_tready;
    meta_tvalid<= rx_tvalid and fir_tready;
end Behavioral;
