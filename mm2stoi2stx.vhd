----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 02/23/2026 05:32:22 PM
-- Design Name: 
-- Module Name: mm2stoi2stx - Behavioral
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
use IEEE.NUMERIC_STD.ALL;

entity mm2stoi2stx is
    port(
        axis_aclk     : in  std_logic;
        axis_aresetn  : in  std_logic;

        s_axis_tdata  : in  std_logic_vector(31 downto 0);
        s_axis_tvalid : in  std_logic;
        s_axis_tready : out std_logic;
        s_axis_tlast  : in std_logic;
        s_axis_tkeep  : in  std_logic_vector(3 downto 0);

        m_axis_tdata  : out std_logic_vector(31 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tready : in  std_logic;
        m_axis_tid    : out std_logic_vector(2 downto 0)
    );
end mm2stoi2stx;

architecture Behavioral of mm2stoi2stx is
    signal channel_flag : std_logic := '0';
begin
    -- Pass-through: no buffering, just add TID
    s_axis_tready <= m_axis_tready;
    m_axis_tvalid <= s_axis_tvalid;
    m_axis_tdata  <= s_axis_tdata;
    m_axis_tid    <= "00" & channel_flag;

    process(axis_aclk)
    begin
        if rising_edge(axis_aclk) then
            if axis_aresetn = '0' then
                channel_flag <= '0';
            elsif s_axis_tvalid = '1' and m_axis_tready = '1' then
                channel_flag <= not channel_flag;
            end if;
        end if;
    end process;
end Behavioral;