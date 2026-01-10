----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 01/09/2026 12:37:07 PM
-- Design Name: 
-- Module Name: micvolumecontrol - Behavioral
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

entity micvolumecontrol is
    port( 
        -- Audio Clocking Connections 
        axis_aclk     : in std_logic;
        axis_aresetn  : in std_logic;
        -- Audio Slave Interface
        s_axis_tdata  : in std_logic_vector (31 downto 0);
        s_axis_tvalid : in std_logic;
        s_axis_tready : out std_logic;
        s_axis_tlast  : in std_logic; 
        s_axis_tid    : in std_logic_vector (2 downto 0); 
        -- Audio Master Interface
        m_axis_tdata  : out std_logic_vector (31 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tready : in std_logic;
        m_axis_tlast  : out std_logic;
        m_axis_tid    : out std_logic_vector (2 downto 0) 
        );
end micvolumecontrol;

architecture Behavioral of micvolumecontrol is
    constant SHIFT_AMOUNT : integer := 10; 
    -- Filter (holds the last 3 audio samples)
    type sample_buffer is array (0 to 2) of signed(31 downto 0);
    signal taps : sample_buffer := (others => (others => '0'));
begin
        -- Passthrough, so transfer/receive always ready 
        s_axis_tready <= m_axis_tready;
        -- Math Logic 
        process(axis_aclk)
            variable shifted_raw    : std_logic_vector(31 downto 0);
            variable current_clean  : signed(31 downto 0);
            variable sum            : signed(31 downto 0);
            variable final_out      : signed(31 downto 0);
            variable p0, p1, p2, p3 : signed(31 downto 0);
    begin
        if rising_edge(axis_aclk) then
            if axis_aresetn = '0' then
                m_axis_tdata  <= (others => '0');
                m_axis_tvalid <= '0';
                m_axis_tlast  <= '0';
                m_axis_tid    <= (others => '0');
                taps          <= (others => (others => '0'));
            else
                m_axis_tvalid <= s_axis_tvalid;
                m_axis_tlast  <= s_axis_tlast;
                m_axis_tid    <= s_axis_tid;
                
                if s_axis_tvalid = '1' and m_axis_tready = '1' then
                    
                    -- FIX: SHIFT LEFT 8
                    -- Input is 24 bits in the bottom of 32 bits (00xxxxxx).
                    -- We shift left 8 to move the Sign Bit to position 31.
                    -- Zeros are padded at the bottom (killing the tri-state noise).
                    shifted_raw := s_axis_tdata(23 downto 0) & x"00";
                    
                    -- Convert to signed for math
                    current_clean := signed(shifted_raw);
                    
                    -- FILTERING (Average last 4 samples)
                    taps(0) <= current_clean;
                    taps(1) <= taps(0);
                    taps(2) <= taps(1);
                    
                    -- We divide each sample by 4 (Shift Right 2) BEFORE adding.
                    p0 := shift_right(current_clean, 2);
                    p1 := shift_right(taps(0), 2);
                    p2 := shift_right(taps(1), 2);
                    p3 := shift_right(taps(2), 2);
                    
                    -- Now the sum is guaranteed to fit in 32 bits
                    sum := p0 + p1 + p2 + p3;
                    
                    -- VOLUME
                    final_out := shift_right(sum, SHIFT_AMOUNT);
                    
                    -- CHANNEL SELECT
                    if s_axis_tid(0) = '0' then
                        m_axis_tdata <= std_logic_vector(final_out);
                    else
                        m_axis_tdata <= (others => '0');
                    end if;
                end if; 
            end if; 
        end if; 
    end process;
end Behavioral;