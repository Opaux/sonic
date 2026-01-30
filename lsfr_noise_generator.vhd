library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity lfsr_noise_generator is
    generic (
        -- Output scaling: right shift reduces amplitude (more headroom).
        -- Example: 8 gives a comfortable level for 32-bit audio paths.
        AMP_SHIFT : integer := 4;

        -- Seed for repeatable noise (change seed to get different sequence)
        SEED      : std_logic_vector(31 downto 0) := x"ACE1ACE1"
    );
    port (
        axis_aclk     : in  std_logic;
        axis_aresetn  : in  std_logic;

        -- AXI Stream Master Interface (connect to I2S TX or DMA MM2S sink)
        m_axis_tdata  : out std_logic_vector(31 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tready : in  std_logic;
        m_axis_tlast  : out std_logic;
        m_axis_tid    : out std_logic_vector(2 downto 0)
    );
end lfsr_noise_generator;

architecture rtl of lfsr_noise_generator is

    signal lfsr         : std_logic_vector(31 downto 0) := SEED;
    signal channel_flag : std_logic := '0'; -- '0' = Left, '1' = Right
    signal sample_reg   : signed(31 downto 0) := (others => '0');

    -- XOR taps for a 32-bit maximal-length LFSR:
    -- polynomial: x^32 + x^22 + x^2 + x + 1
    -- (common, works well for PRBS noise)
    function lfsr_next(x : std_logic_vector(31 downto 0)) return std_logic_vector is
        variable fb : std_logic;
        variable y  : std_logic_vector(31 downto 0);
    begin
        fb := x(31) xor x(21) xor x(1) xor x(0);
        y  := x(30 downto 0) & fb;
        return y;
    end function;

begin

    -- Always have data available
    m_axis_tvalid <= '1';

    process(axis_aclk)
        variable next_l : std_logic_vector(31 downto 0);
        variable s      : signed(31 downto 0);
    begin
        if rising_edge(axis_aclk) then
            if axis_aresetn = '0' then
                lfsr         <= SEED;
                channel_flag <= '0';
                sample_reg   <= (others => '0');
            else
                if m_axis_tready = '1' then
                    -- advance channel every accepted beat
                    channel_flag <= not channel_flag;

                    -- advance LFSR every accepted beat (new sample each L/R beat)
                    next_l := lfsr_next(lfsr);
                    lfsr   <= next_l;

                    -- interpret bits as signed noise
                    s := signed(next_l);

                    -- scale down (right shift) to prevent clipping
                    if AMP_SHIFT > 0 then
                        s := shift_right(s, AMP_SHIFT);
                    end if;

                    sample_reg <= s;
                end if;
            end if;
        end if;
    end process;

    m_axis_tdata <= std_logic_vector(sample_reg);

    -- End-of-frame on Right channel (same convention as your sine module)
    m_axis_tlast <= '1' when channel_flag = '1' else '0';

    -- "000" = Left, "001" = Right (matches your Digilent I2S TX expectation)
    m_axis_tid   <= "00" & channel_flag;

end rtl;
