library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity sinewavegenerator is
    Port ( 
        axis_aclk     : in STD_LOGIC;
        axis_aresetn  : in STD_LOGIC;
        -- AXI Stream Master Interface (Connect to I2S Transmitter)
        m_axis_tdata  : out STD_LOGIC_VECTOR (31 downto 0);
        m_axis_tvalid : out STD_LOGIC;
        m_axis_tready : in STD_LOGIC;
        m_axis_tlast  : out STD_LOGIC;
        -- ADDED: Transaction ID to identify Left (0) vs Right (1) Channel
        m_axis_tid    : out STD_LOGIC_VECTOR (2 downto 0) 
    );
end sinewavegenerator;

architecture Behavioral of sinewavegenerator is

    -- 48-point Sine Wave Look-Up Table (LUT)
    -- At 48kHz sample rate, a 48-point cycle = 1 kHz Tone.
    type sine_lut_type is array (0 to 47) of integer;
    constant SINE_LUT : sine_lut_type := (
         0,   784591,  1546025,  2262192,  2912769,  3480287,  3949985,  4310872,
   4554816,  4672689,  4658255,  4515535,  4249969,  3869279,  3383344,  2804561,
   2148158,  1432497,   678002,  -104528,  -894989, -1671293, -2411512, -3094470,
  -3700720, -4213000, -4617486, -4900000, -5048896, -5063000, -4942964, -4690000,
  -4307000, -3805000, -3200000, -2509000, -1750000,  -940000,  -100000,   740000,
   1550000,  2300000,  2960000,  3510000,  3930000,  4200000,  4300000,  4200000
    );

    signal lut_index : integer range 0 to 47 := 0;
    signal channel_flag : std_logic := '0'; -- '0' = Left, '1' = Right

begin

    -- Always Valid (we always have data to send)
    m_axis_tvalid <= '1';

    -- Logic for indexing the Sine Wave
    process(axis_aclk)
    begin
        if rising_edge(axis_aclk) then
            if axis_aresetn = '0' then
                lut_index <= 0;
                channel_flag <= '0';
            else
                -- Only advance when the Downstream IP (I2S Transmitter) accepts data
                if m_axis_tready = '1' then

                    -- Flip channel every sample (Left -> Right -> Left...)
                    channel_flag <= not channel_flag;

                    -- Only advance the sine wave index after the RIGHT channel (End of Frame)
                    if channel_flag = '1' then 
                        if lut_index = 47 then
                            lut_index <= 0;
                        else
                            lut_index <= lut_index + 1;
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- Output Assignment
    -- Send the LUT value * 256 (Left Shift 8) to align data to the audible MSB range.
    m_axis_tdata <= std_logic_vector(to_signed(SINE_LUT(lut_index) * 256, 32));  

    -- TLAST is High on the Right Channel (End of Frame)
    m_axis_tlast <= '1' when channel_flag = '1' else '0';

    -- TID indicates the channel index. "000" = Left, "001" = Right.
    -- This matches the requirement of the Digilent I2S Transmitter.
    m_axis_tid   <= "00" & channel_flag;

end Behavioral;