library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity audio_align is
    port(
        axis_aclk     : in  std_logic;
        axis_aresetn  : in  std_logic;

        s_axis_tdata  : in  std_logic_vector(31 downto 0);
        s_axis_tvalid : in  std_logic;
        s_axis_tready : out std_logic;
        s_axis_tlast  : in  std_logic;
        s_axis_tid    : in  std_logic_vector(2 downto 0);

        m_axis_tdata  : out std_logic_vector(31 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tready : in  std_logic;
        m_axis_tlast  : out std_logic;
        m_axis_tid    : out std_logic_vector(2 downto 0)
    );
end audio_align;

architecture Behavioral of audio_align is
begin
    s_axis_tready <= m_axis_tready;

    process(axis_aclk)
        variable sample24  : signed(23 downto 0);
        variable aligned32 : signed(31 downto 0);
    begin
        if rising_edge(axis_aclk) then
            if axis_aresetn = '0' then
                m_axis_tdata  <= (others => '0');
                m_axis_tvalid <= '0';
                m_axis_tlast  <= '0';
                m_axis_tid    <= (others => '0');
            else
                m_axis_tvalid <= s_axis_tvalid;
                m_axis_tlast  <= s_axis_tlast;
                m_axis_tid    <= s_axis_tid;

                -- Only update output valid audio 
                if (s_axis_tvalid = '1') and (m_axis_tready = '1') then

                    -- Pick the 24-bit sample
                    sample24 := signed(s_axis_tdata(31 downto 8));

                    -- Sign-extend 24->32 and left-justify into [31:8]
                    aligned32 := resize(sample24, 32) sll 8;

                    if s_axis_tid(0) = '0' then
                        -- Mute other channel (mono)
                        m_axis_tdata <= std_logic_vector(aligned32);
                    else
                        m_axis_tdata <= (others => '0');
                    end if;

                end if;
            end if;
        end if;
    end process;
end Behavioral;
