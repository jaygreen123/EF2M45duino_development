<?xml version="1.0" encoding="UTF-8"?>
<Project>
    <Project_Created_Time>2018-05-04 09:56:58</Project_Created_Time>
    <TD_Version>4.6.14756</TD_Version>
    <UCode>10110111</UCode>
    <Name>Quick_Start</Name>
    <HardWare>
        <Family>EF2</Family>
        <Device>EF2M45LG48B</Device>
    </HardWare>
    <Source_Files>
        <Verilog>
            <File>al_ip/AL_MCU.v</File>
            <File>src/quick_start.v</File>
            <File>al_ip/sys_pll.v</File>
            <File>al_ip/bram256kbit.v</File>
        </Verilog>
        <ADC_FILE>constrs/board.adc</ADC_FILE>
        <SDC_FILE/>
        <CWC_FILE>simulation/ChipWatcher.cwc</CWC_FILE>
    </Source_Files>
    <TOP_MODULE>
        <LABEL/>
        <MODULE>quick_start</MODULE>
        <CREATEINDEX>auto</CREATEINDEX>
    </TOP_MODULE>
    <Property>
        <BitgenProperty::GeneralOption>
            <data_ram/>
            <instruct_ram>../ArduinoProject/Blink/Blink.ino.generic_stm32f103c.bin</instruct_ram>
            <s>off</s>
        </BitgenProperty::GeneralOption>
        <GlobalProperty/>
    </Property>
    <Project_Settings>
        <Step_Last_Change>2020-04-08 15:46:58</Step_Last_Change>
        <Current_Step>60</Current_Step>
        <Step_Status>true</Step_Status>
    </Project_Settings>
</Project>
