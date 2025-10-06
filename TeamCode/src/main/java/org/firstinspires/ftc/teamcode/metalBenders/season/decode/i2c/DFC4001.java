package org.firstinspires.ftc.teamcode.metalBenders.season.decode.i2c;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "DFRobot C4001 MMWave Sensor", xmlTag = "DFC4001")

public class DFC4001 extends I2cDeviceSynchDevice<I2cDeviceSynch> {


    @Override
    public Manufacturer getManufacturer()
    {

        return Manufacturer.DFRobot;
    }
    
    @Override
    public String getDeviceName()
    {

        return "DFRobot C4001 mmWave Sensor";
    }


    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    public enum senModes
    {
        eStartSen(0x55),
        eStopSen(0x33),
        eResetSen(0xCC),
        eRecoverSen(0xAA),
        eSaveParams(0x5C),
        eChangeMode(0x3B);
        public int mVal;
        senModes(int mVal)
        {
            this.mVal = mVal;
        }
    }
    public enum Register
    {
        REG_STATUS(0x00),
        REG_CTRL0(0x01),
        REG_plc02(0x02),
        REG_plc03(0x03),
        REG_plc04(0x04),
        REG_plc05(0x05),
        REG_plc06(0x06),
        REG_plc07(0x07),
        REG_plc08(0x08),
        REG_plc09(0x09),
        // REG_CTRL1(0x02),
        // REG_SOFT_VERSION(0x03),
        // REG_RESULT_STATUS(0x10),
        // REG_TRIG_SENSITIVITY(0x20),
        // REG_KEEP_SENSITIVITY(0x21),
        // REG_TRIG_DELAY(0x22),
        // REG_KEEP_TIMEOUT_L(0x23),
        // REG_KEEP_TIMEOUT_H(0x24),
        // REG_E_MIN_RANGE_L(0x25),
        // REG_E_MIN_RANGE_H(0x26),
        // REG_E_MAX_RANGE_L(0x27),
        // REG_E_MAX_RANGE_H(0x28),
        // REG_E_TRIG_RANGE_L(0x29),
        // REG_E_TRIG_RANGE_H(0x2A),
        REG_RESULT_OBJ_MUN(0x10),
        REG_RESULT_RANGE_L(0x11),
        REG_RESULT_RANGE_H(0x12),
        REG_RESULT_SPEED_L(0x13),
        REG_RESULT_SPEED_H(0x14),
        REG_RESULT_ENERGY_L(0x15),
        REG_RESULT_ENERGY_H(0x16),
        // REG_CFAR_THR_L(0x20),
        // REG_CFAR_THR_H(0x21),
        // REG_T_MIN_RANGE_L(0x22),
        // REG_T_MIN_RANGE_H(0x23),
        // REG_T_MAX_RANGE_L(0x24),
        // REG_T_MAX_RANGE_H(0x25),
        // REG_MICRO_MOTION(0x26),
        FIRST(REG_STATUS.bVal),
        LAST(REG_RESULT_ENERGY_H.bVal);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
        
    }

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }
    
    public DFC4001(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x2A));

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }    

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Raw Register Reads
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public short getStatus()
    {
        return readShort(Register.REG_STATUS);
    }

    public int getRange()
    {
        short upperByte = readShort(DFC4001.Register.REG_RESULT_RANGE_H);
        short lowerByte = readShort(DFC4001.Register.REG_RESULT_RANGE_L);
        int combinedValue = ((upperByte & 0xFF) << 8) | (lowerByte & 0xFF);
        return combinedValue;
    }

    public void startSensor()
    {
        writeShort(Register.REG_CTRL0, senModes.eStartSen);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    protected void writeShort(final Register reg, final senModes value)
    {
        deviceClient.write(reg.bVal, TypeConversion.intToByteArray(value.mVal));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }


}
