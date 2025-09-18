package org.firstinspires.ftc.teamcode.metalBenders.archive.core;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "AS5600 Magnetic Encoder", xmlTag = "AS5600")

public class AS5600 extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    
    public double getAngleTicks()
    {
        short dataRaw = getAngle();
        return dataRaw;
    }
    
    public double getAngleDeg()
    {
        double dataRaw = getAngle();
        double dataDeg = dataRaw * 360 / 4096;
        return dataDeg;
    }    

    public double getAngleRad()
    {
        double dataRaw = getAngle();
        double dataRad = dataRaw * Math.PI * 2.0 / 4096;
        return dataRad;
    }    

    public double getRawAngleTicks()
    {
        short dataRaw = getRawAngle();
        return dataRaw;
    }
    
    public double getRawAngleDeg()
    {
        double dataRaw = getRawAngle() * (360 / 4096);
        double dataDeg = dataRaw * 360 / 4096;
        return dataDeg;
    }

    public double getRawAngleRad()
    {
        double dataRaw = getRawAngle() * (Math.PI * 2.0 / 4096);
        double dataRad = dataRaw * Math.PI * 2.0 / 4096;
        return dataRad;
    }
    
    @Override
    public Manufacturer getManufacturer()
    {

        return Manufacturer.Adafruit;
    }
    
    @Override
    public String getDeviceName()
    {

        return "Adafruit AS5600 Magnetic Encoder";
    }


    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }    
    
    public enum Register
    {
        
        STATUS(0x0B),
        RAW_ANGLE_UPPER(0x0C),
        RAW_ANGLE(0x0D),
        ANGLE_UPPER(0x0E),
        ANGLE(0x0F),
        FIRST(STATUS.bVal),
        LAST(ANGLE.bVal);

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
    
    public AS5600(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x36));

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Raw Register Reads
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public short getAngle()
    {
        return readShort(Register.ANGLE_UPPER);
    }
    
    public short getRawAngle()
    {
        return readShort(Register.RAW_ANGLE_UPPER);
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

}






















