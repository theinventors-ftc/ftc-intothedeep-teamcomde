package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.util;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "URM09", xmlTag = "Urm09")
public class Urm09 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x11);

    public Urm09(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.DFRobot;
    }

    @Override
    protected synchronized boolean doInitialize() {
        setModeRange(DistanceMode.MODE_300CM.bVal, MeasurementMode.AUTOMATIC.bVal);
        return true;
    }

    @Override
    public String getDeviceName() {
        return "DFRobot URM09";
    }

    public enum Register {
        FIRST(0),
        ADDR(0x00),
        PRODUCT_ID(0x01),
        VERSION(0x02),
        DISTANCE_HIGH_BIT(0x03),    // CM
        DISTANCE_LOW_BIT(0x04),     // CM
        TEMP_HIGH_BIT(0x05),        // 10x actual temperature
        TEMP_LOW_BIT(0x06),         // 10x actual temperature
        CONFIGURE(0x07),            // passive_auto measurement, distance range set
        COMMAND(0x08),              // passive measurement control
        LAST(COMMAND.bVal);

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    public enum DistanceMode {
        MODE_100CM(0x00),
        MODE_300CM(0x10),
        MODE_500CM(0x20);

        public int bVal;

        DistanceMode(int bVal) {
            this.bVal = bVal;
        }
    }

    public enum MeasurementMode {
        AUTOMATIC(0x80),
        PASSIVE(0x00);

        public int bVal;

        MeasurementMode(int bVal) {
            this.bVal = bVal;
        }
    }

    public void setModeRange(int range, int mode) {
        byte config = (byte) (range | mode);
        deviceClient.write8(Register.CONFIGURE.bVal, config);
    }

    public void measurement() {
        deviceClient.write8(Register.COMMAND.bVal, 0x01);
    }

    public int getDistanceCM() {
        byte[] distanceData = deviceClient.read(Register.DISTANCE_HIGH_BIT.bVal, 2);
        int distanceHigh = TypeConversion.unsignedByteToInt(distanceData[0]);
        int distanceLow = TypeConversion.unsignedByteToInt(distanceData[1]);
        return (distanceHigh << 8) | distanceLow;
    }

    public float getTemperature() {
        byte[] tempData = deviceClient.read(Register.TEMP_HIGH_BIT.bVal, 2);
        int tempHigh = TypeConversion.unsignedByteToInt(tempData[0]);
        int tempLow = TypeConversion.unsignedByteToInt(tempData[1]);
        int temp = (tempHigh << 8) | tempLow;
        return temp / 10.0f;
    }

    public void modifyI2CAddress(int newAddress) {
        deviceClient.write8(Register.ADDR.bVal, newAddress);
    }

    public int getI2CAddress() {
        return deviceClient.read8(Register.ADDR.bVal);
    }

    protected void setOptimalReadWindow() {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    public byte[] getAllRegister() {
        return deviceClient.read(Register.FIRST.bVal, Register.LAST.bVal - Register.FIRST.bVal + 1);
    }
}

