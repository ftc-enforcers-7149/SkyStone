package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(xmlTag = "prusa", name = "Prusa Filament Sensor")

public class PrusaI2C extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private void writeShort(final Register reg, short val) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(val));
    }

    private short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Prusa Filament Sensor";
    }

    public enum Register {
        FIRST(0),
        PRODUCT_ID1(0x00),
        PRODUCT_ID2(0x01),
        MOTION_STATUS(0x02),
        DELTA_X_LO(0x03),
        DELTA_Y_LO(0x04),
        OPERATION_MODE(0x05),
        CONFIGURATION(0x06),
        WIRTE_PROTECT(0x09),
        SLEEP1(0x0A),
        SLEEP2(0x0B),
        RES_X(0x0D),
        RES_Y(0x0E),
        DELTA_XY_HI(0x12),
        SHUTTER(0x14),
        FRAME_AVG(0x17),
        ORIENTATION(0x19),
        LAST(ORIENTATION.bVal);

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    public enum Reset {
        PRODUCT_ID1(0x31),
        PRODUCT_ID2(0x91),
        OPERATION_MODE(0xA0),
        CONFIGURATION(0x17),
        WIRTE_PROTECT(0x00),
        SLEEP1(0x77),
        SLEEP2(0x10),
        RES_X(0x14),
        RES_Y(0x14),
        ORIENTATION(0x04);

        public int bVal;

        Reset(int bVal) {
            this.bVal = bVal;
        }
    }

    private void setOptimalReadWindow() {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT
        );

        this.deviceClient.setReadWindow(readWindow);
    }

    public PrusaI2C(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(new I2cAddr(0x75));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
}
