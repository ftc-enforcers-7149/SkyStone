package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Range {

    private DistanceSensor dLeft, dRight, dCenter;

    public Range(HardwareMap hardwareMap, String left, String right, String center) {
        dLeft = hardwareMap.get(DistanceSensor.class, left);
        dRight = hardwareMap.get(DistanceSensor.class, right);
        dCenter = hardwareMap.get(DistanceSensor.class, center);
    }

    public double getLeft(DistanceUnit unit) {
        return dLeft.getDistance(unit);
    }

    public double getRight(DistanceUnit unit) {
        return dRight.getDistance(unit);
    }

    public double getCenter(DistanceUnit unit) {
        return dCenter.getDistance(unit);
    }
}
