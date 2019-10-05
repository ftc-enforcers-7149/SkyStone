package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DetectionClass {
    //Declarations of hardware
    private DistanceSensor distL, distR, distC;

    //Class vars
    private double fDLast, fDCurrent;
    private double tDLast, tDCurrent;

    //left = left distance sensor return value
    //right = right distance sensor return value
    //mid = center distance sensor return value
    //distance = value to determine if sensors detect obstacle
    private double left, right, mid;
    private double distance;

    // State used for updating telemetry
    //sensorState = array to hold boolean values of whether or not each sensor detects an object {left, mid, right}
    //movingState = object to hold direction of motion intended for the switch statement in loop
    private boolean[] sensorState = {false, false, false};

    //Used for encoders
    static final double     EXTERNAL_GEARING        = 1;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;

    /**
     * Constructor hardware maps distance sensors and sets variables
     * @param hardwareMap
     * @param left
     * @param right
     * @param mid
     */
    public DetectionClass(HardwareMap hardwareMap, String left, String right, String mid) {
        //Hardware map the distance sensors
        distL = hardwareMap.get(DistanceSensor.class, left);
        distR = hardwareMap.get(DistanceSensor.class, right);
        distC = hardwareMap.get(DistanceSensor.class, mid);

        //Setup variables so errors aren't thrown
        distance = 35;
        tDCurrent = 0;
        getInput();
        fDCurrent = this.mid;
    }

    /**
     * detects if object is moving or not
     * @return
     */
    public boolean isObjectMoving(DcMotor fLeft, DcMotor fRight) {
        tDLast = tDCurrent;
        fDLast = fDCurrent;

        double pos = inToCm(fLeft.getCurrentPosition()/COUNTS_PER_INCH + fRight.getCurrentPosition()/COUNTS_PER_INCH) / 2;
        double dist = distC.getDistance(DistanceUnit.CM);

        if (fDLast - fDCurrent > (tDCurrent - tDLast) + 2) {
            tDCurrent = pos;
            fDCurrent = dist;
            return true;
        }

        tDCurrent = pos;
        fDCurrent = dist;
        return false;
    }

    /**
     * Get sensor inputs for sensorState
     */
    public void getInput() {
        //Set variables for distance values
        left = distL.getDistance(DistanceUnit.CM);
        mid = distC.getDistance(DistanceUnit.CM);
        right = distR.getDistance(DistanceUnit.CM);

        //Setup sensorState with boolean values
        sensorState[0] = left <= distance;
        sensorState[1] = mid <= distance;
        sensorState[2] = right <= distance;
    }

    public double getLeft() {
        return left;
    }

    public double getRight() {
        return right;
    }

    public double getMid() {
        return mid;
    }

    public boolean[] getSensorState() {
        return sensorState;
    }

    /**
     * Converts centimeter input to inch output
     * @param cm
     * @return
     */
    public double cmToIn(double cm) {
        return cm / 2.54;
    }

    /**
     * Converts inch input to centimeter output
     * @param in
     * @return
     */
    public double inToCm(double in) {
        return in * 2.54;
    }

    /**
     * Returns one string containing current and last travel and sensor distances
     * @return
     */
    public String getMovingData() {
        return "Distance traveled: " + tDCurrent + ", Last distance traveled: " + tDLast +
                "\nSensor Distance: " + fDCurrent + ", Last sensor distance: " + fDLast;
    }
}
