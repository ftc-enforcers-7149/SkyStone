package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

/**
 * This is a concept file for code to be implemented in CollisionAvoidance/CollisionData
 */
//
@Autonomous(name = "DeltaDistance")
public class RecordCollisionData extends OpMode {

    //DC Motors
    DcMotor fLeft, fRight, bRight, bLeft; //Left and right motors

    //Distance Sensors
    DistanceSensor front;

    //Array used to hold power and distance values for telemetry
    public double[] distanceDelta, encoderDelta;

    //Power set to the motors (in this case front left)
    public double power;

    //Distance read from front facing sensor
    public double distF, lastDistance, lastEncoder, deltaTime;

    //Time that test starts
    public double startTime;

    //Gamepad inputs for changing tests
    public boolean startTest;

    //loops is used to count the amount of loops the program has run through while testing
    //count is used to set the next index in datapoints every some amount of loops
    public int count;

    //Used to determine when test has ended
    public boolean testing = false;

    //used for encoders
    static final double     EXTERNAL_GEARING        = 1;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;

    public void init() {
        fLeft= hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bRight = hardwareMap.dcMotor.get("bRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.REVERSE);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front = hardwareMap.get(DistanceSensor.class, "distanceC");

        //double array that can hold 20 values
        distanceDelta = new double[50];
        encoderDelta = new double[50];

        //Initialize variables
        power = 0.2;
        lastDistance = 0;
        lastEncoder = 0;
        count = 0;
        testing = true;
        deltaTime = 0;
    }

    public void loop() {
        startTest = gamepad1.a;

        telemetry.addData("Power: ", power);

        if (testing) {
            if (startTest) {     //Wait until button A is pressed to continue
                testing = false;
                startTime = System.currentTimeMillis();
                driveStraight("forward", 36, power);
            }
        }
    }

    /**
     * drives inputted distance
     * @param direction direction of driving. "backward" to go backward
     * @param distance distance driving in inches
     */
    public void driveStraight(String direction, double distance, double speed) {
        resetEncoderWithoutEncoder();
        int mDirection = 1;
        if (direction.equals("backward")) {
            mDirection = -1;
        }

        double power=speed*mDirection;
        double cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;

        while(cPosition < distance){
            fLeft.setPower(power);
            fRight.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(power);

            cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;
            distF = front.getDistance(DistanceUnit.CM);

            if (System.currentTimeMillis() - deltaTime >= 100) {
                lastDistance = distF;
                lastEncoder = cPosition;
                count++;

                deltaTime -= 100;
            }

            distanceDelta[count] = distF - lastDistance;       //Set data point for distance at some amount of seconds
            encoderDelta[count] = cPosition - lastEncoder;     //Set data point for encoder at some amount of seconds

            deltaTime = System.currentTimeMillis();

            Log.i("Power Level: ", Double.toString(power));
            Log.i("Distance per 0.5 secs", Arrays.toString(distanceDelta));    //Log array data for safe-keeping
            Log.i("Position per 0.5 secs", Arrays.toString(encoderDelta));
        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);

        Log.i("Power Level: ", Double.toString(power));
        Log.i("Distance per 0.5 secs", Arrays.toString(distanceDelta));    //Log array data for safe-keeping
        Log.i("Position per 0.5 secs", Arrays.toString(encoderDelta));
    }

    /**
     * Resets drive encoders without running using encoders
     */
    public void resetEncoderWithoutEncoder(){
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}