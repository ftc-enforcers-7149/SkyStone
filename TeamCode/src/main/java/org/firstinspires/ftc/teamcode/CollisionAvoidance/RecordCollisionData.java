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
//@Autonomous(name = "DeltaDistance")
public class RecordCollisionData extends OpMode {

    //DC Motors
    DcMotor fLeft, fRight, bRight, bLeft; //Left and right motors

    //Distance Sensors
    DistanceSensor front;

    //Power set to the motors (in this case front left)
    public double power;

    //Distance read from front facing sensor
    public double distF, encoderDistance;

    //Time that test starts
    public double startTime;

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

        resetEncoderWithoutEncoder();

        front = hardwareMap.get(DistanceSensor.class, "distanceC");

        //Initialize variables
        power = 0.50;
    }

    public void start() {
        startTime = System.currentTimeMillis();
    }

    public void loop() {
        distF = front.getDistance(DistanceUnit.CM);
        encoderDistance = fRight.getCurrentPosition() / COUNTS_PER_INCH;

        telemetry.addData("Sensor Distance: ", distF);
        telemetry.addData("Travel Distance: ", encoderDistance);

        if (distF > 10){
            fLeft.setPower(power);
            fRight.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(power);

            Log.i("TravelDist,FDist,Time: ", Double.toString(encoderDistance).concat(" \\t ").concat(Double.toString(distF).concat(" \\t ").concat(Double.toString(System.currentTimeMillis() - startTime))));
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            Log.i("Done: ", Double.toString(encoderDistance).concat(" ").concat(Double.toString(System.currentTimeMillis() - startTime)));
        }
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