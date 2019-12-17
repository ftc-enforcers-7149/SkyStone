package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@TeleOp(name = "Encoder Testing")
public class EncoderTest extends OpMode {
    //Drive train
    Headless driveSystem;

    DcMotor fRight,fLeft,bRight,bLeft, lift;

    boolean startAccel;


    //used for encoders
    private static final double     EXTERNAL_GEARING        = 1;
    private static final double     COUNTS_PER_MOTOR_REV    = 753.2 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;

    public void init(){

        //Inits to combat lag
        /*colorSensor = hardwareMap.colorSensor.get("color");
        distL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distC = hardwareMap.get(DistanceSensor.class, "distanceC");*/

        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        lift = hardwareMap.dcMotor.get("lift");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize drive train
        Gyroscope gyroscope = new Gyroscope(telemetry, hardwareMap);
        driveSystem = new Headless(gyroscope, fLeft, fRight, bLeft, bRight);

        //Lift brake
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(){

        startAccel = gamepad1.x;

        //Drive
        driveSystem.drive(gamepad1);


        if (startAccel) {
            driveSystem.setAccel();
        }

        telemetry.addData("fLeft",fLeft.getCurrentPosition()/COUNTS_PER_INCH);
        telemetry.addData("fRight",fRight.getCurrentPosition()/COUNTS_PER_INCH);
        telemetry.addData("bLeft",bLeft.getCurrentPosition()/COUNTS_PER_INCH);
        telemetry.addData("bRight",bRight.getCurrentPosition()/COUNTS_PER_INCH);



    }
    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
