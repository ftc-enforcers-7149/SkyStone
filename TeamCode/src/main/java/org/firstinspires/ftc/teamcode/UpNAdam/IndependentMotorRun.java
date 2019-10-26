package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;


@TeleOp(name = "Motor Test")
public class IndependentMotorRun extends OpMode {
    //used for encoders
    private static final double     EXTERNAL_GEARING        = 1;
    private static final double     COUNTS_PER_MOTOR_REV    = 753.2 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;
    //Drive train
    Headless driveSystem;

    Servo lArm, rArm, lGrab, rGrab, lFound, rFound;
    DcMotor fRight,fLeft,bRight,bLeft,lift;

    boolean armUp, armDown;
    boolean isBreak=false;
    float leftG,rightG;
    float liftUp,liftDown;
    boolean foundationDown;
    float lDrive,rDrive,lStrafe,rStrafe;
    public void init(){
        //Servos
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");
        lFound = hardwareMap.servo.get("lFound");
        rFound = hardwareMap.servo.get("rFound");
        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        lift = hardwareMap.dcMotor.get("lift");

        //Initialize drive train
        driveSystem = new Headless(hardwareMap, telemetry, "fLeft", "fRight", "bLeft", "bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);
        lFound.setDirection(Servo.Direction.REVERSE);
        rFound.setDirection(Servo.Direction.FORWARD);


        lArm.setPosition(0.1);
        rArm.setPosition(0.05);
        lGrab.setPosition(0.2);
        rGrab.setPosition(0.25);
        lFound.setPosition(0);
        rFound.setPosition(0);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }//
    public void loop(){
        armUp = gamepad2.y;
        armDown = gamepad2.a;
        rightG = gamepad2.right_trigger;
        leftG = gamepad2.left_trigger;
        liftDown = gamepad1.left_trigger;
        liftUp = gamepad1.right_trigger;
        foundationDown = gamepad1.a;
        lDrive = gamepad1.left_stick_y;
        rDrive = gamepad1.right_stick_y;
        lStrafe = gamepad1.left_trigger;
        rStrafe = gamepad1.right_trigger;
        boolean b=gamepad1.b;
        boolean y=gamepad1.y;
        boolean x=gamepad1.x;
        boolean dUp=gamepad1.dpad_up;

        if(foundationDown){
            bLeft.setPower(0.5);
        }
        else{
            bLeft.setPower(0);
        }

        if(b){
            bRight.setPower(0.5);
        }
        else{
            bRight.setPower(0);
        }

        if(y){
            fLeft.setPower(0.5);
        }
        else{
            fLeft.setPower(0);
        }
        if(x){
            fRight.setPower(0.5);
        }
        else{
            fRight.setPower(0);
        }

        if(dUp){
            fRight.setPower(0.5);
            bRight.setPower(0.5);
            fLeft.setPower(0.5);
            bLeft.setPower(0.5);
        }
        else{
            fRight.setPower(0);
            bRight.setPower(0);
            fLeft.setPower(0);
            bLeft.setPower(0);
        }

        telemetry.addData("bLeft ",bLeft.getCurrentPosition()/COUNTS_PER_INCH);
        telemetry.addData("bRight",bRight.getCurrentPosition()/COUNTS_PER_INCH);
        telemetry.addData("fLeft ",fLeft.getCurrentPosition()/COUNTS_PER_INCH);
        telemetry.addData("fRight",fRight.getCurrentPosition()/COUNTS_PER_INCH);

    }
    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
