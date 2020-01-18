package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@TeleOp(name = "TeleOp v2")
public class TeleOpV2 extends OpMode {
    //Drive train
    Headless driveSystem;
    //Hardware
    Servo fLFound, fRFound, bLFound, bRFound;
    Servo lArm, rArm, lGrab, rGrab;
    DcMotor fRight,fLeft,bRight,bLeft, liftMotor;

    float armUp, last_armUp=0;
    boolean isBreak=false;
    float liftUp, last_liftUp=0, liftDown, last_liftDown=0;
    boolean lFoundationDown, last_lFoundationDown=false, rFoundationDown, last_rFoundationDown=false;
    float grab, last_grab=0;
    boolean armsDown=false;

    public void init(){
        //Servos
        fLFound = hardwareMap.servo.get("fLFound");
        fRFound = hardwareMap.servo.get("fRFound");
        bLFound = hardwareMap.servo.get("bLFound");
        bRFound = hardwareMap.servo.get("bRFound");
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");

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
        liftMotor = hardwareMap.dcMotor.get("lift");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize drive train
        Gyroscope gyroscope = new Gyroscope(telemetry, hardwareMap);
        driveSystem = new Headless(gyroscope, fLeft, fRight, bLeft, bRight);

        //Servo directions
        fLFound.setDirection(Servo.Direction.REVERSE);
        fRFound.setDirection(Servo.Direction.FORWARD);
        bLFound.setDirection(Servo.Direction.FORWARD);
        bRFound.setDirection(Servo.Direction.REVERSE);

        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);

        //Lift brake
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        //Inputs
        armUp = gamepad2.left_trigger;
        grab = gamepad2.right_trigger;
        liftUp=gamepad1.right_trigger;
        liftDown=gamepad1.left_trigger;
        lFoundationDown = gamepad1.left_bumper || gamepad2.x;
        rFoundationDown = gamepad1.right_bumper || gamepad2.b;

        //Drive
        driveSystem.drive(gamepad1);

        //FoundationV1 grabbers//
        //Left foundation
        if (lFoundationDown != last_lFoundationDown) {
            if (lFoundationDown) {
                fLFound.setPosition(0.34);//0.79
                bLFound.setPosition(0.34 );//0.79
            } else {
                fLFound.setPosition(1);//1
                bLFound.setPosition(1);//1
            }
        }

        //Right foundation
        if (rFoundationDown != last_rFoundationDown) {
            if (rFoundationDown) {
                fRFound.setPosition(0.75);//40
                bRFound.setPosition(0.75);//40
            } else {
                fRFound.setPosition(1);
                bRFound.setPosition(1);
            }
        }

        //Arms
        if (armsDown) {
            if (armUp != last_armUp) {
                if (armUp > 0.1) {
                    lArm.setPosition(0.95);
                    rArm.setPosition(0.81);
                } else {
                    lArm.setPosition(0.36);
                    rArm.setPosition(0.3);
                }
            }
        }
        else if (armUp > 0.1) {
            armsDown=true;
        }

        //Grabber
        if (grab != last_grab) {
            //Double servo claw: r 0.2 l 0.28 closed, r 0.13 l 0.23 open

            if (grab > 0.1) {
                rGrab.setPosition(0.2);//.2
                lGrab.setPosition(0.28);//.28
            } else {
                rGrab.setPosition(0.13);//.13
                lGrab.setPosition(0.23);//0.23
            }
        }

        //Lift
        if (liftUp != last_liftUp || liftDown != last_liftDown) {
            if (liftUp > 0.1) {
                liftMotor.setPower(0.98);
                isBreak = true;
            } else if (liftDown > 0.1) {
                liftMotor.setPower(-0.6);
                isBreak = false;
            } else {
                liftMotor.setPower(0);
            }
        }

        //Telemetry
        telemetry.addData("fL servo pos: ", fLFound.getPosition());
        telemetry.addData("fR servo pos: ", fRFound.getPosition());
        telemetry.addData("bL servo pos: ", bLFound.getPosition());
        telemetry.addData("bR servo pos: ", bRFound.getPosition());

        //Update last variables
        last_armUp = armUp;
        last_lFoundationDown = lFoundationDown;
        last_rFoundationDown = rFoundationDown;
        last_grab = grab;
        last_liftUp = liftUp;
        last_liftDown = liftDown;
    }

    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
        liftMotor.setPower(0);
    }
}
