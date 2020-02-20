package org.firstinspires.ftc.teamcode.TeleOpPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationV2;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@TeleOp(name = "TeleOp v2")
public class TeleOpV2_2 extends OpMode {
    //Subsystems
    Headless driveSystem;
    Gyroscope gyroscope;
    Claw claw;
    FoundationV2 foundation;

    //Hardware
    Servo fLFound, fRFound, bLFound, bRFound;
    Servo lArm, rArm, lGrab, rGrab;
    DcMotor fRight,fLeft,bRight,bLeft, liftMotor;

    float armUp, last_armUp=0; //arm variables
    float liftUp, last_liftUp=0, liftDown, last_liftDown=0; //lift variables
    float grab, last_grab=0; //grabber variables
    boolean smallGrab, last_smallGrab=false;

    boolean isBreak=false; // breaking for lift
    boolean lFoundationDown, last_lFoundationDown=false, rFoundationDown, last_rFoundationDown=false; //lift variable
    boolean armsDown=false;//used to only have arms work after pressing once

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

        //Initialize subsystems
        gyroscope = new Gyroscope(telemetry, hardwareMap);
        driveSystem = new Headless(gyroscope, fLeft, fRight, bLeft, bRight);
        claw = new Claw(lArm,rArm,lGrab,rGrab);
        foundation = new FoundationV2(fLFound,fRFound,bLFound,bRFound);
    }

    public void loop() {
        //Controls
        armUp = gamepad2.left_trigger;
        grab = gamepad2.right_trigger;
        liftUp=gamepad1.right_trigger;
        liftDown=gamepad1.left_trigger;
        lFoundationDown = gamepad1.left_bumper || gamepad2.x;
        rFoundationDown = gamepad1.right_bumper || gamepad2.b;
        smallGrab = gamepad2.a;

        //Drive
        driveSystem.drive(gamepad1);

        //FoundationV1 grabbers//
        //Left foundation
        if (lFoundationDown != last_lFoundationDown) {
            if (lFoundationDown) {
                foundation.lDown();
            } else {
                foundation.lUp();
            }
        }

        //Right foundation
        if (rFoundationDown != last_rFoundationDown) {
            if (rFoundationDown) {
                foundation.rDown();
            } else {
                foundation.rUp();
            }
        }

        //Arms
        if (armsDown) {
            if (armUp != last_armUp) {
                if (armUp > 0.1) {
                    claw.up();
                } else {
                    claw.down();
                }
            }
        }
        else if (armUp > 0.1) {
            armsDown=true;
        }

        //Grabber
        if ((grab != last_grab) || (smallGrab != last_smallGrab)) {
            //Double servo claw: r 0.2 l 0.28 closed, r 0.13 l 0.23 open
            if (grab > 0.1) {
                claw.grab();
            } else if (smallGrab){
                claw.grabVertical();
            } else {
                claw.release();
            }
        }

        //Lift
        if (liftUp != last_liftUp || liftDown != last_liftDown) {
            if (liftUp > 0.1) {
                liftMotor.setPower(liftUp);
                isBreak = true;
            } else if (liftDown > 0.1) {
                liftMotor.setPower(-liftDown*0.6);
                isBreak = false;
            } else if(isBreak){
                liftMotor.setPower(0.1);
            }
            else{
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
        last_smallGrab = smallGrab;
    }

    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
        liftMotor.setPower(0);
    }
}
