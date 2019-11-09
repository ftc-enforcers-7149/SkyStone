package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;

@TeleOp(name = "TeleOp v2")
public class TeleOpV2 extends OpMode {
    //Drive train
    Headless driveSystem;

    Servo fLFound, fRFound, bLFound, bRFound;
    DcMotor fRight,fLeft,bRight,bLeft;

    boolean armUp, armDown;
    boolean isBreak=false;
    float fLeftG, fRightG;
    float liftUp,liftDown;//liftMove;
    boolean lFoundationDown, rFoundationDown;
    float lDrive,rDrive,lStrafe,rStrafe;
    public void init(){
        //Servos
        fLFound = hardwareMap.servo.get("fLFound");
        fRFound = hardwareMap.servo.get("fRFound");
        bLFound = hardwareMap.servo.get("bLFound");
        bRFound = hardwareMap.servo.get("bRFound");
        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        //Initialize drive train
        driveSystem = new Headless(hardwareMap, telemetry, "fLeft", "fRight", "bLeft", "bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        fLFound.setDirection(Servo.Direction.FORWARD);
        fRFound.setDirection(Servo.Direction.REVERSE);
        bLFound.setDirection(Servo.Direction.REVERSE);
        bRFound.setDirection(Servo.Direction.FORWARD);

    }//
    public void loop(){
        armUp = gamepad2.y;
        armDown = gamepad2.a;
        fRightG = gamepad2.right_trigger;
        fLeftG = gamepad2.left_trigger;
        //liftMove = gamepad2.left_stick_y;
        liftUp=gamepad1.right_trigger;
        liftDown=gamepad1.left_trigger;
        lFoundationDown = gamepad1.a;
        rFoundationDown = gamepad1.x;
        lDrive = gamepad1.left_stick_y;
        rDrive = gamepad1.right_stick_y;
        lStrafe = gamepad1.left_trigger;
        rStrafe = gamepad1.right_trigger;

        //Drive
        driveSystem.drive(gamepad1);


        if (lFoundationDown) {
            fLFound.setPosition(1);
            bLFound.setPosition(1);
        }
        else {
            fLFound.setPosition(0);
            bLFound.setPosition(0);
        }

        if (rFoundationDown) {
            fRFound.setPosition(1);
            bRFound.setPosition(1);
        }
        else {
            fRFound.setPosition(0.4);
            bRFound.setPosition(0.4);
        }




        /*if(gGrab){
            lGrab.setPosition(0.1);
            fRGrab.setPosition(0.1);
        }
        else if(gRelease){
            lGrab.setPosition(0.2);
            fRGrab.setPosition(0.25);
        }*/

        /*if(liftMove<-0.1){
            lift.setPower(0.7);
            isBreak=true;
        }
        else if(liftMove>0.1){
            lift.setPower(-0.05);
            isBreak=false;
        }*/


    }
    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
