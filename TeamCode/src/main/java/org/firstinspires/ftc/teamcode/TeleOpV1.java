/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;

//@TeleOp(name = "TeleOp")
public class TeleOpV1 extends OpMode {
    //Drive train
    Headless driveSystem;

    Servo lArm, rArm, lGrab, rGrab, lFound, rFound;
    DcMotor fRight,fLeft,bRight,bLeft,lift;

    boolean armUp, armDown;
    boolean isBreak=false;
    float leftG,rightG;
    float liftUp,liftDown;//liftMove;
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
        //driveSystem = new Headless(hardwareMap, telemetry, "fLeft", "fRight", "bLeft", "bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.FORWARD);
        rGrab.setDirection(Servo.Direction.REVERSE);
        lFound.setDirection(Servo.Direction.REVERSE);
        rFound.setDirection(Servo.Direction.FORWARD);


        lArm.setPosition(0.25);
        rArm.setPosition(0.25);
        lGrab.setPosition(0);
        rGrab.setPosition(0);//0.25
        lFound.setPosition(0);
        rFound.setPosition(0);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }//
    public void loop(){
        armUp = gamepad2.y;
        armDown = gamepad2.a;
        rightG = gamepad2.right_trigger;
        leftG = gamepad2.left_trigger;
        //liftMove = gamepad2.left_stick_y;
        liftUp=gamepad1.right_trigger;
        liftDown=gamepad1.left_trigger;
        foundationDown = gamepad1.a;
        lDrive = gamepad1.left_stick_y;
        rDrive = gamepad1.right_stick_y;
        lStrafe = gamepad1.left_trigger;
        rStrafe = gamepad1.right_trigger;

        //Drive
        driveSystem.drive(gamepad1);

        if (armUp) {
            lArm.setPosition(0.25);
            rArm.setPosition(0.25);
        }
        else if(armDown){
            lArm.setPosition(1);
            rArm.setPosition(1);
        }

        if(rightG>0.1){
            rGrab.setPosition(0.16);//0.15
        }
        else{
            rGrab.setPosition(0);//0.25
        }

        if(leftG>0.1){
            lGrab.setPosition(0.16);
        }
        else{
            lGrab.setPosition(0);
        }

        if (foundationDown) {
            lFound.setPosition(1);
            rFound.setPosition(1);
        }
        else {
            lFound.setPosition(0);
            rFound.setPosition(0);
        }
        */
/*if(gGrab){
            lGrab.setPosition(0.1);
            rGrab.setPosition(0.1);
        }
        else if(gRelease){
            lGrab.setPosition(0.2);
            rGrab.setPosition(0.25);
        }*//*


        */
/*if(liftMove<-0.1){
            lift.setPower(0.7);
            isBreak=true;
        }
        else if(liftMove>0.1){
            lift.setPower(-0.05);
            isBreak=false;
        }*//*


        if(liftUp>0.1){
            lift.setPower(0.7);
            isBreak=true;
        }
        else if(liftDown>0.1){
            lift.setPower(-0.05);
           isBreak=false;
        }
        else{
            if(isBreak){
                lift.setPower(0.3);
            }
            else{
                lift.setPower(0.0);
            }
        }

    }
    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
*/
