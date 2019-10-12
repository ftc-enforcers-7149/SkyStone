package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TeleOpV1")
public class TeleOpV1 extends OpMode {
    Servo lArm, rArm, lGrab, rGrab;
    DcMotor fRight,fLeft,bRight,bLeft,lift;

    boolean armUp, armDown;
    boolean gGrab,gRelease;
    boolean liftUp,liftDown;
    float lDrive,rDrive,lStrafe,rStrafe;
    public void init(){
        //Servos
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");
        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        lift = hardwareMap.dcMotor.get("lift");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);

        lArm.setPosition(0.40);
        rArm.setPosition(0.57);
        lGrab.setPosition(0.25);
        rGrab.setPosition(0.3);

    }//
    public void loop(){
        armUp = gamepad2.y;
        armDown = gamepad2.a;
        gGrab = gamepad2.x;
        gRelease = gamepad2.b;
        liftDown = gamepad1.left_bumper;
        liftUp = gamepad1.right_bumper;
        lDrive = gamepad1.left_stick_y;
        rDrive = gamepad1.right_stick_y;
        lStrafe = gamepad1.left_trigger;
        rStrafe = gamepad1.right_trigger;


        if (armUp) {
            lArm.setPosition(0.6);
            rArm.setPosition(0.4);
        }
        else if(armDown){
            lArm.setPosition(1);
            rArm.setPosition(1);
        }

        if(gGrab){
            lGrab.setPosition(0);
            rGrab.setPosition(0);
        }
        else if(gRelease){
            lGrab.setPosition(0.2);
            rGrab.setPosition(0.25);
        }

        if(lStrafe<0.1 && rStrafe<0.1){
            fLeft.setPower(lDrive);
            bLeft.setPower(lDrive);
            fRight.setPower(rDrive);
            bRight.setPower(rDrive);
        }
        else if(lStrafe>0.1){
            fLeft.setPower(-lStrafe);
            bLeft.setPower(lStrafe);
            fRight.setPower(lStrafe);
            bRight.setPower(-lStrafe);
        }
        else{
            fLeft.setPower(rStrafe);
            bLeft.setPower(-rStrafe);
            fRight.setPower(-rStrafe);
            bRight.setPower(rStrafe);
        }

        if(liftUp){
            lift.setPower(0.1);
        }
        else if(liftDown){
            lift.setPower(-0.1);
        }
        else{
            lift.setPower(0);
        }




    }
    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
