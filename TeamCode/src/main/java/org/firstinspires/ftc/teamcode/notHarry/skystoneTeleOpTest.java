package org.firstinspires.ftc.teamcode.notHarry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "claw test")
public class skystoneTeleOpTest extends OpMode {
    Servo lArm, rArm, lGrab, rGrab;
    DcMotor fRight,fLeft,bRight,bLeft;

    boolean armUp, armDown;
    boolean gGrab,gRelease;
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

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);

    }//
    public void loop(){
        armUp = gamepad1.b;
        armDown = gamepad1.a;
        gGrab = gamepad1.x;
        gRelease = gamepad1.y;
        lDrive = gamepad1.left_stick_y;
        rDrive = gamepad1.right_stick_y;
        lStrafe = gamepad1.left_trigger;
        rStrafe = gamepad1.right_trigger;


        if (armUp) {
            lArm.setPosition(0);
            rArm.setPosition(0);
        }
        else if(armDown){
            lArm.setPosition(0.40);
            rArm.setPosition(0.57);
        }

        if(gGrab){
            lGrab.setPosition(0);
            rGrab.setPosition(0);
        }
        else if(gRelease){
            lGrab.setPosition(0.25);
            rGrab.setPosition(0.2);
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




    }
    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
