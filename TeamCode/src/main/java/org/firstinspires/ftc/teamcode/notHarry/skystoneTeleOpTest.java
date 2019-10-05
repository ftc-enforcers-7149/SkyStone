package org.firstinspires.ftc.teamcode.notHarry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpTest")
public class skystoneTeleOpTest extends OpMode {
    CRServo lArm, rArm, lGrab, rGrab;
    DcMotor fRight,fLeft,bRight,bLeft;

    boolean armUp, armDown;
    boolean gGrab,gRelease;
    float lDrive,rDrive;
    public void init(){
        //Servos
        lArm = hardwareMap.crservo.get("lArm");
        rArm = hardwareMap.crservo.get("rArm");
        lGrab = hardwareMap.crservo.get("lGrab");
        rGrab = hardwareMap.crservo.get("rGrab");
        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void loop(){
        armUp = gamepad1.b;
        armDown = gamepad1.a;
        gGrab = gamepad1.x;
        gRelease = gamepad1.y;
        lDrive = gamepad1.left_trigger;
        rDrive = gamepad1.right_trigger;


        if (armUp = true) {
            lArm.setPower(1);
            rArm.setPower(0);
        } else if (armDown = true) {
            lArm.setPower(0);
            rArm.setPower(1);
        } else if (gGrab = true) {
            lGrab.setPower(1);
            rGrab.setPower(0);
        } else if (gRelease = true) {
            lGrab.setPower(0);
            rGrab.setPower(1);
        }
        if (lDrive > 0.1){
            bLeft.setPower((lDrive*100)/128);
            fLeft.setPower((lDrive*100)/128);
        }else if (rDrive > 0.1) {
            bRight.setPower((rDrive*100)/128);
            fRight.setPower((rDrive*100)/128);
        }else {
            stop();
        }



    }
    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
