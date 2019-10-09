package org.firstinspires.ftc.teamcode.NateTheGrate;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class test extends OpMode {
    DcMotor fRight, fLeft, bLeft, bRight;


    public void init () {
        fRight = hardwareMap.dcMotor.get("fRight");
        fLeft = hardwareMap.dcMotor.get("fLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");

        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void loop (){
        if (gamepad1.right_stick_y < 0.1  & gamepad1.left_stick_y < 0.1) {
            fRight.setPower(gamepad1.right_stick_y);
            bRight.setPower(gamepad1.right_stick_y);
            fLeft.setPower(gamepad1.left_stick_y);
            bLeft.setPower(gamepad1.left_stick_y);
        } else if (gamepad1.right_stick_y > 0.1 & gamepad1.left_stick_y > 0.1) {
            fRight.setPower(gamepad1.right_stick_y);
            bRight.setPower(gamepad1.right_stick_y);
            fLeft.setPower(gamepad1.left_stick_y);
            bLeft.setPower(gamepad1.left_stick_y);
        } else if (gamepad1.right_stick_y < 0.1 & gamepad1.left_stick_y > 0.1) {
            fRight.setPower(gamepad1.right_stick_y);
            bRight.setPower(gamepad1.right_stick_y);
            fLeft.setPower(gamepad1.left_stick_y);
            bLeft.setPower(gamepad1.left_stick_y);
        }else if (gamepad1.right_stick_y > 0.1 & gamepad1.left_stick_y < 0.1) {
            fRight.setPower(gamepad1.right_stick_y);
            bRight.setPower(gamepad1.right_stick_y);
            fLeft.setPower(gamepad1.left_stick_y);
            bLeft.setPower(gamepad1.left_stick_y);
        }else {
            stop();
        }
    }

    public void stop (){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
