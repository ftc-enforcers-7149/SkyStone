package org.firstinspires.ftc.teamcode.Your50thPresidentJimmy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.SkyStonev1_1;

public class FoundationAuto extends SkyStonev1_1 {
    DcMotor fLeft, fRight, bLeft, bRight;
    int step=0;
    public void init() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void loop() {
        switch(step){
            case 0:driveStraight("forward", 50);
                break;
            case 1:driveStraight("backward", 34);
                break;
            case 2:Rotation(90);
                break;
            case 3:driveStraight("forward", 28);
                break;
        }
        step++;

    }

    public void stop() {
        bLeft.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        fRight.setPower(0);
    }
}
