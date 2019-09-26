package org.firstinspires.ftc.teamcode.UpNAdam;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.SkyStonev1_1;

@Autonomous(name="auto 1")
public class ClassTest extends OpMode {
    public DcMotor fLeft, fRight, bLeft, bRight;
    int step=0;
    Movement robot;
    public void init() {
        //Hardware mapping of the four motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.REVERSE);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot=new Movement(bLeft,bRight,fLeft,fRight);
        //
    }

    public void loop() {
        switch(step) {
            case 0:
                robot.driveStraight("forward", 50);
                break;
        }
        step++;

    }

    public void stop() {
      super.stop();
    }
}
