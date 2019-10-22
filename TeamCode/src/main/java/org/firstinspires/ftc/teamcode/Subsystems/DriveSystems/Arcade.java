package org.firstinspires.ftc.teamcode.Subsystems.DriveSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arcade {
    //DC Motors and velocity variables
    DcMotor fLeft, fRight, bLeft, bRight;
    double v1, v2, v3, v4; //In same order as motors

    //Variables for inputs
    double leftX, leftY, rightX;

    //Power limit
    double lim;

    //Telemetry object
    Telemetry telemetry;

    public Arcade(HardwareMap hardwareMap, Telemetry telemetry, String fl, String fr, String bl, String br) {
        //Hardware mapping the motors
        fLeft = hardwareMap.dcMotor.get(fl);
        fRight = hardwareMap.dcMotor.get(fr);
        bLeft = hardwareMap.dcMotor.get(bl);
        bRight = hardwareMap.dcMotor.get(br);

        //Reversing left motors
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;

        //Initialize variables
        lim = 0.9;
        v1 = 0;
        v2 = 0;
        v3 = 0;
        v4 = 0;
    }

    public void drive(Gamepad gamepad1) {
        //Getting inputs
        leftY = gamepad1.left_stick_y;
        leftX = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;

        v1 = leftY - leftX + rightX;
        v2 = leftY + leftX - rightX;
        v3 = leftY + leftX + rightX;
        v4 = leftY - leftX - rightX;

        //Getting the max value can assure that no motor will be set to a value above a certain point.
        double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));

        //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
        if (max > lim) {
            v1 /= max * (1/lim);
            v2 /= max * (1/lim);
            v3 /= max * (1/lim);
            v4 /= max * (1/lim);
        }

        //Only set brakes if no inputs are given from the joysticks
        //this makes it easier for the robot to move diagonally
        if (leftY == 0 && leftX == 0 && rightX == 0) {
            bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //Telemetry for the motor velocities
        telemetry.addData("fLeft: ", v1);
        telemetry.addData("fRight: ", v2);
        telemetry.addData("bLeft: ", v3);
        telemetry.addData("bRight: ", v4);

        //Setting each velocity to its respective motor
        fLeft.setPower(v1);
        fRight.setPower(v2);
        bLeft.setPower(v3);
        bRight.setPower(v4);
    }
}
