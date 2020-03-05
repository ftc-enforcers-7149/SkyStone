package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV3;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;
import org.firstinspires.ftc.teamcode.Subsystems.Range;

//@TeleOp(name = "AccelerationTest")
public class AccelerationTest extends OpMode {

    DcMotor fRight, fLeft, bRight, bLeft;
    Gyroscope gyroscope;

    double xSpeed=0, ySpeed=0;
    double xPos=0, yPos=0;

    Headless driveSystem;

    double lastTime;

    public void init() {

        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);


        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gyroscope = new Gyroscope(telemetry, hardwareMap);

        driveSystem = new Headless(gyroscope, fLeft, fRight, bLeft, bRight, false);
    }

    public void loop(){
        telemetry.addData("x pos", xPos);
        telemetry.addData("y pos", yPos);
        telemetry.addLine();
        telemetry.addData("x speed", xSpeed);
        telemetry.addData("y speed", ySpeed);
        telemetry.addLine();
        telemetry.addData("x accel:", gyroscope.getXAccel());
        telemetry.addData("y accel:", gyroscope.getYAccel());

        driveSystem.drive(gamepad1);

        xSpeed += gyroscope.getXAccel() * (System.currentTimeMillis() - lastTime)/1000;
        ySpeed += gyroscope.getYAccel() * (System.currentTimeMillis() - lastTime)/1000;

        xPos += xSpeed * (System.currentTimeMillis() - lastTime)/1000;
        yPos += ySpeed * (System.currentTimeMillis() - lastTime)/1000;

        lastTime = System.currentTimeMillis();
    }
}
