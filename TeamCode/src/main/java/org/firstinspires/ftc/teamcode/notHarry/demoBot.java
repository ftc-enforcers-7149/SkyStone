package org.firstinspires.ftc.teamcode.notHarry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;


//@TeleOp(name = "Demo Bot")
public class demoBot extends OpMode {
    Headless driveSystem;
    Gyroscope gyro;

    DcMotor fLeft, fRight, bLeft, bRight, lift;

    Servo lArm, rArm, claw;

    double arms, last_arms=0, grab, last_grab=0;
    boolean liftUp, last_liftUp=false, liftDown, last_liftDown=false;

    public void init(){
        //Motor initialization
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        lift = hardwareMap.dcMotor.get("lift");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo initialization
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        claw = hardwareMap.servo.get("claw");

        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        //Drive System initialization
        gyro = new Gyroscope(telemetry, hardwareMap);
        driveSystem = new Headless(gyro, fLeft, fRight, bLeft, bRight, false);//false);

    }
    public void loop(){
        //Inputs
        arms = gamepad1.left_trigger;
        grab = gamepad1.right_trigger;
        liftUp = gamepad1.dpad_up;
        liftDown = gamepad1.dpad_down;

        //Drive
        driveSystem.drive(gamepad1);

        //Arms
        if (arms != last_arms) {
            if (arms > 0.1) {
                lArm.setPosition(0.6);
                rArm.setPosition(0.8);
            }
            else {
                lArm.setPosition(0.15);
                rArm.setPosition(0.37);
            }
        }

        //Claw
        if (grab != last_grab) {
            if (grab > 0.1) {
                claw.setPosition(0.67);
            }
            else {
                claw.setPosition(0.95);
            }
        }

        //Lift
        if (liftUp != last_liftUp || liftDown != last_liftDown) {
            if (liftUp) {
                lift.setPower(0.75);
            }
            else if (liftDown) {
                lift.setPower(-0.5);
            }
            else {
                lift.setPower(0.1);
            }
        }

        //Update _last variables
        last_arms = arms;
        last_grab = grab;
        last_liftUp = liftUp;
        last_liftDown = liftDown;
    }

    public void stop(){
        driveSystem.stop();
    }

}
