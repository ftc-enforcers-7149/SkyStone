package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@TeleOp(name = "Intake And Claw V3")
public class IntakeAndClawV3 extends OpMode {

    //Drive System
    Headless driveSystem;

    //Hardware
    DcMotor fLeft, fRight, bLeft, bRight;
    DcMotor inLeft, inRight;
    Servo arm, clawLeft, clawRight, stopper;

    //Input
    boolean clawButton, armButton, stopperButton;
    float intakeIn, intakeOut;

    public void init() {
        //Map devices
        /*fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");*/

        inLeft = hardwareMap.dcMotor.get("inLeft");
        inRight = hardwareMap.dcMotor.get("inRight");

        arm = hardwareMap.servo.get("arm");
        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight = hardwareMap.servo.get("clawRight");
        stopper = hardwareMap.servo.get("stopper");

        //Set directions and brakes
        /*fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);*/

        inLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        inRight.setDirection(DcMotorSimple.Direction.REVERSE);
        inLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawLeft.setDirection(Servo.Direction.FORWARD);
        clawRight.setDirection(Servo.Direction.REVERSE);

        //driveSystem = new Headless(new Gyroscope(telemetry, hardwareMap), fLeft, fRight, bLeft, bRight);
    }

    public void loop() {
        clawButton = gamepad1.a;
        armButton = gamepad1.b;
        intakeIn = gamepad1.right_trigger;
        intakeOut = gamepad1.left_trigger;

        //driveSystem.drive(gamepad1);

        //Control claw servos to grab block inside robot
        if(clawButton) {
            clawLeft.setPosition(0.25);
            clawRight.setPosition(0);
        }
        else {
            clawLeft.setPosition(0.65);
            clawRight.setPosition(0.27);
        }

        //Control arm servo to move block from intake to outside robot
        if (armButton) {
            arm.setPosition(0.95);
        }
        else {
            arm.setPosition(0);
        }

        //Control block stopper
        if (stopperButton) {
            stopper.setPosition(0);
        }

        //Control the two intake motors
        //Positive power rotates inward
        //Negative power rotates outward
        if (intakeIn > 0.1) {
            inLeft.setPower(1);
            inRight.setPower(1);
        }
        else if (intakeOut > 0.1) {
            inLeft.setPower(-1);
            inRight.setPower(-1);
        }
        else {
            inLeft.setPower(0);
            inRight.setPower(0);
        }
    }

    public void stop() {
        //driveSystem.stop();
        inLeft.setPower(0);
        inRight.setPower(0);
    }
}
