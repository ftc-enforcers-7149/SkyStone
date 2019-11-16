/*
package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationV1;

public class TeleOpHeadless extends OpMode {

    //Drive train
    Headless driveSystem;
    FoundationV1 foundationV1;

    //Servos and motors not used for driving
    Servo lArm, rArm, lGrab, rGrab, lFound, rFound;
    DcMotor lift;

    //Input variables
    boolean armUp, armDown;
    double liftUp,liftDown;
    boolean foundationDown, foundationPressed, foundationState;
    boolean isBreak;
    float leftG,rightG;

    public void init() {
        //Initialize drive train
        driveSystem = new Headless(hardwareMap, telemetry, "fLeft", "fRight", "bLeft", "bRight");

        //Servos
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");
        lFound = hardwareMap.servo.get("lFound");
        rFound = hardwareMap.servo.get("rFound");
        //Lift
        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo directions
        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);
        lFound.setDirection(Servo.Direction.REVERSE);
        rFound.setDirection(Servo.Direction.FORWARD);

        //Set initial positions
        lArm.setPosition(0.1);
        rArm.setPosition(0.05);
        lGrab.setPosition(0.2);
        rGrab.setPosition(0.25);
        lFound.setPosition(0);
        rFound.setPosition(0);

        foundationV1 = new FoundationV1(lFound, rFound);

        isBreak = false;
        foundationPressed = false;
        foundationState = false;
    }

    public void loop() {
        //Get inputs
        armUp = gamepad2.y;
        armDown = gamepad2.a;
        rightG = gamepad2.right_trigger;
        leftG = gamepad2.left_trigger;
        liftDown = gamepad1.left_trigger;
        liftUp = gamepad1.right_trigger;
        foundationDown = gamepad1.a;

        //Do flip function
        if (armUp) {
            lArm.setPosition(0.1);
            rArm.setPosition(0.05);
        }
        else if(armDown){
            lArm.setPosition(0.65);
            rArm.setPosition(0.45);
        }

        //Grab with separate controls for each grabber
        if(rightG>0.1){
            rGrab.setPosition(0.15);
        }
        else{
            rGrab.setPosition(0.25);
        }

        if(leftG>0.1){
            lGrab.setPosition(0.1);
        }
        else{
            lGrab.setPosition(0.2);
        }

        //Toggle foundationV1
        if (foundationPressed) {
            if (!foundationDown) {
                foundationPressed = false;
                foundationState = !foundationState;
            }
        }
        else {
            if (foundationDown) {
                foundationPressed = true;
            }
        }
        if (foundationState) {
            foundationV1.up();
        }
        else {
            foundationV1.down();
        }

        //Do lift function
        if(liftUp > 0.1){
            lift.setPower(0.7);
            isBreak = true;
        }
        else if(liftDown > 0.1){
            lift.setPower(-0.05);
            isBreak = false;
        }
        else{
            if (isBreak) {
                lift.setPower(0.3);
            }
            else {
                lift.setPower(0);
            }
        }

        //Output telemetry for each servo / motor function
        telemetry.addData("Left Grabber Closed: ", lGrab.getPosition() == 0.1);
        telemetry.addData("Right Grabber Closed: ", rGrab.getPosition() == 0.15);
        telemetry.addLine("Arms are " + (lArm.getPosition() == 0.65 ? "down" : "up"));
        telemetry.addLine("Lift " + (lift.getPower() == 0.7 ? "is " : "isn't ") + "going up");
        telemetry.addLine("FoundationV1 grabbers are" + (lFound.getPosition() == 0.95 ? "down" : "up"));

        //Drive
        driveSystem.drive(gamepad1);
    }

    public void stop(){
        driveSystem.stop();
    }
}
*/
