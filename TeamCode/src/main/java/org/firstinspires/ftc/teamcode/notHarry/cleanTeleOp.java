package org.firstinspires.ftc.teamcode.notHarry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Foundation;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;

public class cleanTeleOp extends ParentInit {
    DcMotor fLeft, fRight, bLeft, bRight;
    Claw  claw;
    Foundation foundation;
    Headless driveSystem;
    boolean armUp, armDown;
    boolean isBreak=false;
    float leftG,rightG;
    float liftUp,liftDown;
    boolean foundationDown;
    float lDrive,rDrive,lStrafe,rStrafe;

    public void init() {
    claw = new Claw(lArm,rArm,lGrab,rGrab);
    foundation = new Foundation(lFound,rFound);
    driveSystem = new Headless(hardwareMap, telemetry, "fLeft", "fRight", "bLeft", "bRight");
    }
    public void loop() {
        armUp = gamepad2.y;
        armDown = gamepad2.a;
        rightG = gamepad2.right_trigger;
        leftG = gamepad2.left_trigger;
        liftDown = gamepad1.left_trigger;
        liftUp = gamepad1.right_trigger;
        foundationDown = gamepad1.a;
        lDrive = gamepad1.left_stick_y;
        rDrive = gamepad1.right_stick_y;
        lStrafe = gamepad1.left_trigger;
        rStrafe = gamepad1.right_trigger;

        //Drive
        driveSystem.drive(gamepad1);

        if (armUp) {
            claw.up();
        }
        else if(armDown){
            claw.down();
        }

        if(rightG > 0.1 || leftG > 0.1){
           claw.grab();
        }
        else{
           claw.release();
        }


        if (foundationDown) {
            foundation.up();
        }
        else {
            foundation.down();
        }
        /*if(gGrab){
            lGrab.setPosition(0.1);
            rGrab.setPosition(0.1);
        }
        else if(gRelease){
            lGrab.setPosition(0.2);
            rGrab.setPosition(0.25);
        }*/

        if(liftUp>0.1){
            lift.setPower(0.7);
            isBreak=true;
        }
        else if(liftDown>0.1){
            lift.setPower(-0.05);
            isBreak=false;
        }
        else{
            if(isBreak){
                lift.setPower(0.3);
            }
            else{
                lift.setPower(0.0);
            }
        }
    }
    public void stop() {
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}

