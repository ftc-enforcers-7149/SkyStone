package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Headless;

@TeleOp(name = "Headless")
public class TeleOpHeadless extends OpMode {

    //Drive train
    Headless driveSystem;

    //Servos and motors not used for driving
    Servo lArm, rArm, lGrab, rGrab;
    DcMotor lift;

    //Input variables
    boolean armUp, armDown;
    boolean gGrab,gRelease;
    boolean liftUp,liftDown;
    float lDrive,rDrive,lStrafe,rStrafe;

    public void init() {
        //Initialize drive train
        driveSystem = new Headless(hardwareMap, telemetry, "fLeft", "fRight", "bLeft", "bRight");

        //Servos
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");
        //Lift
        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo directions
        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);

        //Set initial positions
        lArm.setPosition(0.4); //0.4
        rArm.setPosition(0.57); //0.57
        lGrab.setPosition(0.25); //0.25
        rGrab.setPosition(0.3); //0.3
    }

    public void loop() {
        //Get inputs
        armUp = gamepad1.y;
        armDown = gamepad1.a;
        gGrab = gamepad1.x;
        gRelease = gamepad1.b;
        liftUp = gamepad1.dpad_down;
        liftDown = gamepad1.dpad_up;
        lDrive = gamepad1.left_stick_y;
        rDrive = gamepad1.right_stick_y;
        lStrafe = gamepad1.left_trigger;
        rStrafe = gamepad1.right_trigger;

        //Do flip function
        if (armUp) {
            lArm.setPosition(0);
            rArm.setPosition(0);
        }
        else if(armDown){
            lArm.setPosition(0.40);
            rArm.setPosition(0.57);
        }

        //Do grab function
        if(gGrab){
            lGrab.setPosition(0);
            rGrab.setPosition(0);
        }
        else if(gRelease){
            lGrab.setPosition(0.25);
            rGrab.setPosition(0.3);
        }

        //Drive
        driveSystem.drive(gamepad1);

        //Do lift function
        if(liftUp){
            lift.setPower(0.6);
        }
        else if(liftDown){
            lift.setPower(-0.1);
        }
        else{
            lift.setPower(0);
        }
    }
}
