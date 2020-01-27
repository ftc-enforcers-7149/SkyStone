package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@TeleOp(name = "V3 Robot TeleOp")
public class IntakeAndClawV3 extends OpMode {

    //Drive System
    Headless driveSystem;
    Gyroscope gyro;

    //Hardware
    DcMotor fLeft, fRight, bLeft, bRight;
    DcMotor inLeft, inRight, lift;
    Servo arm, clawLeft, clawRight, stopper;
    DistanceSensor detectBlock;

    //Input
    boolean stopperButton, last_stopperButton=false;
    float clawButton, last_clawButton=0, armButton, last_armButton=0;
    boolean liftUp, last_liftUp=false, liftDown, last_liftDown=false;
    float intakeIn, last_intakeIn=0, intakeOut, last_intakeOut=0;

    enum State {
        GRAB,
        AVOID_CHANNEL,
        PLACE,
        LET_GO,
        RETRACT
    }
    State armState = State.GRAB;

    public void init() {
        //Map devices
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        inLeft = hardwareMap.dcMotor.get("lIntake");
        inRight = hardwareMap.dcMotor.get("rIntake");
        lift = hardwareMap.dcMotor.get("lift");

        arm = hardwareMap.servo.get("stoneLift");
        clawLeft = hardwareMap.servo.get("lClaw");
        clawRight = hardwareMap.servo.get("rClaw");
        stopper = hardwareMap.servo.get("stoneStop");

        detectBlock = hardwareMap.get(DistanceSensor.class, "detectBlock");

        //Set directions and brakes
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);

        inLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        inRight.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        inLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawLeft.setDirection(Servo.Direction.FORWARD);
        clawRight.setDirection(Servo.Direction.REVERSE);

        //Drive System initialization
        gyro = new Gyroscope(telemetry, hardwareMap);
        driveSystem = new Headless(gyro, fLeft, fRight, bLeft, bRight, false);

        //Initialize servo positions
        clawLeft.setPosition(0.65);
        clawRight.setPosition(0.6);

        arm.setPosition(0);
        stopper.setPosition(0.55);//90 degrees //0.4
    }

    public void loop() {
        //clawButton = gamepad1.left_trigger;
        armButton = gamepad1.right_trigger;
        //stopperButton = gamepad2.x;
        intakeIn = gamepad1.left_trigger;
        liftUp = gamepad1.right_bumper;
        liftDown = gamepad1.left_bumper;

        driveSystem.drive(gamepad1);

        //Control claw servos to grab block inside robot
        /*if (clawButton != last_clawButton) {
            if (clawButton > 0.1) {
                clawLeft.setPosition(0.25);
                clawRight.setPosition(0.25);
            } else {
                clawLeft.setPosition(0.65);
                clawRight.setPosition(0.6);
            }
        }*/

        //Control arm servo to move block from intake to outside robot
        /*if (armButton != last_armButton) {
            if (armButton > 0.1) {
                stopper.setPosition(0.16);
                clawLeft.setPosition(0.25);
                clawRight.setPosition(0.25);
                arm.setPosition(0.9);
            } else {
                clawLeft.setPosition(0.65);
                clawRight.setPosition(0.6);
                arm.setPosition(0);
                stopper.setPosition(0.53);
            }
        }*/

        switch (armState) {
            case GRAB:
                if (armButton > 0.1 && last_armButton <= 0.1) {
                    clawLeft.setPosition(0.33);
                    clawRight.setPosition(0.33);
                    stopper.setPosition(0.16);

                    armState = State.AVOID_CHANNEL;
                }
                break;
            case AVOID_CHANNEL:
                if (detectBlock.getDistance(DistanceUnit.CM) > 200) {
                    arm.setPosition(0.5);

                    armState = State.PLACE;
                }
                break;
            case PLACE:
                if (armButton > 0.1 && last_armButton <= 0.1) {
                    arm.setPosition(0.9);

                    armState = State.LET_GO;
                }
                break;
            case LET_GO:
                if (armButton < 0.1 && last_armButton >= 0.1) {
                    clawLeft.setPosition(0.65);
                    clawRight.setPosition(0.6);

                    armState = State.RETRACT;
                }
                break;
            case RETRACT:
                if (armButton > 0.1 && last_armButton <= 0.1) {
                    arm.setPosition(0);
                    stopper.setPosition(0.4);

                    armState = State.GRAB;
                }
                break;
        }

        //Control block stopper
        /*if (stopperButton != last_stopperButton) {
            if (stopperButton) {
                stopper.setPosition(0.53);
            } else {
                stopper.setPosition(0.16);
            }
        }*/

        //Control the two intake motors
        //Positive power rotates inward
        //Negative power rotates outward
        if (intakeIn > 0.1) {
            inLeft.setPower(1);
            inRight.setPower(1);
        } else if (intakeOut > 0.1) {
            inLeft.setPower(-1);
            inRight.setPower(-1);
        } /*else if (armState == State.GRAB && detectBlock.getDistance(DistanceUnit.CM) < 40 && detectBlock.getDistance(DistanceUnit.CM) > 3) {
            inLeft.setPower(1);
            inRight.setPower(1);
        }*/
        else {
            inLeft.setPower(0);
            inRight.setPower(0);
        }

        //Control lift motor
        //Positive power goes up
        //Negative power goes down
        if (liftUp != last_liftUp || liftDown != last_liftDown) {
            if (liftUp) {
                lift.setPower(1);
            } else if (liftDown) {
                lift.setPower(-0.7);
            } else {
                lift.setPower(0);
            }
        }

        telemetry.addData("Arm State: ", armState);
        telemetry.addData("Block Detected: ", detectBlock.getDistance(DistanceUnit.CM));

        last_clawButton = clawButton;
        last_armButton = armButton;
        last_stopperButton = stopperButton;
        last_intakeIn = intakeIn;
        last_intakeOut = intakeOut;
        last_liftUp = liftUp;
        last_liftDown = liftDown;
    }

    public void stop() {
        driveSystem.stop();
        inLeft.setPower(0);
        inRight.setPower(0);
        lift.setPower(0);
    }
}
