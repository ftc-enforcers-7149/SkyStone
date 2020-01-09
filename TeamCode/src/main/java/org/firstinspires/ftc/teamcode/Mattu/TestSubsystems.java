package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV3;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationV2;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;

/**
 * This class was made to test various subsystems and functions of the robot
 * It uses controller input, but is done in an autonomous fashion
 * As of right now, it can drive by distance in 4 directions, rotate to 4 angles, use claw, and use foundation
 */
@Autonomous(name = "TestSubsystems")
public class TestSubsystems extends OpMode {

    //Robot automations
    private DriveTrainV3 driveTrain;
    private OdometryPosition.Direction direction = OdometryPosition.Direction.FORWARD;

    private FoundationV2 foundation;
    private Claw claw;

    private Directions driveDir;
    private double rotateAngle;

    //Hardware
    private DcMotor fRight, fLeft, bRight, bLeft;
    private Servo lArm, rArm, lGrab, rGrab, fLFound, fRFound, bLFound, bRFound;
    private Gyroscope gyroscope;

    //Control
    private boolean drive, rotate, grabber, found;
    private boolean driving, rotating;

    //Dpad for driving
    private boolean dLeft, last_dLeft=false, dRight, last_dRight=false, dUp, last_dUp=false, dDown, last_dDown=false;

    //Standard buttons for rotating
    private boolean a, last_a=false, b, last_b=false, x, last_x=false, y, last_y=false;

    //Triggers for claw and foundation
    private float grab, last_grab=0, arms, last_arms=0;
    private boolean lFound, last_lFound=false, rFound, last_rFound=false;

    public void init() {
        //Servos
        fLFound = hardwareMap.servo.get("fLFound");
        fRFound = hardwareMap.servo.get("fRFound");
        bLFound = hardwareMap.servo.get("bLFound");
        bRFound = hardwareMap.servo.get("bRFound");
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");

        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        rGrab.setDirection(Servo.Direction.FORWARD);

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servo directions
        fLFound.setDirection(Servo.Direction.REVERSE);
        fRFound.setDirection(Servo.Direction.FORWARD);
        bLFound.setDirection(Servo.Direction.FORWARD);
        bRFound.setDirection(Servo.Direction.REVERSE);

        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);
        lGrab.setDirection(Servo.Direction.REVERSE);
        //Initialize foundation and claw
        foundation = new FoundationV2(fLFound,fRFound,bLFound,bRFound);
        claw = new Claw(lArm,rArm,lGrab,rGrab);

        foundation.lUp();
        foundation.rUp();

        claw.release();
        claw.up();

        //Set which things to test
        drive = false;
        rotate = false;
        grabber = false;
        found = false;
    }

    public void start() {
        if (drive || rotate) {
            gyroscope = new Gyroscope(telemetry, hardwareMap);
            driveTrain = new DriveTrainV3(hardwareMap, telemetry, fLeft, fRight, bLeft, bRight, gyroscope);
        }
    }

    public void loop() {

        if (drive) {
            driveTrain.updateOdom(direction);

            telemetry.addData("Position: ", "(" + driveTrain.getPosX() + ", " + driveTrain.getPosY() + ")");
        }

        if (rotate) {
            telemetry.addData("Angle: ", gyroscope.getYaw());
        }

        //Handle input
        if (drive && !driving && !rotating) {
            dUp = gamepad1.dpad_up;
            dDown = gamepad1.dpad_down;
            dLeft = gamepad1.dpad_left;
            dRight = gamepad1.dpad_right;

            //Dpad Up
            if(!last_dUp){
                if (dUp) {
                    driveDir = Directions.FORWARD;
                    driving = true;
                    last_dUp = true;
                }
            }
            else{
                if (!dUp) {
                    last_dUp = false;
                }
            }

            //Dpad Down
            if(!last_dDown){
                if (dDown) {
                    driveDir = Directions.BACKWARD;
                    driving = true;
                    last_dDown = true;
                }
            }
            else{
                if (!dDown) {
                    last_dDown = false;
                }
            }

            //Dpad Left
            if(!last_dLeft){
                if (dLeft) {
                    driveDir = Directions.LEFT;
                    driving = true;
                    last_dLeft = true;
                }
            }
            else{
                if (!dLeft) {
                    last_dLeft = false;
                }
            }

            //Dpad Right
            if(!last_dRight){
                if (dRight) {
                    driveDir = Directions.RIGHT;
                    driving = true;
                    last_dRight = true;
                }
            }
            else{
                if (!dRight) {
                    last_dRight = false;
                }
            }
        }

        if (rotate && !rotating && !driving) {
            a = gamepad1.a;
            b = gamepad1.b;
            x = gamepad1.x;
            y = gamepad1.y;

            //Button A
            if(!last_a){
                if (a) {
                    rotateAngle = 180;
                    rotating = true;
                    last_a = true;
                }
            }
            else{
                if (!a) {
                    last_a = false;
                }
            }

            //Button B
            if(!last_b){
                if (b) {
                    rotateAngle = 90;
                    rotating = true;
                    last_b = true;
                }
            }
            else{
                if (!b) {
                    last_b = false;
                }
            }

            //Button X
            if(!last_x){
                if (x) {
                    rotateAngle = 270;
                    rotating = true;
                    last_x = true;
                }
            }
            else{
                if (!x) {
                    last_x = false;
                }
            }

            //Button Y
            if(!last_y){
                if (y) {
                    rotateAngle = 0;
                    rotating = true;
                    last_y = true;
                }
            }
            else{
                if (!y) {
                    last_y = false;
                }
            }
        }

        if (grabber) {
            grab = gamepad1.right_trigger;
            arms = gamepad1.left_trigger;

            //Grabbers
            if (grab != last_grab) {
                if (grab > 0.1) {
                    claw.grab();
                }
                else {
                    claw.release();
                }
            }

            //Arms
            if (arms != last_arms) {
                if (arms > 0.1) {
                    claw.up();
                }
                else {
                    claw.down();
                }
            }
        }

        if (found) {
            lFound = gamepad1.left_bumper;
            rFound = gamepad1.right_bumper;

            //Left Foundation
            if (lFound != last_lFound) {
                if (lFound) {
                    foundation.lDown();
                }
                else {
                    foundation.lUp();
                }
            }

            //Right Foundation
            if (rFound != last_rFound) {
                if (rFound) {
                    foundation.rDown();
                }
                else {
                    foundation.rUp();
                }
            }
        }

        //Telemetry for hardware
        if (grabber) {
            telemetry.addData("Grabbing? ", grab > 0.1 ? "YES" : "NO");
            telemetry.addData("Arms Raised? ", arms > 0.1 ? "YES" : "NO");
        }

        if (found) {
            telemetry.addData("Left Foundation? ", lFound ? "YES" : "NO");
            telemetry.addData("Right Foundation? ", rFound ? "YES" : "NO");
        }

        //Handle actions
        if (drive && driving) {
            if (driveTrain.driveStraight(driveDir, 10, 0.5)) {
                driving = false;
            }
        }

        if (rotate && rotating) {
            direction = OdometryPosition.Direction.TURNING;

            if (driveTrain.rotate(rotateAngle)) {
                direction = OdometryPosition.Direction.FORWARD;
                rotating = false;
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
