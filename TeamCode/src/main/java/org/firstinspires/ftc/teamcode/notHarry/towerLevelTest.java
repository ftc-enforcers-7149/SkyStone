package org.firstinspires.ftc.teamcode.notHarry;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;

//fish in a tube
@TeleOp(name = "towerLevelTest")
public class towerLevelTest extends OpMode {
    DcMotor lift;
    Servo lArm, rArm, lGrab, rGrab;
    DistanceSensor distanceLift;

    float liftUp, liftDown, grab,liftL;
    boolean levelPlus, levelMinus, pressP, pressM;
    Claw claw;

    int level;
    public void init(){
        //Servos
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");

        lift = hardwareMap.dcMotor.get("lift");

        distanceLift = hardwareMap.get(DistanceSensor.class, "distanceLift");

        //Motor directions
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);

        //Lift brake
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = new Claw(lArm, rArm, lGrab, rGrab);
    }

    public void loop(){
        grab = gamepad1.right_trigger;
        liftDown = gamepad1.left_trigger;
        liftL = gamepad1.left_stick_y;

        levelPlus = gamepad1.dpad_up;
        levelMinus = gamepad1.dpad_down;

        telemetry.addData("distanceLift",distanceLift.getDeviceName() );
        telemetry.addData("range", distanceLift.getDistance(DistanceUnit.CM));
        telemetry.addData("level", level);
        if(liftL > 0.1){
            lift.setPower(0.8);
        }
        else if(liftL < -0.1){
            lift.setPower(-0.4);
        }
        else{
            lift.setPower(0);
        }


        if(grab > 0.1){
            claw.grab();
        }
        else{
            claw.release();
        }

        if(!pressP){
            if (levelPlus) {
                if (level < 5){
                    level++;
                    goLevel("up");
                }
                pressP = true;
            }

        }
        else{
            if (!levelPlus){
                pressP = false;
            }
        }

        if(!pressM){
            if (levelMinus) {
                if(level > 0){
                    level--;
                    goLevel("down");
                }
                pressM = true;
            }

        }
        else{
            if (!levelMinus){
                pressM = false;
            }
        }


    }

    public void goLevel(String dir) {
        if (dir.equals("up")) {
            switch (level) {
                case 1:
                    while (distanceLift.getDistance(DistanceUnit.CM)< 10.5) {
                        lift.setPower(0.8);
                    }
                    lift.setPower(0);
                    break;
                case 2:
                    while (distanceLift.getDistance(DistanceUnit.CM)< 20) {
                        lift.setPower(0.8);
                    }
                    lift.setPower(0);
                    break;
                case 3:
                    while (distanceLift.getDistance(DistanceUnit.CM)< 30.5) {
                        lift.setPower(0.8);
                    }
                    lift.setPower(0);
                    break;
                case 4:
                    while (distanceLift.getDistance(DistanceUnit.CM)< 40.5) {
                        lift.setPower(0.8);
                    }
                    lift.setPower(0);
                    break;
                case 5:
                    while (distanceLift.getDistance(DistanceUnit.CM)< 54) {
                        lift.setPower(0.8);
                    }
                    lift.setPower(0);
                    break;
            }

        }
        if(dir.equals("down")) {
            switch (level) {
                case 1:
                    while (distanceLift.getDistance(DistanceUnit.CM) > 40.5) {
                        lift.setPower(-0.4);
                    }
                    lift.setPower(0);
                    break;
                case 2:
                    while (distanceLift.getDistance(DistanceUnit.CM) > 30.5) {
                        lift.setPower(-0.4);
                    }
                    lift.setPower(0);
                    break;
                case 3:
                    while (distanceLift.getDistance(DistanceUnit.CM) > 20) {
                        lift.setPower(-0.4);
                    }
                    lift.setPower(0);
                    break;
                case 4:
                    while (distanceLift.getDistance(DistanceUnit.CM) > 10.5) {
                        lift.setPower(-0.4);
                    }
                    lift.setPower(0);
                    break;
                case 5:
                    while (distanceLift.getDistance(DistanceUnit.CM) > 3) {
                        lift.setPower(-0.4);
                    }
                    lift.setPower(0);
                    break;

            }
        }
    }

    public void stop()
    {
        lift.setPower(0);
    }
}
