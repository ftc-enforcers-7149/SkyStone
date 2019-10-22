package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Headless;

@TeleOp(name = "Detect Movement")
public class TestMovementDetection extends OpMode {

    MovementDetectionClass detection;
    Headless driveSystem;

    //Servos and motors not used for driving
    Servo lArm, rArm, lGrab, rGrab;

    double v1, v2, v3, v4;

    public void init() {
        detection = new MovementDetectionClass(hardwareMap, "distanceC", "distanceR", "distanceL", "fLeft", "bLeft", "fRight", "bRight");
        driveSystem = new Headless(hardwareMap, telemetry, "fLeft", "fRight","bLeft","bRight");
        v1 = 0; v2 = 0; v3 = 0; v4 = 0;

        //Servos
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");

        //Servo directions
        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);

        //Set initial positions
        lArm.setPosition(0.1); //0.4
        rArm.setPosition(0.05); //0.57
        lGrab.setPosition(0.2); //0.25
        rGrab.setPosition(0.25); //0.3
    }

    public void loop() {
        detection.update();
        boolean isFront = detection.isFrontMoving();
        boolean isLeft = detection.isLeftClose();
        boolean isRight = detection.isRightClose();
        String raw = detection.rawData();

        telemetry.addData("Moving in front? ", isFront);
        telemetry.addData("Is left close? ", isLeft);
        telemetry.addData("Is right close? ", isRight);
        telemetry.addLine(raw);
        Log.i("Raw Data: ", raw);

        if (Math.abs(gamepad1.left_stick_y) < 0.1 && Math.abs(gamepad1.left_stick_x) < 0.1 && Math.abs(gamepad1.right_stick_x) < 0.1) {
            v1 = 0;
            v2 = 0;
            v3 = 0;
            v4 = 0;
            if (isFront) {
                v1 -= 0.5;
                v2 -= 0.5;
                v3 -= 0.5;
                v4 -= 0.5;
            }
            /*if (isLeft) {
                v1 -= 0.25;
                v2 += 0.25;
                v3 += 0.25;
                v4 -= 0.25;
            }
            else if (isRight) {
                v1 += 0.25;
                v2 -= 0.25;
                v3 -= 0.25;
                v4 += 0.25;
            }*/

            //Getting the max value can assure that no motor will be set to a value above a certain point.
            double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));

            //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
            if (max > 0.5) {
                v1 /= max * (1 / 0.5);
                v2 /= max * (1 / 0.5);
                v3 /= max * (1 / 0.5);
                v4 /= max * (1 / 0.5);
            }

            detection.fLeft.setPower(v1);
            detection.fRight.setPower(v2);
            detection.bLeft.setPower(v3);
            detection.bRight.setPower(v4);
        }
        else {
            driveSystem.drive(gamepad1);
        }
    }
}