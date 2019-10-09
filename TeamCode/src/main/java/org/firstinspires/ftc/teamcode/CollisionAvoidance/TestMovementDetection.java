package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name = "Detect Movement")
public class TestMovementDetection extends OpMode {

    MovementDetectionClass detection;

    double deltaTime;

    public void init() {
        detection = new MovementDetectionClass(hardwareMap, "distanceC", "distanceR", "distanceL", "fLeft", "bLeft", "fRight", "bRight");
    }

    public void start() {
        deltaTime = System.currentTimeMillis();
    }

    public void loop() {
        //detection.updateMovement();
        telemetry.addData("Moving Obstacle? ", detection.isMoving());
        telemetry.addLine(detection.rawData());

        detection.fLeft.setPower(-gamepad1.left_stick_y/1.5);
        detection.fRight.setPower(-gamepad1.left_stick_y/1.5);
        detection.bLeft.setPower(-gamepad1.left_stick_y/1.5);
        detection.bRight.setPower(-gamepad1.left_stick_y/1.5);

        deltaTime = System.currentTimeMillis();
    }
}