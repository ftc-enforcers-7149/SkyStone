package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name = "Detect Movement")
public class TestMovementDetection extends OpMode {

    MovementDetectionClass detection;

    double deltaTime;

    double v1, v2, v3, v4;

    public void init() {
        detection = new MovementDetectionClass(hardwareMap, "distanceC", "distanceR", "distanceL", "fLeft", "bLeft", "fRight", "bRight");
        v1 = 0; v2 = 0; v3 = 0; v4 = 0;
    }

    public void start() {
        deltaTime = System.currentTimeMillis();
    }

    public void loop() {
        boolean isFront = detection.isFrontMoving();
        boolean isLeft = detection.isLeftMoving();
        boolean isRight = detection.isRightMovement();
        String raw = detection.rawData();

        telemetry.addData("Moving in front? ", isFront);
        telemetry.addData("Moving on left? ", isLeft);
        telemetry.addData("Moving on right? ", isRight);
        telemetry.addLine(raw);

        if (isFront) {
            v1 = -0.7;
            v2 = -0.7;
            v3 = -0.7;
            v4 = -0.7;
        }
        else if (isLeft) {
            v1 = -0.8;
            v2 = 0.8;
            v3 = 0.8;
            v4 = -0.8;
        }
        else if (isRight) {
            v1 = 0.8;
            v2 = -0.8;
            v3 = -0.8;
            v4 = 0.8;
        }
        else {
            v1 = 0; v2 = 0; v3 = 0; v4 = 0;
        }

        detection.fLeft.setPower(v1);
        detection.fRight.setPower(v2);
        detection.bLeft.setPower(v3);
        detection.bRight.setPower(v4);

        deltaTime = System.currentTimeMillis();
    }
}