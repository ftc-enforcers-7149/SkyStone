package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Detect Movement")
public class TestMovementDetection extends OpMode {

    MovementDetectionClass detection;

    public void init() {
        detection = new MovementDetectionClass(hardwareMap, "distanceC", "distanceR", "distanceL", "fLeft", "bLeft", "fRight", "bRight");
    }

    public void loop() {

    }
}