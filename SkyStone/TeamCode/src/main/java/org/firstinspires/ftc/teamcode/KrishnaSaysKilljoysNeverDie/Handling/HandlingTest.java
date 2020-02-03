package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Handling;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.SamplePath;

public class HandlingTest extends OpMode {

    HandlingSystem handle;
    SamplePath samplePath;
    Gyroscope gyroscope;

    public void init() {
        handle = new HandlingSystem(hardwareMap, samplePath, gyroscope, "encX", "encY", "imu", 0, 0, "distC", "distR", "distL");
    }

    public void loop() {
        telemetry.addLine(handle.EasterEgg());
    }


}
