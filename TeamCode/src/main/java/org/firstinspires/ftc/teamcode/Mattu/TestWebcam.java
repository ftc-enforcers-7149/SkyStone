package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@Autonomous(name = "TestWebcam")
public class TestWebcam extends OpMode {

    Webcam webcam;
    int step=0;
    String position="";

    public void init() {
        webcam=new Webcam(hardwareMap);
    }

    public void loop() {
        switch(step) {
            case 0:
                double startTime = System.currentTimeMillis();
                while (System.currentTimeMillis() < startTime + 2000) {
                    position = webcam.getBitmapPos(telemetry);
                }
                webcam.deactivate();
                break;
            case 1:
                break;
        }

        telemetry.addData("position",position);
        telemetry.addData("Step: ", step);
        step++;
    }

    public void stop() {

    }
}
