package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

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
                while (System.currentTimeMillis() < startTime + 3000) {
                    position = webcam.getBitmapPos();
                }
                break;
            case 1:
                webcam.captureFrameToFile();
                break;
        }

        telemetry.addData("position",position);
        telemetry.addData("Step: ", step);
        step++;
    }

    public void stop() {

    }
}
