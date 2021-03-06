package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

//@Autonomous(name = "TestWebcam")
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
                telemetry.addData("Red of center", webcam.getRed(telemetry));
                break;
            case 1:
                break;
        }

        telemetry.addData("position",position);
        telemetry.addData("Step: ", step);
        //step++
    }

    public void stop() {

    }
}
