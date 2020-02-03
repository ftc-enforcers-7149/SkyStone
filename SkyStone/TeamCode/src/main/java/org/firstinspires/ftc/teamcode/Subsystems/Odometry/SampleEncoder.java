package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;


//THIS IS A TEST PROGRAM
//This program tests all of our classes
public class SampleEncoder extends OpMode {


    //Declaring all of our object imports;
    OdometryEncoder odometryEncoder;
    SamplePath samplePath;
    Gyroscope gyroscope;

    public void init() {

        //Declaring encoder test object
        odometryEncoder = new OdometryEncoder(hardwareMap, "encoderX", "encoderY", "imu", 0, 0, gyroscope);

    }

    public void loop() {

        //Declares our sample path
        samplePath.runSamplePath();
        odometryEncoder.OnTrack("test", samplePath.getStep(), samplePath.getDirection());

    }

    public void stop() {}

}
