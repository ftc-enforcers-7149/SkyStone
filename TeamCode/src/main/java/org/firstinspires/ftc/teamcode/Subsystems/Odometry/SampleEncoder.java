package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


//THIS IS A TEST PROGRAM
//This program tests all of our classes
public class SampleEncoder extends OpMode {


    //Declaring all of our object imports;
    OdometryEncoder encoderTestObject;
    SamplePath samplePath;

    public void init() {

        //Declaring encoder test object
        encoderTestObject = new OdometryEncoder(hardwareMap, "fLeft", "fRight", "bLeft", "bRight", "encoderX", "encoderY", "imu", 0, 0);

    }

    public void loop() {

        //Declares our sample path
        samplePath.runSamplePath();
        encoderTestObject.OnTrack("test", samplePath.getStep(), samplePath.returnDirection());

    }

    public void stop() {}

}
