package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.TestEncoderAuto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.PositionClass;


//THIS IS A TEST PROGRAM
//This program tests all of our classes
public class SampleEncoder extends OpMode {


    //Declaring all of our object imports;
    EncoderTestObject encoderTestObject;
    SamplePath samplePath;

    public void init() {

        //Declaring encoder test object
        encoderTestObject = new EncoderTestObject(hardwareMap, "fLeft", "fRight", "bLeft", "bRight", "encoderX", "encoderY", "imu", 0, 0);

    }

    public void loop() {

        //Declares our sample path
        samplePath.runSamplePath();
        encoderTestObject.EncoderTrack("test", samplePath.getStep());

    }

    public void stop() {}

}
