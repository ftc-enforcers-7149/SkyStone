package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.TestEncoderAuto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.PositionClass;


//THIS IS A TEST PROGRAM


public class SampleEncoder extends OpMode {


    //Declaring all of our object imports
    PositionClass positionClass;
    EncoderTestObject encoderTestObject;
    SamplePath samplePath;

    public void init() {

        positionClass = new PositionClass(0, 0);
        encoderTestObject = new EncoderTestObject(hardwareMap, "fLeft", "fRight", "bLeft", "bRight", "encoderX", "encoderY", "imu", 0, 0);

    }

    public void loop() {

        samplePath.runSamplePath();
        encoderTestObject.EncoderTrack("test", 1);

    }

    public void stop() {}

}
