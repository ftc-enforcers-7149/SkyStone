package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.TestEncoderAuto;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

public class SamplePath {

    DriveTrain driveTrain;

    //Declaring object imports
    OdometryPositionClass odometryPositionClass;

    //Declaring variables
    String name;
    int step = 1;

    public SamplePath() {
        name = "test";
    }

    public void runSamplePath() {
        outputStep(1);
        driveTrain.driveStraight("forward", 5);
        driveTrain.Rotation(90);
        outputStep(2);
        driveTrain.driveStraight("forward", 5);


    }

    public int outputStep(int stepIn) {return stepIn;}

}
