package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.TestEncoderAuto;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;


//A sample classfile for a path.
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


    //Runs our actual path
    public void runSamplePath() {
        step = 1;
        driveTrain.driveStraight("forward", 5);
        driveTrain.Rotation(90);
        step = 2;
        driveTrain.driveStraight("forward", 5);


    }

    //Outputs our step
    public int getStep() {return step;}

}
