package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;


//A sample classfile for a path.
public class SamplePath {

    DriveTrain driveTrain;

    //Declaring object imports
    private OdometryPositionClass odometryPositionClass = new OdometryPositionClass();

    //Declaring variables
    String name;
    int step = 1;
    OdometryPositionClass.Direction direction;


    //Constructor
    public SamplePath(){
        name = "test";
    }


    //Runs our actual path
    public void runSamplePath() {
        step = 1;
        switch(step) {
            case 1:
                direction = OdometryPositionClass.Direction.FORWARD;
                driveTrain.driveStraight("forward", 5);
                driveTrain.Rotation(90);
                step = 2;
                break;
            case 2:
                direction = OdometryPositionClass.Direction.FORWARD;
                driveTrain.driveStraight("forward", 5);
                break;

        }
    }

    //Outputs our step
    public int getStep() {return step;}

    public OdometryPositionClass.Direction returnDirection() {return direction;}

}
