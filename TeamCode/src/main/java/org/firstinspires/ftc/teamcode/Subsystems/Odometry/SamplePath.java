package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;


//A sample classfile for a path.
//TODO: write in better error handling
public class SamplePath {

    DriveTrain driveTrain;

    //Declaring object imports
    private OdometryPosition odometryPositionClass = new OdometryPosition();

    //Declaring variables
    String name;
    int step = 1;
    OdometryPosition.Direction direction;


    //Constructor
    public SamplePath(){
        name = "test";
    }


    //Runs our actual path
    public void runSamplePath() {
        step = 1;
        switch(step) {
            case 1:
                direction = OdometryPosition.Direction.FORWARD;
                driveTrain.driveStraight("forward", 5);
                direction = OdometryPosition.Direction.TURNING;
                driveTrain.Rotation(90);
                step = 2;
                break;
            case 2:
                direction = OdometryPosition.Direction.FORWARD;
                driveTrain.driveStraight("forward", 5);
                break;

        }
    }

    //Outputs our step
    public int getStep() {return step;}

    public OdometryPosition.Direction returnDirection() {return direction;}

}
