package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV1;


//A sample classfile for a path.
//TODO: write in better error handling
public class SamplePath {

    DriveTrainV1 driveTrainV1;

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
                driveTrainV1.driveStraight("forward", 5);
                direction = OdometryPosition.Direction.TURNING;
                //driveTrainV1.rotation(90,angles);
                step = 2;
                break;
            case 2:
                direction = OdometryPosition.Direction.FORWARD;
                driveTrainV1.driveStraight("forward", 5);
                break;

        }
    }

    //Outputs our step
    public int getStep() {return step;}

    public OdometryPosition.Direction getDirection() {return direction;}

    public String getPath() {return name;}


}
