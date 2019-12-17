package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

//TODO: write in better error handling
public class OdometryEncoder {



    //Initializing position
    OdometryPosition odometryPositionClass;

    //Variables
    double x, y;


    //Our constructor requires: hardware map, 4 motors, 2 encoders, an imu, and 2 positions.
    public OdometryEncoder(HardwareMap hardwareMap, String encX, String encY, String imumap, double posX, double posY, Gyroscope gyroscope){

        //Initializes start x
        x = posX;
        y = posY;

        //Initializes odometry pos class
        odometryPositionClass = new OdometryPosition(hardwareMap, encX, encY, imumap, posX, posY, gyroscope);

        //Starts our odometry tracking
        startOdometry();
    }

    public void startOdometry() {
        x = odometryPositionClass.getPositionX();
        y = odometryPositionClass.getPositionY();
    }



    /**
     * This returns a double array. OnTrack[0] returns a 0 or a 1, which indicates whether the robot
     * is on-path (0) or not (1). OnTrack[1] returns the total x displacement (if any) and OnTrack[2] returns
     * the total y displacement. Note that each path has to be programmed in until I can figure out a way to
     * do it in the class file.
     * @param path path we're on
     * @param step step of the path
     * @return
     */
    public double[] OnTrack(String path, int step, OdometryPosition.Direction direction) {

        odometryPositionClass.updatePosition(direction);

        double encoderX = odometryPositionClass.positionX;
        double encoderY = odometryPositionClass.positionY;

        double[] returnArray = new double[3];

        if(path.equals("test")) {

            switch(step) {

                case 1:
                    if(encoderX > 5 || encoderX < 0) {
                        returnArray[0] = 1;
                        returnArray[1] = encoderX - 5;
                    }
                    if(encoderY > 10 || encoderY < 7) {
                        returnArray[0] = 1;
                        returnArray[2] = encoderY - 8;
                    }
                    break;

                case 2:
                    if(encoderX > 5 || encoderX < 4) {
                        returnArray[0] = 1;
                        returnArray[1] = encoderX - 5;
                    }
                    if(encoderY > 5 || encoderY < 0) {
                        returnArray[0] = 1;
                        returnArray[2] = encoderY - 5;
                    }





            }

        }

        return returnArray;


    }

}
