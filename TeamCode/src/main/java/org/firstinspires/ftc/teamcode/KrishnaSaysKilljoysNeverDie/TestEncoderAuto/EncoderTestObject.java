package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.TestEncoderAuto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderTestObject{

    //Declaring hardware
    DcMotor fLeft, fRight, bLeft, bRight;


    //Initializing position
    OdometryPositionClass odometryPositionClass;

    //Variables
    double x, y;


    public EncoderTestObject(HardwareMap hardwareMap, String fL, String fR, String bL, String bR, String encX, String encY, String imumap, double posX, double posY){
        x = posX;
        y = posY;

        odometryPositionClass = new OdometryPositionClass(hardwareMap, fL, fR, bL, bR, encX, encY, imumap, posX, posY);
        startOdometry();
    }

    public void startOdometry() {
        x = odometryPositionClass.getPositionX();
        y = odometryPositionClass.getPositionY();
    }


    /**
     * This method handles all of the sub-methods in this class. It takes input and accurately moves the robot.
     * @param path the path used for the encoders.
     */
    public void EncoderTrack(String path, int step) {

        double encY = odometryPositionClass.positionY;
        double encX = odometryPositionClass.positionX;

        double[] tracking = OnTrack(path, step, encY, encX);
        if(tracking[0] == 1) {
            //TODO program in movement handling

        }

    }


    /**
     * This returns a double array. OnTrack[0] returns a 0 or a 1, which indicates whether the robot
     * is on-path (0) or not (1). OnTrack[1] returns the total x displacement (if any) and OnTrack[2] returns
     * the total y displacement. Note that each path has to be programmed in until I can figure out a way to
     * do it in the class file.
     * @param path
     * @param encoderY
     * @param encoderX
     * @return
     */
    private double[] OnTrack(String path, int step, double encoderY, double encoderX) {

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
