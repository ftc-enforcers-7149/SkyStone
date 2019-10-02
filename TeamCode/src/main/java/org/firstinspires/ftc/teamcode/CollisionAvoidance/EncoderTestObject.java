package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.PositionClass;

public class EncoderTestObject{

    //Declaring hardware
    DcMotor fLeft, fRight, bLeft, bRight;


    //Initializing position
    PositionClass positionClass;

    EncoderTestObject(HardwareMap hardwareMap, String fL, String fR, String bL, String bR){

        fLeft = hardwareMap.dcMotor.get(fL);
        fRight = hardwareMap.dcMotor.get(fR);
        bLeft = hardwareMap.dcMotor.get(bL);
        bRight = hardwareMap.dcMotor.get(bR);

    }

    public void startOdometry(double encoderY, double encoderX) {
        positionClass.updatePosition(encoderX, encoderY);
    }


    /**
     * This method handles all of the sub-methods in this class. It takes input and accurately moves the robot.
     * @param path the path used for the encoders.
     * @param encoderY the y encoder input
     * @param encoderX the x encoder input
     */
    public void EncoderTrack(String path, int step, double encoderY, double encoderX) {

        double[] tracking = OnTrack(path, step, encoderY, encoderX);
        if(tracking[0] == 1) {

        }

    }


    /**
     * This returns a double array. OnTrack[0] returns a 0 or a 1, which indicates whether the robot
     * is on-path (0) or not (1). OnTrack[1] returns the total x displacement (if any) and
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
                    else if(encoderY > 10 || encoderY < 7) {
                        returnArray[0] = 1;
                        returnArray[2] = encoderY - 8;
                    }





            }

        }

        return returnArray;


    }

}
