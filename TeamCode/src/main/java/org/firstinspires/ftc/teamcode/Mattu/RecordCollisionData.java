package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

/**
 * This is a concept file for code to be implemented in CollisionAvoidance/CollisionData
 */

public class RecordCollisionData extends OpMode {

    //Array used to hold power and distance values for telemetry
    public double[][] dataPoints;

    //Power set to the motors (in this case front left)
    public double powerFL;

    //Distance read from front facing sensor
    public double distF;

    //Gamepad inputs for changing tests
    public boolean buttonA, bumperL, bumperR;

    //loops is used to count the amount of loops the program has run through while testing
    //count is used to set the next index in datapoints every some amount of loops
    public int loops, count;

    //Used to determine when test has ended
    public boolean pause;

    public void init() {
        //double array that can hold 100 x 2 values
        dataPoints = new double[100][2];

        //Initialize variables
        pause = false;
        count = 0;
        loops = 0;
    }

    public void loop() {
        //General telemetry
        telemetry.addData("Done? ", pause);
        telemetry.addData("Power:", powerFL);
        telemetry.addData("Data Points [distance, power]: ", Arrays.toString(dataPoints));

        if (loops > 1000) {  //If loops is within testing limit
            pause = true;
            loops = 0;
        }
        else if (pause) {      //If test completed
            if (bumperL) {
                powerFL -= 0.1;
            }                       //Adjust motor powers for next test
            else if (bumperR) {
                powerFL += 0.1;
            }
            else if (buttonA) {     //Wait until button A is pressed to continue
                pause = false;
            }
        }
        else {
            count = loops % 10 == 0 ? loops / 10 : count;   //Add to count every 10 loops

            dataPoints[count][0] = distF;       //Set data points for distance and motor power while test is running
            dataPoints[count][1] = powerFL;

            loops++;        //Increase loops at end of testing loop
        }
    }
}