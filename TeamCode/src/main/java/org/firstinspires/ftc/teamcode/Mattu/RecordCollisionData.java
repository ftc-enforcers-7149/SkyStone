package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

/**
 * This is a concept file for code to be implemented in CollisionAvoidance/CollisionData
 */

public class RecordCollisionData extends OpMode {

    //Array used to hold power and distance values for telemetry
    public double[] dataPoints;

    //Power set to the motors (in this case front left)
    public double power;

    //Distance read from front facing sensor
    public double distF;

    //Time that test starts
    public double startTime;

    //Gamepad inputs for changing tests
    public boolean buttonA, bumperL, bumperR;

    //loops is used to count the amount of loops the program has run through while testing
    //count is used to set the next index in datapoints every some amount of loops
    public int count;

    //Used to determine when test has ended
    public boolean pause;

    public void init() {
        //double array that can hold 100 x 2 values
        dataPoints = new double[20];

        //Initialize variables
        pause = false;
        count = 0;
    }

    public void start() {
        startTime = System.currentTimeMillis();
    }

    public void loop() {
        //General telemetry
        telemetry.addData("Done? ", pause);
        telemetry.addData("Power:", power);
        telemetry.addData("Data Points (distance per second): ", Arrays.toString(dataPoints));

        if (count > 10) {  //If seconds are within testing limit
            pause = true;
            count = 0;
        }
        else if (pause) {      //If test completed
            if (bumperL) {
                power -= 0.1;
            }                       //Adjust motor powers for next test
            else if (bumperR) {
                power += 0.1;
            }
            else if (buttonA) {     //Wait until button A is pressed to continue
                pause = false;
                startTime = System.currentTimeMillis(); //Reset startTime
            }
        }
        else {
            count = (int) Math.floor((System.currentTimeMillis() - startTime) / 1000);   //Set count to amount of seconds since startTime

            dataPoints[count] = distF;       //Set data point for distance at some amount of seconds
        }
    }
}