package org.firstinspires.ftc.teamcode.Mattu;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

/**
 * This is a concept file for code to be implemented in CollisionAvoidance/CollisionData
 */

@Autonomous(name = "DeltaDistance")
public class RecordCollisionData extends OpMode {

    //DC Motors
    DcMotor fLeft, fRight, bRight, bLeft; //Left and right motors

    //Distance Sensors
    DistanceSensor front;

    //Array used to hold power and distance values for telemetry
    public double[] dataPoints;

    //Power set to the motors (in this case front left)
    public double power;

    //Distance read from front facing sensor
    public double distF;

    //Time that test starts
    public double startTime;

    //Gamepad inputs for changing tests
    public boolean endTest, startTest, decreasePower, increasePower;
    public double joystickL, joystickR;

    //loops is used to count the amount of loops the program has run through while testing
    //count is used to set the next index in datapoints every some amount of loops
    public int count;

    //Used to determine when test has ended
    public boolean pause;

    public void init() {
        fLeft= hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bRight = hardwareMap.dcMotor.get("bRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");

        front = hardwareMap.get(DistanceSensor.class, "front");

        //double array that can hold 100 x 2 values
        dataPoints = new double[20];

        //Initialize variables
        power = 0.1;
        pause = false;
        count = 0;
    }

    public void start() {
        startTime = System.currentTimeMillis();
    }

    public void loop() {
        endTest = gamepad1.x;
        startTest = gamepad1.a;
        increasePower = gamepad1.right_bumper;
        decreasePower = gamepad1.left_bumper;
        joystickL = gamepad1.left_stick_y;
        joystickR = gamepad1.right_stick_y;

        distF = front.getDistance(DistanceUnit.CM);

        //General telemetry
        telemetry.addData("Done? ", pause);
        telemetry.addData("Power: ", power);
        telemetry.addData("Distance: ", distF);
        telemetry.addData("Data Points (distance per second): ", Arrays.toString(dataPoints));

        if (joystickL > 0.5 & joystickR > 0.5) {
            fLeft.setPower(power);
            fRight.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(power);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
        }

        if (count > dataPoints.length || endTest) {  //If seconds are within testing limit
            pause = true;
            count = 0;
            power = 0;

            double total = 0;

            for (int data = 0; data < dataPoints.length; data+=2) {
                total += dataPoints[data];
            }
            Log.i("Average change per sec", Double.toString(total / (dataPoints.length / 2)));
            Log.i("Distance per 0.5 secs", Arrays.toString(dataPoints));    //Log array data for safe-keeping
        }
        else if (pause) {      //If test completed
            if (decreasePower) {
                power -= 0.1;
            }                       //Adjust motor powers for next test
            else if (increasePower) {
                power += 0.1;
            }
            else if (startTest) {     //Wait until button A is pressed to continue
                pause = false;
                startTime = System.currentTimeMillis(); //Reset startTime
                dataPoints = new double[dataPoints.length];
            }
        }
        else {
            count = (int) Math.round((System.currentTimeMillis() - startTime) / 500);   //Set count to amount of seconds since startTime

            dataPoints[count] = distF;       //Set data point for distance at some amount of seconds
        }
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}