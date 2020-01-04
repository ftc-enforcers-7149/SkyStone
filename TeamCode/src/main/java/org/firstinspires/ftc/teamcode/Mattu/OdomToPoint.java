package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV2;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;

@Autonomous(name = "OdomToPoint")
public class OdomToPoint extends OpMode {

    //Odometry calculations
    OdometryPosition oP;
    OdometryPosition.Direction direction = OdometryPosition.Direction.FORWARD;

    DriveTrainV2 driveTrain;

    //Motors and imu
    DcMotor fRight, fLeft, bRight, bLeft;
    Gyroscope gyroscope;

    double refX, refY;

    int step = 0;


    public void init() {

        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);


        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void start() {
        gyroscope = new Gyroscope(telemetry, hardwareMap);
        driveTrain = new DriveTrainV2(telemetry, fLeft, fRight, bLeft, bRight, gyroscope);
        oP = new OdometryPosition(hardwareMap, "encX", "encY", 0, 0, gyroscope, fLeft, fRight, bLeft, bRight);
        oP.reverseY();
    }

    public void loop() {
        oP.updatePosition(direction);
        telemetry.addData("Encoder Y: ", oP.positionY);
        telemetry.addData("Encoder X: ", oP.positionX);
        telemetry.addData("Raw Y: ", oP.getRawY());
        telemetry.addData("Raw X: ", oP.getRawX());
        telemetry.addData("Heading: ", oP.getHeading());

        switch (step) {
            case 0:
                //This is the right format for driving to a point
                //It will keep driving until at the point, then it will stop and move on
                if (oP.driveToPoint(0, 10, 0.3, telemetry)) {
                    //direction = OdometryPosition.Direction.TURNING;
                    step=2;
                }
                break;
            case 1:
                if (driveTrain.rotate(90)) {
                    direction = OdometryPosition.Direction.FORWARD;
                    step++;
                }
                break;
            case 2:
                if (oP.driveToPoint(10, 10, 0.3, telemetry)) {
                    step++;
                }
                break;
        }
    }

    /**
     * Sets refX and refY to the point that is xDist from x and yDist from y
     * @param xDist
     * @param yDist
     * @return
     *//*
    public double[] getRefPoint(double xDist, double yDist) {
        refX = oP.positionX+xDist;
        refY = oP.positionY+yDist;
        return new double[]{oP.positionX+xDist, oP.positionY+yDist};
    }

    *//**
     * Uses if statements to drive towards the point.
     * Returns true if it still needs to run.
     * Returns false if it is done
     * @param x     X coord
     * @param y     Y coord
     * @param lim   Speed limit
     * @return
     *//*
    public boolean driveToPoint(double x, double y, double lim) {
        //Gets the distance to the point
        double relativeX = x - oP.positionX;
        double relativeY = y - oP.positionY;

        //Uses distance to calculate power and angle
        double r = Math.hypot(relativeX, relativeY);
        double robotAngle = Math.atan2(relativeY, relativeX) - Math.toRadians(cvtDegrees(oP.getHeading())) + Math.PI / 4;

        telemetry.addData("Rel X: ", relativeX);
        telemetry.addData("Rel Y: ", relativeY);
        telemetry.addData("Robot Angle: ", robotAngle);

        //Calculates each motor power using trig
        double v1 = r * Math.cos(robotAngle);
        double v2 = r * Math.sin(robotAngle);
        double v3 = r * Math.sin(robotAngle);
        double v4 = r * Math.cos(robotAngle);

        //Getting the max value can assure that no motor will be set to a value above a certain point.
        double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));

        //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
        if (max > lim) {
            v1 /= max * (1 / lim);
            v2 /= max * (1 / lim);
            v3 /= max * (1 / lim);
            v4 /= max * (1 / lim);
        }

        //Sets power to motors
        fLeft.setPower(v1);
        fRight.setPower(v2);
        bLeft.setPower(v3);
        bRight.setPower(v4);

        //Returns true when the robot is close to the point
        if (Math.abs(relativeX) < 1 && Math.abs(relativeY) < 1) {
            motorStop();
            return true;
        }

        //Returns false if the robot is not at the point yet
        return false;
    }

    *//**
     * Converts a normal circle of degrees (0 at top) to a unit circle (0 at right and goes counter-clockwise)
     * @param heading
     * @return
     *//*
    private double cvtDegrees(double heading) {
        if (heading >= 0 && heading < 90) {
            return -heading + 90;
        }
        return -heading + 450;
    }

    *//**
     * Stops all motors without using actual OpMode stop method
     *//*
    public void motorStop() {
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }*/

    public void stop() {
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}


