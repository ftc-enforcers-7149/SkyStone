package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc.Position;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

import java.util.Locale;


//TODO: CALCULATE COUNTS PER INCH
//TODO: write in better error handling

public class OdometryPosition extends Position {

    //Declaring motors
    private DcMotor encoderY, encoderX;
    private DcMotor fLeft, fRight, bLeft, bRight;

    private Gyroscope gyro;

    //Stored position for turning
    private double storedX, storedY;

    //Last direction value
    private boolean last_dir;

    //Used for encoders

    //used for encoders (y)
    private static final double     COUNTS_PER_MOTOR_REVY    = 1440;  //1440 for 1 enc //512 for another(x) 400 for (y) //
    private static final double     WHEEL_DIAMETER_INCHESY  = 1.49606299d ;     // For figuring circumference
    public static final double     COUNTS_PER_INCHY        = COUNTS_PER_MOTOR_REVY /(WHEEL_DIAMETER_INCHESY * Math.PI);

    //used for encoders (x)
    private static final double     COUNTS_PER_MOTOR_REVX    = 1440;  //1440 for 1 enc //512 for another(x) 400 for (y) //
    private static final double     WHEEL_DIAMETER_INCHESX  = 1.49606299d ;     // For figuring circumference
    public static final double     COUNTS_PER_INCHX      = COUNTS_PER_MOTOR_REVX /(WHEEL_DIAMETER_INCHESX * Math.PI);




    //Direction enum
    public enum Direction {FORWARD, BACKWARD, TURNING}






    //Constructor
    public OdometryPosition() {
        positionX = 0;
        positionY = 0;
    }

    public OdometryPosition(HardwareMap hardwareMap, String encX, String encY, double posX, double posY, Gyroscope gyro) {

        encoderX = hardwareMap.dcMotor.get(encX);
        encoderY = hardwareMap.dcMotor.get(encY);

        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets our encoders to run again
        encoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Setting pos
        positionX = posX;
        positionY = posY;

        this.gyro = gyro;

    }

    public OdometryPosition(HardwareMap hardwareMap, String encX, String encY, double posX, double posY, Gyroscope gyro, DcMotor fLeft, DcMotor fRight, DcMotor bLeft, DcMotor bRight) {

        encoderX = hardwareMap.dcMotor.get(encX);
        encoderY = hardwareMap.dcMotor.get(encY);

        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets our encoders to run again
        encoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.fLeft = fLeft;
        this.fRight = fRight;
        this.bLeft = bLeft;
        this.bRight = bRight;

        //Setting pos
        positionX = posX;
        positionY = posY;

        this.gyro = gyro;

    }



    //Simple return methods that return data
    public double getPositionX() {
        return positionX;
    }

    public double getPositionY() {
        return positionY;
    }

    public double getRawX() {
        return encoderX.getCurrentPosition();
    }

    public double getRawY() {
        return encoderY.getCurrentPosition();
    }

    public double[] getBotPosition() {
        return super.getBotPosition();
    }

    public double getHeading() {
        return gyro.getRawYaw();
    }

    //Returns the motor distance in inches for Y
    private double getMotorDistInY(double input) {
        return input/COUNTS_PER_INCHY;
    }

    //Returns the motor distance in inches for Y
    private double getMotorDistInX(double input) {
        return input/COUNTS_PER_INCHX;
    }


    /**
     * Updates the overall position of the robot based on the change in x odometer and y odometer
     * @param dir
     */
    public void updatePosition(Direction dir) {

        //Gets our heading and change in x and y odometers
        double heading = getHeading();
        double yDisp = encoderY.getCurrentPosition();
        double xDisp = encoderX.getCurrentPosition();

        //When turning, the odometers are not accurate, so they must be ignored
        if (dir != Direction.TURNING) {
            last_dir = false;
            //Converts the robot's angle for use with sine and cosine
            //Then uses that as a modifier for how much an odometer will effect that axis

            //Apply the x odometer to the x and y axes
            positionX = ((xDisp/COUNTS_PER_INCHX) * Math.sin(Math.toRadians(gyro.cvtTrigAng(heading)))) + ((yDisp/COUNTS_PER_INCHY)* Math.cos(Math.toRadians(gyro.cvtTrigAng(heading)))) + storedX;
            positionY = ((yDisp/COUNTS_PER_INCHY) * Math.sin(Math.toRadians(gyro.cvtTrigAng(heading)))) + ((xDisp/COUNTS_PER_INCHX) * Math.cos(Math.toRadians(gyro.cvtTrigAng(heading)))) + storedY;

            //Rounds the positions so you don't get numbers like 6.6278326e^-12678
            /*positionY = Math.ceil(positionY * 1000000) / 1000000;
            positionX = Math.ceil(positionX * 1000000) / 1000000;*/
        }
        else {
            if (!last_dir) {
                last_dir = true;
                encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //Sets our encoders to run again
                encoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                storedX = positionX;
                storedY = positionY;
            }
        }
    }

    /**
     * Uses if statements to drive towards the point.
     * Returns true if it still needs to run.
     * Returns false if it is done
     * @param x     X coord
     * @param y     Y coord
     * @param telemetry Telemetry object
     * @return
     */
    public boolean driveToPoint(double x, double y, double lim, Telemetry telemetry) {
        //Gets the distance to the point
        double relativeX = x - positionX;
        double relativeY = y - positionY;

        //Uses distance to calculate power and angle
        double r = Math.hypot(relativeX, relativeY);
        double robotAngle = Math.atan2(relativeY, relativeX) - Math.toRadians(cvtDegrees(getHeading())) + Math.PI / 4;

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

    /**
     * Converts a normal circle of degrees (0 at top) to a unit circle (0 at right and goes counter-clockwise)
     * @param heading   Angle in degrees to be converted
     * @return Converted angle in degrees
     */
    public double cvtDegrees(double heading) {
        if (heading >= 0 && heading < 90) {
            return -heading + 90;
        }
        return -heading + 450;
    }

    /**
     * Stops all motors without using actual OpMode stop method
     */
    public void motorStop() {
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }

    /**
     * method needed for gyro
     * @param angleUnit
     * @param angle
     * @return
     */
    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    /**
     * method needed for gyro
     * @param degrees
     * @return
     */
    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void reverseX() {
        encoderX.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void reverseY() {
        encoderY.setDirection(DcMotorSimple.Direction.REVERSE);
    }

}
