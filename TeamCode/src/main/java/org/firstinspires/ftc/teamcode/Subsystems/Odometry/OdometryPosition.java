package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc.Position;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

import java.util.Locale;


//TODO: CALCULATE COUNTS PER INCH
//TODO: write in better error handling

public class OdometryPosition extends Position {

    //Declaring motors
    DcMotor encoderY, encoderX;

    private Gyroscope gyro;


    //Used for encoders

    //used for encoders (y)
    private static final double     COUNTS_PER_MOTOR_REVY    = 400;  //1440 for 1 enc //512 for another(x) 400 for (y) //
    private static final double     WHEEL_DIAMETER_INCHESY  = 1.49606299d ;     // For figuring circumference
    public static final double     COUNTS_PER_INCHY        = COUNTS_PER_MOTOR_REVY /(WHEEL_DIAMETER_INCHESY * Math.PI);

    //used for encoders (x)
    private static final double     COUNTS_PER_MOTOR_REVX    = 400;  //1440 for 1 enc //512 for another(x) 400 for (y) //
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

        //Resets our encoders every time this is called. This is important because we
        //are adding to the position every time the method is called, and we don't
        //want distances to stack.

        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets our encoders to run again
        encoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Gets our heading and change in x and y odometers
        double heading = getHeading();
        double yDist = encoderY.getCurrentPosition();
        double xDist = encoderX.getCurrentPosition();

        //When turning, the odometers are not accurate, so they must be ignored
        if (dir != Direction.TURNING) {
            //Converts the robot's angle for use with sine and cosine
            //Then uses that as a modifier for how much an odometer will effect that axis

            //Apply the x odometer to the x and y axes
            positionY += getMotorDistInY(xDist) * Math.cos(Math.toRadians(gyro.cvtTrigAng(heading)));
            positionX += getMotorDistInX(xDist) * Math.sin(Math.toRadians(gyro.cvtTrigAng(heading)));

            //Apply the y odometer to the x and y axes
            positionY += getMotorDistInY(yDist) * Math.sin(Math.toRadians(gyro.cvtTrigAng(heading)));
            positionX += getMotorDistInX(yDist) * Math.cos(Math.toRadians(gyro.cvtTrigAng(heading)));
        }

        //Rounds the positions so you don't get numbers like 6.6278326e^-12678
        positionY = Math.ceil(positionY * 10000) / 10000;
        positionX = Math.ceil(positionX * 10000) / 10000;
    }

    /**
     * Converts a normal circle of degrees (0 at top) to a unit circle (0 at right and goes counter-clockwise)
     * @param heading
     * @return
     */
    private double cvtDegrees(double heading) {

        if (heading <0 ) {
            return 360 + heading;
        } else {
            return heading;
        }
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
