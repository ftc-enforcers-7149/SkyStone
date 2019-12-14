package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc.Position;

import java.util.Locale;


//TODO: CALCULATE COUNTS PER INCH
//TODO: write in better error handling

public class OdometryPosition extends Position {

    //Declaring motors
    DcMotor encoderY, encoderX;

    BNO055IMU imu;
    private Orientation angles;



    //Class vars
    public double positionX, positionY;


    //Used for encoders

    //used for encoders
    private static final double     COUNTS_PER_MOTOR_REV    = 360;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    private static final double     WHEEL_DIAMETER_INCHES   = 1.49606299d ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV /(WHEEL_DIAMETER_INCHES * Math.PI);




    //Direction enum
    public enum Direction {FORWARD, BACKWARD, TURNING}









    //Constructor
    public OdometryPosition() {
        positionX = 0;
        positionY = 0;
    }

    public OdometryPosition(HardwareMap hardwareMap, String encX, String encY, String imumap, double posX, double posY) {

        imu = hardwareMap.get(BNO055IMU.class, imumap);
        encoderX = hardwareMap.dcMotor.get(encX);
        encoderY = hardwareMap.dcMotor.get(encY);

        //Setting pos
        positionX = posX;
        positionY = posY;


        //Set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new org.firstinspires.ftc.robotcore.external.navigation.Position(), new Velocity(), 1000);

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
        return cvtDegrees(angles.firstAngle);
    }

    public void manualUpdatePosition(double newPosX, double newPosY) {
        manualUpdatePosition(newPosX, newPosY);
    }

    //Returns the motor distance in inches
    private double getMotorDistIn(double input) {
        return input/COUNTS_PER_INCH;
    }













    //Method that updates position (the big one!)
    public void updatePosition(Direction dir) {

        //Resets our encoders every time this is called. This is important because we
        //are adding to the position every time the method is called, and we don't
        //want distances to stack.

        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets our encoders to run again
        encoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Gets our heading
        double heading = getHeading();
        double yPos = encoderY.getCurrentPosition();
        double xPos = encoderX.getCurrentPosition();

        //Calculates whether the robot is facing forwards or backwards. Note that all calculations are based on
        //The assumption we are facing straight out from the wall
        if (dir != Direction.TURNING) {
            if (heading % 180 == 0) {

                if (heading == 0) {
                        positionX += getMotorDistIn(xPos);
                } else {
                        positionX -= getMotorDistIn(xPos);
                }

            }
            //This time, we calculate if we are facing up or not. If we are not forwards/backwards or up/down, we move on.
            else if ((heading - 90) % 90 == 0) {
                if (heading == 90) {
                        positionY -= getMotorDistIn(yPos);

                } else {
                    if (dir == Direction.FORWARD) {
                        positionY += getMotorDistIn(yPos);
                    } else {
                        positionY -= getMotorDistIn(yPos);
                    }
                }
            }
            //Uses some reeeeeeeeally (not) complicated trig to calculate the distance.
            else {
                if (heading > 0 && heading < 90) {
                    positionY -= getMotorDistIn(xPos) * Math.sin(360 - heading);
                    positionX += getMotorDistIn(xPos) * Math.cos(360 - heading);
                } else if (heading > 90 && heading < 180) {
                    positionY -= getMotorDistIn(xPos) * Math.sin(360 - heading);
                    positionX -= getMotorDistIn(xPos) * Math.cos(360 - heading);
                } else if (heading > 180 && heading < 270) {
                    positionY += getMotorDistIn(xPos) * Math.sin(360 - heading);
                    positionX -= getMotorDistIn(xPos) * Math.cos(360 - heading);
                } else if (heading > 270 && heading < 360) {
                    positionY += getMotorDistIn(xPos) * Math.sin(360 - heading);
                    positionX += getMotorDistIn(xPos) * Math.cos(360 - heading);
                }
            }

        }
    }




    //Converts degrees
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
