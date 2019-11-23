package org.firstinspires.ftc.teamcode.Subsystems.DriveSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Headless {

    //DC Motors and velocity variables
    private DcMotor fLeft, fRight, bLeft, bRight;
    private double v1, v2, v3, v4; //In same order as motors

    //IMU variables
    private BNO055IMU imu;
    private Orientation angles;
    private double angle, offset;

    //Variables for inputs
    private double leftX, leftY, rightX;
    private boolean resetAngle, changeMode;

    //Power limit
    private double lim;

    //Telemetry object
    private Telemetry telemetry;

    public Headless(HardwareMap hardwareMap, Telemetry telemetry, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        //Hardware mapping the motors
        fLeft = fl;
        fRight = fr;
        bLeft = bl;
        bRight = br;

        //Reversing left motors
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        this.telemetry = telemetry;

        this.telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        });

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Initialize variables
        lim = 1;
        v1 = 0;
        v2 = 0;
        v3 = 0;
        v4 = 0;
    }

    public void drive(Gamepad gamepad1) {
        //Getting inputs
        leftY = gamepad1.left_stick_y;
        leftX = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;
        changeMode = gamepad1.a;

        //Specific inputs
        angle = angles.firstAngle;
        resetAngle = gamepad1.y;

        //Fixing inputs
        rightX = -rightX;

        //Only set brakes if no inputs are given from the joysticks
        //this makes it easier for the robot to move diagonally
        if (leftY == 0 && leftX == 0 && rightX == 0) {
            bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //Offset the angle if the imu is incorrect
        if (resetAngle) {
            offset = angle;
        }

        //r is used to scale the power of the motors depending on how much the joysticks are pushed
        //robotAngle is the directional angle (radians) that the robot wants to go in terms of itself.
        //45 degrees is adding in radians because that is the small angle on a right triangle.
        //v1 - v4 are the velocities for each motor. r is multiplied here.
        //Sine and cosine are applied to their respective diagonal's wheels.
        //rightX is added to and subtracted from their respective side's wheels.
        double r = Math.hypot(leftX, leftY);
        double robotAngle = Math.atan2(leftY, leftX) - Math.toRadians(cvtDegrees(angle - offset)) + Math.PI / 4;

        v1 = r * Math.sin(robotAngle) + rightX;
        v2 = r * Math.cos(robotAngle) - rightX;
        v3 = r * Math.cos(robotAngle) + rightX;
        v4 = r * Math.sin(robotAngle) - rightX;

        //Getting the max value can assure that no motor will be set to a value above a certain point.
        double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));

        //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
        if (max > lim) {
            v1 /= max * (1 / lim);
            v2 /= max * (1 / lim);
            v3 /= max * (1 / lim);
            v4 /= max * (1 / lim);
        }

        //Setting each velocity to its respective motor
        fLeft.setPower(v1);
        fRight.setPower(v2);
        bLeft.setPower(v3);
        bRight.setPower(v4);
    }

    //This converts degrees to work with sine and cosine.
    //The equations were made in Desmos by plotting certain points (input, output)
    //Equation 1: y = -x + 90
    //Equation 2: y = -x + 450
    private double cvtDegrees(double heading) {
        if (heading >= 0 && heading < 90) {
            return -heading + 90;
        }
        return -heading + 450;
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

}