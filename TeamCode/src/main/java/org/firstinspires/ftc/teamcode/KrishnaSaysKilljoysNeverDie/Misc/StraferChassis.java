package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

//@TeleOp(name = "Strafer Chassis")
public class StraferChassis extends OpMode {

    //Declaring motors
    DcMotor fLeft, fRight, bLeft, bRight;

    //IMU
    BNO055IMU imu;
    Orientation angles;


    //Testing
    double threshold = 0.25;//degree threshold of what counts as straight
    double initAngle, time;
    boolean state = false;//boolean that switches between initializing angle and detetcting current angle
    boolean isStraight = true;


    //Declaring drive-related vars
    float lDrive, rDrive;
    float lStrafe, rStrafe;
    double speed = 1;


    public void init() {

        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        /*fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);*/

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        telemetry.addAction(new Runnable() {
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

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void loop() {

        lDrive = gamepad1.left_stick_y;
        rDrive = -gamepad1.right_stick_y;
        lStrafe = gamepad1.left_trigger;
        rStrafe = gamepad1.right_trigger;



        if (lStrafe > 0) {
            fRight.setPower((-lDrive / 1.2) * speed);
            fLeft.setPower((lDrive / 1.2)  * speed);
            bLeft.setPower((-lDrive / 1.2)  * speed);
            bRight.setPower((lDrive / 1.2)  * speed);
        }
        else if (rStrafe > 0) {
            fRight.setPower(rDrive / 1.2);
            fLeft.setPower(-rDrive / 1.2);
            bLeft.setPower(rDrive / 1.2);
            bRight.setPower(-rDrive / 1.2);
        }
        else {
        fRight.setPower(rDrive);
        fLeft.setPower(lDrive);
        bLeft.setPower(lDrive);
        bRight.setPower(rDrive);
        }


        telemetry.addData("Am I Straight? ", isStraight);
        //current time
        time = System.currentTimeMillis();

        //if and else if are the different states. if is initializing, else if is comparing
        if (time % 500 == 0 && state) {
            initAngle = angles.firstAngle;
            state = false;
        } else if (time % 500 == 0 && !state) {
            //logic that sees if robot is drifting
            if (Math.abs(initAngle - angles.firstAngle) <= threshold) {
                isStraight = true;
            } else {
                isStraight = false;
            }
            state = true;
        }
    }




    public void stop() {

    }


    /**
     * a function that is needed to format the gyro angle
     * @param angleUnit
     * @param angle
     * @return
     */
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    /**
     * a function that is needed to format the gyro angle
     * @param degrees
     * @return
     */
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
