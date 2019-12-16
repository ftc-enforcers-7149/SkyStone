package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

import java.util.Locale;

@Autonomous(name="auto class")
public class ClassTest extends OpMode {

    public DcMotor fLeft, fRight, bLeft, bRight;
    Servo lArm, rArm, lGrab, rGrab, lFound, rFound;
    int step=0;
    Gyroscope gyro;

    BNO055IMU imu;
    Orientation angles;
    public void init() {


    }
public void start(){
    gyro=new Gyroscope(telemetry,hardwareMap);

}
    public void loop() {
        telemetry.addData("degree",gyro.getRawYaw());

    }

    public void stop() {
    }

    /**
     * method needed for gyro
     * @param angleUnit
     * @param angle
     * @return
     */
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    /**
     * method needed for gyro
     * @param degrees
     * @return
     */
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
