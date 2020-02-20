package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV4;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Positions;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;
import org.firstinspires.ftc.teamcode.Subsystems.Range;

@Autonomous(name = "Red Skystone Park V2_3")
public class RedSkyStoneParkV2_3 extends ParentInit {

    DriveTrainV4 driveTrain;
    Gyroscope gyroscope;
    Range range;

    OdometryPosition.Direction direction;
    Positions position=Positions.RIGHT;

    //Motors and imu
    DcMotor fRight, fLeft, bRight, bLeft;

    int step = 0;

    //Positions for easily changing autonomous
    final double PRE_GRAB_Y = 22;   //Before robot drives into skystone
    final double POST_GRAB_Y = 34;  //Position to drive to to get skystone
    final double FOUNDATION_Y = 20;//Use 24 eventually, 20 is for testing purposes

    //These are the x values
    final double FIRST_LEFT_SKYSTONE = 19;
    final double FIRST_CENTER_SKYSTONE = 27;
    final double FIRST_RIGHT_SKYSTONE = 35;

    final double SECOND_LEFT_SKYSTONE = -13;
    final double SECOND_CENTER_SKYSTONE = -5;
    final double SECOND_RIGHT_SKYSTONE = 12;//3;

    final double PRE_FOUNDATION_X = 42; //Spot to go to before or after cross skybridge
    final double FIRST_FOUNDATION_SIDE = 108;   //Spot to go to in first skystone cycle
    final double SECOND_FOUNDATION_SIDE = 96;   //Spot to go to in second skystone cyce
    final double NAVIGATION_LINE = 63;  //X value for parking on line

    public void init() {

        super.init();

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

        direction = OdometryPosition.Direction.FORWARD;

        range = new Range(hardwareMap, "distanceL", "distanceR", "distanceC");
        gyroscope = new Gyroscope(telemetry, hardwareMap);
        driveTrain = new DriveTrainV4(hardwareMap, telemetry, fLeft, fRight, bLeft, bRight, gyroscope);
    } //

    public void init_loop() {
        super.init_loop();
    }

    public void start() {
    }

    public void loop() {
        telemetry.addData("delta: ",gyroscope.getDelta(90,gyroscope.getRawYaw()));
        telemetry.addData("heading: ",gyroscope.getYaw());
        telemetry.addData("step: ",step);

        switch (step) {
            case 0:
                //Get skystone position
                position= Positions.RIGHT;
                claw.release();
                step++;
                break;
            case 1:
                if (position == Positions.RIGHT) {
                    if (driveTrain.driveStraight(Directions.FORWARD,4)) {
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.driveToPoint(FIRST_CENTER_SKYSTONE, PRE_GRAB_Y, 0.5,0)) {
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(FIRST_LEFT_SKYSTONE, PRE_GRAB_Y, 0.5,0)) {
                        step++;
                    }
                }
                break;
            case 2:
                if (driveTrain.strafeDrive(Directions.LEFT,26)) {
                    step++;
                }
                break;
            case 3:
                if (driveTrain.strafeDrive(Directions.RIGHT,1)) {
                    step++;
                }
                break;
            case 4:
                if (driveTrain.driveStraight(Directions.FORWARD,73,0)) {
                    step++;
                }
                break;
            case 5:
                if (driveTrain.strafeDrive(Directions.LEFT,10,0.4)) {
                    step++;
                }
                break;
            case 6:
                if (driveTrain.strafeDrive(Directions.RIGHT,5,0.4)) {
                    step++;
                }
                break;
            case 7:
                if (driveTrain.driveStraight(Directions.BACKWARD,97,358)) {
                    step++;
                }
                break;
            case 8:
                if (driveTrain.rotate(0)){
                    step++;
                }
                break;
            case 9:
                if (driveTrain.strafeDrive(Directions.LEFT,4,0.4)) {
                    step++;
                }
                break;
            case 10:
                if (driveTrain.strafeDrive(Directions.RIGHT,4,0.4)) {
                    step++;
                }
                break;//

        }
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
