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

@Autonomous(name = "Red Skystone Park V2_3 Claw")
public class RedSkyStoneParkV2_3Claw extends ParentInit {

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

        claw.down();
    }

    public void loop() {


        switch (step) {
            case 0:
                //Get skystone position
                position=webcam.getQueuePos(telemetry);
                claw.release();
                step++;
                break;
            case 1:
                //Align to grab stone
                if (position == Positions.RIGHT) {
                    if (driveTrain.strafeDrive(Directions.RIGHT,2)) {
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.strafeDrive(Directions.LEFT,4)) {
                        step++;
                    }
                }
                else {
                    if (driveTrain.strafeDrive(Directions.RIGHT,12)) {
                        step++;
                    }
                }
                break;
            case 2:
                //Move to put stone in claw
                if (driveTrain.driveStraight(Directions.FORWARD,26,0,0.3)) {
                    step=100;
                }
                break;
            case 3:
                if(driveTrain.delay(500)){
                    claw.halfUp();
                    step++;
                }
                break;
            case 4:
                //Drive back in preparation of going under skybridge
                if (driveTrain.driveToPoint(PRE_FOUNDATION_X, FOUNDATION_Y, 0.8, 0)) {
                    direction = OdometryPosition.Direction.TURNING;
                    step++;
                }
                break;
            case 5:
                //Self correction
                if (driveTrain.rotate(90)) {
                    direction = OdometryPosition.Direction.FORWARD;
                    step++;
                }
                break;
            case 6:
                //Drive to foundation side
                if (driveTrain.driveToPoint(FIRST_FOUNDATION_SIDE, FOUNDATION_Y, 0.8)) {
                    claw.down();
                    claw.release();
                    direction = OdometryPosition.Direction.TURNING;
                    step++;
                }
                /*if (color.red()>80) {
                    telemetry.addLine("read line");
                    driveTrain.setX(64.25);
                }*/
                break;
            case 7:
                //Self correction
                if (driveTrain.rotate(270)) {
                    claw.up();
                    direction = OdometryPosition.Direction.FORWARD;
                    step++;
                }
                break;
            case 8:
                //Drive back in preparation of going to skystone
                if (driveTrain.driveToPoint(PRE_FOUNDATION_X, FOUNDATION_Y, 0.8, 270)) {
                    step++;
                }
                /*if (color.red()>80) {
                    telemetry.addLine("read line");
                    driveTrain.setX(64.25);
                }*/
                break;
            case 9:
                //Move into position for next skystone
                if (position == Positions.RIGHT) {
                    if (driveTrain.driveToPoint(SECOND_RIGHT_SKYSTONE, PRE_GRAB_Y, 0.5, 270)) {
                        direction = OdometryPosition.Direction.TURNING;
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.driveToPoint(SECOND_CENTER_SKYSTONE, PRE_GRAB_Y, 0.5, 270)) {
                        direction = OdometryPosition.Direction.TURNING;
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(SECOND_LEFT_SKYSTONE,PRE_GRAB_Y, 0.5, 270)) {
                        claw.setState(false, true);
                        direction = OdometryPosition.Direction.TURNING;
                        step++;
                    }
                    else if (gyroscope.getXAccel() == 0) {
                        claw.setState(false, true);
                        direction = OdometryPosition.Direction.TURNING;
                        step++;
                    }
                }

                break;
            case 10:
                if (driveTrain.rotate(0)) {
                    direction = OdometryPosition.Direction.FORWARD;
                    claw.down();
                    step++;
                }
                break;
            case 11:
                //Move to put skystone in claw
                if (position == Positions.RIGHT) {
                    if (driveTrain.driveToPoint(SECOND_RIGHT_SKYSTONE, POST_GRAB_Y, 0.3)) {
                        claw.grab();
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.driveToPoint(SECOND_CENTER_SKYSTONE, POST_GRAB_Y, 0.3)) {
                        claw.grab();
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(SECOND_LEFT_SKYSTONE, POST_GRAB_Y, 0.3)) {
                        claw.grabVertical();
                        step++;
                    }
                }
                break;
            case 12:
                claw.halfUp();
                step++;
                break;
            case 13:
                //Drive back in preparation of going under skybridge
                if (driveTrain.driveToPoint(35, FOUNDATION_Y, 0.8, 0)) {
                    direction = OdometryPosition.Direction.TURNING;
                    step++;
                }
                break;
            case 14:
                if (driveTrain.rotate(90)) {
                    direction = OdometryPosition.Direction.FORWARD;
                    step++;
                }
                break;
            case 15:
                //Move to foundation side
                if (driveTrain.driveToPoint(SECOND_FOUNDATION_SIDE, FOUNDATION_Y, 0.6)) {
                    claw.down();
                    claw.release();
                    direction = OdometryPosition.Direction.TURNING;
                    step++;
                }
                /*if (color.red()>80) {
                    driveTrain.setX(64.25);
                }*/
                break;
            case 16:
                if (driveTrain.rotate(270)) {
                    direction = OdometryPosition.Direction.FORWARD;
                    step++;
                }
                break;
            case 17:
                claw.up();
                step++;
                break;
            case 18:
                //Navigate to skybridge line
                if (driveTrain.driveToPoint(NAVIGATION_LINE, FOUNDATION_Y, 0.8, 270)) {
                    step++;
                    requestOpModeStop();
                }
                break;
        }
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
