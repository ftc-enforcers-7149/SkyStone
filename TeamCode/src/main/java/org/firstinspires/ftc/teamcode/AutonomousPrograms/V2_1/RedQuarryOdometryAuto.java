package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2_1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV3;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Positions;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;
import org.firstinspires.ftc.teamcode.Subsystems.Range;

@Autonomous(name = "Red Quarry Odom Auto v2")
public class RedQuarryOdometryAuto extends ParentInit {

    DriveTrainV3 driveTrain;

    OdometryPosition.Direction direction;

    //Motors and imu
    DcMotor fRight, fLeft, bRight, bLeft;
    Gyroscope gyroscope;

    Range range;

    Positions position=Positions.RIGHT;

    int step = 0;

    //Positions for easily changing autonomous
    final double PRE_GRAB_Y = 22;   //Before robot drives into skystone
    final double POST_GRAB_Y = 34;  //Position to drive to to get skystone
    final double FOUNDATION_Y = 20;//Use 24 eventually, 20 is for testing purposes

    //These are the x values
    final double FIRST_LEFT_SKYSTONE = 19;
    final double FIRST_CENTER_SKYSTONE = 28;
    final double FIRST_RIGHT_SKYSTONE = 35;

    final double SECOND_LEFT_SKYSTONE = 1;
    final double SECOND_CENTER_SKYSTONE = 4.5;
    final double SECOND_RIGHT_SKYSTONE = 12;

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

        range = new Range(hardwareMap, "distanceL", "distanceR", "distanceC");

        direction = OdometryPosition.Direction.FORWARD;

        gyroscope = new Gyroscope(telemetry, hardwareMap);
        driveTrain = new DriveTrainV3(hardwareMap, telemetry, fLeft, fRight, bLeft, bRight, gyroscope);
    }

    public void init_loop() {
        super.init_loop();
    }

    public void start() {

        driveTrain.setX(31);
        driveTrain.setY(0);
        claw.down();
    }

    public void loop() {
        driveTrain.updateOdom(direction);
        telemetry.addData("Step: ", step);
        telemetry.addData("Position: ", "(" + driveTrain.getPosX() + ", " + driveTrain.getPosY() + ")");
        telemetry.addData("Heading: ", driveTrain.getHeading());
        telemetry.addData("position",position);
        telemetry.addLine();

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
                    if (driveTrain.driveToPoint(FIRST_RIGHT_SKYSTONE, PRE_GRAB_Y, 0.8, 0)) {
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.driveToPoint(FIRST_CENTER_SKYSTONE, PRE_GRAB_Y, 0.8, 0)) {
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(FIRST_LEFT_SKYSTONE, PRE_GRAB_Y, 0.8, 0)) {
                        step++;
                    }
                }
                break;
            case 2:
                //Move to put stone in claw
                if (position == Positions.RIGHT) {
                    if (driveTrain.driveToPoint(FIRST_RIGHT_SKYSTONE, POST_GRAB_Y, 0.3, 0)) {
                        claw.grab();
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.driveToPoint(FIRST_CENTER_SKYSTONE, POST_GRAB_Y, 0.3, 0)) {
                        claw.grab();
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(FIRST_LEFT_SKYSTONE, POST_GRAB_Y, 0.3, 0)) {
                        claw.grab();
                        step++;
                    }
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
                if (driveTrain.rotate(0)) {
                    direction = OdometryPosition.Direction.FORWARD;
                    step++;
                }
                break;
            case 6:
                //Drive to foundation side
                if (driveTrain.driveToPoint(FIRST_FOUNDATION_SIDE, FOUNDATION_Y, 0.8, 0)) {
                    claw.down();
                    claw.release();
                    direction = OdometryPosition.Direction.TURNING;
                    step++;
                }
                if (color.red()>90) {
                    driveTrain.setX(64.25);
                }
                break;
            case 7:
                //Self correction
                if (driveTrain.rotate(0)) {
                    claw.up();
                    direction = OdometryPosition.Direction.FORWARD;
                    step++;
                }
                break;
            case 8:
                //Drive back in preparation of going to skystone
                if (driveTrain.driveToPoint(PRE_FOUNDATION_X, FOUNDATION_Y, 0.8, 0)) {
                    claw.down();
                    direction = OdometryPosition.Direction.TURNING;
                    step++;
                }
                if (color.red()>90) {
                    driveTrain.setX(64.25);
                }
                break;
            case 9:
                //Self correction
                if (driveTrain.rotate(0)) {
                    direction = OdometryPosition.Direction.FORWARD;
                    step++;
                }
                break;
            case 10:
                //Move into position for next skystone
                if (position == Positions.RIGHT) {
                    if (driveTrain.driveToPoint(SECOND_RIGHT_SKYSTONE, PRE_GRAB_Y, 0.8, 0)) {
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.driveToPoint(SECOND_CENTER_SKYSTONE, PRE_GRAB_Y, 0.8, 0)) {
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(SECOND_LEFT_SKYSTONE,PRE_GRAB_Y, 0.8, 0)) {
                        claw.setState(false, true);
                        step++;
                    }
                    else if (gyroscope.getXAccel() == 0) {
                        claw.setState(false, true);
                        step++;
                    }
                }
                break;
            case 11:
                //Move to put skystone in claw
                if (position == Positions.RIGHT) {
                    if (driveTrain.driveToPoint(SECOND_RIGHT_SKYSTONE, POST_GRAB_Y, 0.3, 0)) {
                        claw.grab();
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.driveToPoint(SECOND_CENTER_SKYSTONE, POST_GRAB_Y, 0.3, 0)) {
                        claw.grab();
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(SECOND_LEFT_SKYSTONE, POST_GRAB_Y, 0.3, 0)) {
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
                if (driveTrain.rotate(0)) {
                    direction = OdometryPosition.Direction.FORWARD;
                    step++;
                }
                break;
            case 15:
                //Move to foundation side
                if (driveTrain.driveToPoint(SECOND_FOUNDATION_SIDE, FOUNDATION_Y, 0.8, 0)) {
                    claw.down();
                    claw.release();
                    direction = OdometryPosition.Direction.TURNING;
                    step++;
                }
                if (color.red()>85) {
                    driveTrain.setX(64.25);
                }
                break;
            case 16:
                if (driveTrain.rotate(0)) {
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
                if (driveTrain.driveToPoint(NAVIGATION_LINE, FOUNDATION_Y, 0.8, 0)) {
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
