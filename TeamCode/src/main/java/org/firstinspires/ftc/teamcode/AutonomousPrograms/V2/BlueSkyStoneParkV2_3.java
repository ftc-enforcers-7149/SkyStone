package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc.Position;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Positions;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;

@Autonomous(name = "Blue Skystone Park V2_3")
public class BlueSkyStoneParkV2_3 extends ParentInit {

    OdometryPosition.Direction direction;
    Positions position;

    int step = 0;
    double offset = 0;

    public void init() {
        super.init();
    direction = OdometryPosition.Direction.FORWARD;
} //

    public void init_loop() {
        super.init_loop();
        telemetry.addData("Position x: ", driveTrain.getPosX());
        telemetry.addData("Position y: ", driveTrain.getPosY());
    }

    public void start() {
    }

    public void loop() {
        telemetry.addData("position: ", position);
        telemetry.addData("delta: ",gyroscope.getDelta(90,gyroscope.getRawYaw()));
        telemetry.addData("heading: ",gyroscope.getYaw());
        telemetry.addData("step: ",step);
        telemetry.addData("Blue: ",color.blue());
        telemetry.addData("Blue: ",color.red());
        telemetry.addData("Position x: ", driveTrain.getPosX());
        telemetry.addData("Position y: ", driveTrain.getPosY());


        switch (step) {
            //Senses skystone
            case 0:
                //Get skystone position
                position = webcam.getQueuePos(telemetry);
                claw.up();
                claw.grab();
                step++;
                break;
            //Moves to align with block, and sets offset
            case 1:
                if (position == Positions.LEFT) {
                    if (driveTrain.driveStraight(Directions.BACKWARD,7)) {
                        offset=2;
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.driveStraight(Directions.BACKWARD, 1.25)) {
                        offset = 7;
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveStraight(Directions.FORWARD, 0.1)) {
                        offset = 18;
                        step++;
                    }
                }
                foundation.lHalf();
                break;
            //Strafes into and grabs block
            case 2:
                if (driveTrain.strafeDrive(Directions.LEFT,24, 0.6, 0)) {
                    foundation.lDown();
                    step++;
                }
                break;

            //Strafes more into block
            case 3:
                if (driveTrain.strafeDrive(Directions.LEFT,8, 0.6, 0)) {
                    step++;
                }
                break;
            //Drives forward to correct block grab
            case 4:
                if (driveTrain.driveStraight(Directions.BACKWARD,2,0,0.2)) {
                    foundation.lStick();
                    step++;
                }
                break;
            //Strafes away from block
            case 5:
                if (driveTrain.strafeDrive(Directions.RIGHT,11)) {
                    step++;
                }
                break;
            //Drives to foundation
            case 6:
                if (driveTrain.driveStraight(Directions.BACKWARD,60 + offset,0)) {
                    step++;
                }
                break;
            //Strafes into foundation and releases block
            case 7:
                if (driveTrain.strafeDrive(Directions.LEFT,7,0.4)) {
                    foundation.lDown();
                    step++;
                }
                break;
            //Waits and puts foundation grabber up
            case 8:
                if(driveTrain.delay(500)) {
                    foundation.lUp();
                    step++;
                }
                break;
            //Strafes away from block
            case 9:
                if (driveTrain.strafeDrive(Directions.RIGHT,6,0.4)) {
                    step++;
                }
                break;
            //Drives back to second block
            case 10:
                if(position==Positions.CENTER){
                    offset=9;
                }
                if (driveTrain.driveStraight(Directions.FORWARD,99 + offset,0)) {
                    foundation.lDown();
                    step++;
                }
                break;
            //Waits for a second
            case 11:
                if (driveTrain.delay(1000)) {
                    step++;
                }
            //Strafes to block
            case 12:
                if (driveTrain.strafeDrive(Directions.LEFT,6, 0.6, 0)) {
                    step++;
                }
                break;
            //If left, drives for 1.25 seconds. Else, sets into ready position
            case 13:
                /*if(position==Positions.LEFT) {
                    fLeft.setPower(-0.2);
                    fRight.setPower(-0.2);
                    bLeft.setPower(-0.2);
                    bRight.setPower(-0.2);
                    if (driveTrain.delay(1250)) {
                        fLeft.setPower(0);
                        fRight.setPower(0);
                        bLeft.setPower(0);
                        bRight.setPower(0);
                        step++;
                    }
                }
                else {
                    step++;
                }*/
                step++;
                break;

            case 14:
                if (driveTrain.driveStraight(Directions.BACKWARD,2,0,0.2)) {
                    foundation.lStick();
                    step++;
                }
                break;
            //Strafes away from block
            case 15:
                if (driveTrain.strafeDrive(Directions.RIGHT,13,0.4)) {
                    step++;
                }
                break;
            //Drives forward to foundation
            case 16:
                if(position==Positions.CENTER){
                    offset=9;
                }
                else if(position==Positions.RIGHT){
                    offset=13;
                }
                if (driveTrain.driveStraight(Directions.BACKWARD,75 + offset,2,0.8)) {
                    step++;
                }//
                break;
            //Strafes into foundation and grabs it
            case 17:
                if (driveTrain.strafeDrive(Directions.LEFT,16,0.4)) {
                    foundation.lDown();
                    step++;
                }
                break;
            //Waits for 1.25 seconds
            case 18:
                if(driveTrain.delay(750)){
                    step++;
                }
                break;
            //Turns foundation
            case 19:
                if (driveTrain.foundationTurn(280)){
                    step++;
                }
                break;
            case 20:
                if (driveTrain.rotate(270,0.7)){
                    step++;
                }
                break;
            //Strafes against wall
            case 21:
                if(driveTrain.strafeSecondsBlue(Directions.LEFT,2500,1)){
                    step++;
                }
                break;
            //Drives foundation in and releases foundation
            case 22:
                if(position==Positions.LEFT){
                    offset=4;
                }
                if(driveTrain.foundationDrive(Directions.BACKWARD,Positions.LEFT,2+offset,0.8)) {
                    foundation.lUp();
                    step++;
                }
                break;
            //Strafes away from foundation
            case 23:
                    if(driveTrain.strafeDrive(Directions.RIGHT, 4)) {
                        claw.release();
                        claw.halfUp();
                        step++;
                    }

                break;
            //Turns to line
            case 24:
                if(driveTrain.rotate(0)) {
                    step++;
                }
                break;
            //Drives to line
            case 25:
                if (driveTrain.driveToLine(color, Directions.FORWARD)) {
                    step++;
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
