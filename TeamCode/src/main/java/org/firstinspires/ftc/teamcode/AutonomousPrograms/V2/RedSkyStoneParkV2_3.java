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

    OdometryPosition.Direction direction;
    Positions position;

    int step = 0;
    double offset = 0;

    public void init() {
        super.init();
        direction = OdometryPosition.Direction.FORWARD;
    } //

    public void init_loop() { super.init_loop(); }

    public void start() {
    }

    public void loop() {
        telemetry.addData("position: ", position);
        telemetry.addData("delta: ",gyroscope.getDelta(90,gyroscope.getRawYaw()));
        telemetry.addData("heading: ",gyroscope.getYaw());
        telemetry.addData("step: ",step);

        switch (step) {
            case 0:
                //Get skystone position
                position = webcam.getQueuePos(telemetry);
                claw.release();
                step++;
                break;
            case 1:
                if (position == Positions.RIGHT) {
                    if (driveTrain.driveStraight(Directions.FORWARD,6)) {
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.driveStraight(Directions.FORWARD, 0,0)) {
                        offset = 9;
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveStraight(Directions.BACKWARD, 7)) {
                        offset = 16;
                        step++;
                    }
                }
                foundation.lHalf();
                break;
            case 2:
                if (driveTrain.strafeDrive(Directions.LEFT,24, 0.6, 0)) {
                    foundation.lDown();
                    step++;
                }
                break;

            case 3:
                if (driveTrain.strafeDrive(Directions.LEFT,8, 0.6, 0)) {
                    step++;
                }
                break;
            case 4:
                if (driveTrain.driveStraight(Directions.FORWARD,1,0,0.2)) {
                    foundation.lHalf();
                    step++;
                }
                break;
            case 5:
                if (driveTrain.strafeDrive(Directions.RIGHT,8)) {
                    step++;
                }
                break;
            case 6:
                if (driveTrain.driveStraight(Directions.FORWARD,55 + offset,0)) {
                    step++;
                }
                break;
            case 7:
                if (driveTrain.strafeDrive(Directions.LEFT,9,0.4)) {
                    step++;
                }
                break;
            case 8:
                if (driveTrain.strafeDrive(Directions.RIGHT,5,0.4)) {
                    step++;
                }
                break;
            case 9:
                if (driveTrain.driveStraight(Directions.BACKWARD,79 + offset,358)) {
                    step++;
                }
                break;
            case 10:
                /*if (driveTrain.rotate(0,0.1)){
                    step++;
                }*/
                step++;
                break;
            case 11:
                if (driveTrain.strafeDrive(Directions.LEFT,6,0.4)) {
                    step++;
                }
                break;
            case 12:
                if (driveTrain.strafeDrive(Directions.RIGHT,4,0.4)) {
                    step++;
                }
                break;
            case 13:
                if (driveTrain.driveStraight(Directions.FORWARD,86 + offset,2)) {
                    step++;
                }
                break;
            case 14:
                if (driveTrain.strafeDrive(Directions.LEFT,10,0.4)) {
                    foundation.lDown();
                    step++;
                }
                break;
            case 15:
                if(driveTrain.delay(1250)){
                    step++;
                }
                break;
            case 16:
                if (driveTrain.foundationTurn(90)){
                    step++;
                }
                break;
            case 17:
                if(driveTrain.strafeSeconds(Directions.LEFT,1500,0.7)){
                    step++;
                }
                break;
            case 18:
                if(driveTrain.driveStraight(Directions.FORWARD, 4)) {
                    foundation.lUp();
                    step++;
                }
                break;
            case 19:
                if(driveTrain.strafeDrive(Directions.RIGHT, 4)) {
                    claw.halfUp();
                    step++;
                }
                break;
            case 20:
                if(driveTrain.rotate(180)) {
                    step++;
                }
                break;
            case 21:
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
