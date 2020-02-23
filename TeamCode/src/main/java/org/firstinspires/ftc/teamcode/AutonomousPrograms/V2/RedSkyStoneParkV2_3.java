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
                    if (driveTrain.driveStraight(Directions.FORWARD,4)) {
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.driveStraight(Directions.BACKWARD, 4)) {
                        offset = 4;
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveStraight(Directions.BACKWARD, 12)) {
                        offset = 12;
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
                if (driveTrain.driveStraight(Directions.FORWARD,55 + offset,0)) {
                    step++;
                }
                break;
            case 5:
                if (driveTrain.strafeDrive(Directions.LEFT,7,0.4)) {
                    step++;
                }
                break;
            case 6:
                if (driveTrain.strafeDrive(Directions.RIGHT,3,0.4)) {
                    step++;
                }
                break;
            case 7:
                if (driveTrain.driveStraight(Directions.BACKWARD,79 + offset,358)) {
                    step++;
                }
                break;
            case 8:
                /*if (driveTrain.rotate(0,0.1)){
                    step++;
                }*/
                step++;
                break;
            case 9:
                if (driveTrain.strafeDrive(Directions.LEFT,4,0.4)) {
                    step++;
                }
                break;
            case 10:
                if (driveTrain.strafeDrive(Directions.RIGHT,2,0.4)) {
                    step++;
                }
                break;
            case 11:
                if (driveTrain.driveStraight(Directions.FORWARD,86 + offset,2)) {
                    step++;
                }
                break;
            case 12:
                if (driveTrain.strafeDrive(Directions.LEFT,10,0.4)) {
                    foundation.lDown();
                    step++;
                }
                break;
            case 13:
                if(driveTrain.delay(1250)){
                    step++;
                }
                break;
            case 14:
                if (driveTrain.foundationTurn(90)){
                    step++;
                }
                break;
            case 15:
                if(driveTrain.strafeSeconds(Directions.LEFT,1500,0.7)){
                    step++;
                }
                break;
            case 16:
                if(driveTrain.driveStraight(Directions.FORWARD, 2)) {
                    foundation.lUp();
                    step++;
                }
                break;
            case 17:
                if(driveTrain.strafeDrive(Directions.RIGHT, 4)) {
                    claw.halfUp();
                    step++;
                }
                break;
            case 18:
                if(driveTrain.rotate(180, .5)) {
                    step++;
                }
                break;
            case 19:
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
