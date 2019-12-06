package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

//@Autonomous(name="Blue SkyStone ParkV2")
public class BlueSkyStoneParkV2 extends ParentInit {
    int step=0;

    String position="";

    public void init(){
       super.init();
    }
    public void start(){
        super.start();
    }
    public void loop(){
        switch(step){
            case 0:
                driveTrain.driveStraight("forward",20,0.2);
                break;
            case 1:
                //driveTrain.driveRange(distanceR,65,"right");
            case 2:
                position = "center";
                webcam.deactivate();
                break;
            case 3:
                if(position.equals("right")){
                    driveTrain.driveRange(distanceR,60,"right");
                }
                else if(position.equals("left")){
                    driveTrain.driveRange(distanceR,92,"right");
                }
                else{
                    driveTrain.driveRange(distanceR,75,"right");
                }
                break;
            case 4:
                claw.down();
                claw.release();
                break;
            case 5:driveTrain.delay(500);
                break;
            case 6:
                driveTrain.driveStraight("forward", 25);
                break;//
            case 7:
                claw.grab();
                break;
            case 8:
                driveTrain.delay(500);
                break;
            case 9:
                claw.up();
                break;
            case 10:
               driveTrain.driveStraight("backward",17);
                break;
            case 11:
                driveTrain.rotation(90);
                telemetry.addData("angle",driveTrain.getRawYaw());
                break;
            case 12:
                claw.down();
                break;
            case 13:
                driveTrain.driveToLine(color, "blue", "forward");
                break;
            case 14:
                driveTrain.driveStraight("forward",20);
                break;
            case 15:
                claw.release();
                claw.up();
                break;
            case 16:
                driveTrain.driveToLine(color, "blue", "backward");
                break;
            case 17:
                telemetry.addData("angle",driveTrain.getRawYaw());
                driveTrain.rotation(80);
                break;
            case 18:
                driveTrain.driveRange(distanceC,40,"center");
                break;
            case 19:
                driveTrain.simpleTurn(0,0.2);
                break;
            case 20:
                if(position.equals("right")){
                    driveTrain.driveRange(distanceR,10,"right");
                }
                else if(position.equals("left")){
                    driveTrain.driveRange(distanceR,30,"right");
                }
                else{
                    driveTrain.driveRange(distanceR,20,"right");
                }
                break;
            case 21:
                claw.down();
                claw.setState(true,false);
                break;
            case 22:driveTrain.delay(500);
                break;
            case 23:
                driveTrain.driveStraight("forward", 25);
                break;
            case 24:
                claw.grabVertical();
                break;

        }
        step++;
        telemetry.addData("position",position);
        telemetry.addData("range",distanceR.getDistance(DistanceUnit.CM));
        telemetry.addData("Step: ", step);
    }
    public void stop(){
        driveTrain.stop();
    }
}
