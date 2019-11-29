package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;

@Autonomous(name="Blue SkyStone ParkV2_1")
public class BlueSkyStoneParkV2_1 extends ParentInit {
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
                driveTrain.driveRange(distanceR,65,"right");
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
                //claw.up();
                break;
            case 10:
               driveTrain.driveStraight("backward",17);
                break;
            case 11:
                driveTrain.strafeToLine(color, "blue", "left");
                break;
            case 12:
                driveTrain.strafeSeconds(2000,"left");
                break;
            case 13:
                claw.release();
                claw.up();
                break;
            case 14://
                driveTrain.strafeToLine(color,"blue","right");
            case 15:
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
            case 16:
                claw.down();
                claw.setState(true,false);
                break;
            case 17:driveTrain.delay(500);
                break;
            case 18:
                driveTrain.driveStraight("forward", 25);
                break;
            case 19:
                if(position.equals("right")){
                    claw.grabVertical();
                }
                else{
                    claw.grab();
                }
                break;


        }
        step++;
        telemetry.addData("position",position);
        telemetry.addData("range",distanceR.getDistance(DistanceUnit.CM));
        telemetry.addData("Step: ", step);
    }
    public void stop(){

    }
}
