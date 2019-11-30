package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;

@Autonomous(name="Red SkyStone ParkV2_1")
public class RedSkyStoneParkV2_1 extends ParentInit {
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
                if(position.equals("left")){
                    driveTrain.driveRange(distanceL,60,"left");
                }
                else if(position.equals("right")){
                    driveTrain.driveRange(distanceL,92,"left");
                }
                else{
                    driveTrain.driveRange(distanceL,75,"left");
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
                driveTrain.strafeToLine(color, "red", "right");
                break;
            case 12:
                driveTrain.strafeSeconds(2000,"right");
                break;
            case 13:
                claw.release();
                claw.up();
                break;
            case 14://
                driveTrain.strafeToLine(color,"red","left");
            case 15:
                if(position.equals("left")){
                    driveTrain.driveRange(distanceL,10,"left");
                }
                else if(position.equals("right")){
                    driveTrain.driveRange(distanceL,30,"left");
                }
                else{
                    driveTrain.driveRange(distanceL,20,"left");
                }//
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
