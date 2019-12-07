package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;

//@Autonomous(name="Blue SkyStone ParkV2_1")
public class BlueSkyStoneParkV2_1 extends ParentInit {

    int step=0;

    String position="";

    public void init(){
       super.init();
    }

    public void init_loop() {
        super.init_loop();
    }

    public void start(){
        super.start();
    }

    public void loop(){
        switch(step){
            case 0:
                position = webcam.getQueuePos(telemetry);
                webcam.deactivate();
                break;
            case 1:
                driveTrain.driveStraight("forward",20,0.2);
                break;
            case 2:
                if(position.equals("right")){
                    driveTrain.driveRange(distanceR,66,"right");
                }
                else if(position.equals("left")){
                    driveTrain.driveRange(distanceR,104,"right");
                }
                else{
                    driveTrain.driveRange(distanceR,82,"right");
                }
                break;
            case 3:
                claw.down();
                claw.release();
                break;
            case 4:driveTrain.delay(500);
                break;
            case 5:
                driveTrain.driveStraight("forward", 12);
                break;//
            case 6:
                claw.grab();
                break;
            case 7:
                driveTrain.delay(500);
                claw.up();
                break;
            case 8:
               driveTrain.driveStraight("backward",6);
                break;
            case 9:
                driveTrain.simpleTurn(0,0.2);
                break;
            case 10:
                driveTrain.strafeToLine(color, "blue", "left");
                break;
            case 11:
                driveTrain.strafeSeconds(1000,"left");
                break;
            case 12:
                driveTrain.simpleTurn(0,0.2);
                break;
            case 13:
                claw.down();
                driveTrain.delay(750);
                claw.release();
                claw.up();
                break;
            case 14:
                driveTrain.strafeToLine(color,"blue","right");
            /*case 13:
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
            case 14:
                claw.down();
                claw.setState(true,false);
                break;
            case 15:driveTrain.delay(500);
                break;
            case 16:
                driveTrain.driveStraight("forward", 25);
                break;
            case 17:
                if(position.equals("right")){
                    claw.grabVertical();
                }
                else{
                    claw.grab();
                }
                break;*/
        }

        step++;
        telemetry.addData("position",position);
        telemetry.addData("range",distanceR.getDistance(DistanceUnit.CM));
        telemetry.addData("Step: ", step);
    }

    public void stop(){
    }
}
