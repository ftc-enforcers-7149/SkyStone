package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Webcam;
@Autonomous(name="VuForia Test")
public class VuForiaTest extends OpMode {
    int step=0;
    public Webcam webcam;
    String position="";
    public void init(){
        webcam=new Webcam(hardwareMap);
    }
    public void start(){

    }
    public void loop(){
        switch(step){
            case 0:position=webcam.getQueuePos(telemetry);
            break;
        }
        telemetry.addData("position",position);
    }
    public void stop(){

    }
}
