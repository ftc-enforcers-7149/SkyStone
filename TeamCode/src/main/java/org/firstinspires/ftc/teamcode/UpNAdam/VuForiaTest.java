package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Enums.Positions;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;
//@Autonomous(name="VuForia Test")
public class VuForiaTest extends OpMode {
    int step=0;
    public Webcam webcam;
    Positions position;
    public void init(){
        webcam=new Webcam(hardwareMap);
    }

    public void init_loop() {

        switch(step){
            case 0:position=webcam.getQueuePos(telemetry);
                break;
        }
        telemetry.addData("position",position);
        telemetry.addData("skystone? ",webcam.posDriveL(telemetry));
    }

    public void loop(){

    }
    public void stop(){

    }

}
