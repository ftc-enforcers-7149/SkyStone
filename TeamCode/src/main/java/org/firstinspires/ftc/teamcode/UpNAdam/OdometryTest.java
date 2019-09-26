package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SkyStonev1_1;
import java.lang.Math;

public class OdometryTest extends SkyStonev1_1 {
    int step=0;
    double x=0;
    double y=0;

    public void init() {
        super.init();
    }

    public void loop() {

        switch(step){
            case 0:distanceDrive(10);
                y=bRight.getCurrentPosition()/COUNTS_PER_INCH;
                break;
            case 1:Rotation(90);
                break;
            case 2:distanceDrive(10);
                x=bRight.getCurrentPosition()/COUNTS_PER_INCH;
                break;
            case 3:toOrigin(y,x);
                break;
        }
        step++;

    }

    public void stop() {
        super.stop();
    }

}
