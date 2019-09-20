package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="claw test")
public class ClawTest extends OpMode {
    Servo clawL, clawR;
    boolean grab,realease,attack,mode1,mode2,toggle=false;
    float joystickR,joystickL;
    double positionR =0,positionL=0;
    public void init(){
        clawR=hardwareMap.servo.get("clawR");
        clawL=hardwareMap.servo.get("clawL");

        clawR.setDirection(Servo.Direction.REVERSE);
        clawL.setDirection(Servo.Direction.FORWARD);

        clawL.setPosition(1);
        clawR.setPosition(1);
    }
    public void loop(){
        grab=gamepad1.y;
        realease=gamepad1.a;
        attack=gamepad1.x;
        mode1=gamepad1.right_bumper;
        mode2=gamepad1.left_bumper;

        joystickR =gamepad1.right_stick_y;
        joystickL =gamepad1.left_stick_y;

        if(mode1){
            toggle=false;
        }
        else if(mode2){
            toggle=true;
        }

        if(joystickR <-0.1 && positionR >0){
            positionR -=0.001;
        }
        else if(joystickR >0.1 && positionR <1){
            positionR +=0.001;
        }

        if(joystickL <-0.1 && positionL >0){
            positionL -=0.001;
        }
        else if(joystickL >0.1 && positionL <1){
            positionL +=0.001;
        }


        if(!toggle) {
            if (grab) {
                clawL.setPosition(1);
                clawR.setPosition(1);
            } else if (realease) {
                clawL.setPosition(0.3);
                clawR.setPosition(0.3);
            } else if (attack){
                clawL.setPosition(0.7);
                clawR.setPosition(0.7);
            }
        }
        else if(toggle){
            clawL.setPosition(positionL);
            clawR.setPosition(positionR);
        }
    }
    public void stop(){

    }
}
