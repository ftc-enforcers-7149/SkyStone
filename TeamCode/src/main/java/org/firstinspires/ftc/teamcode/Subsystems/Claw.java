package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Claw {
    private Servo lArm, rArm, lGrab, rGrab;

    /**
     * Main constructor
     * @param lArm lArm
     * @param rArm rArm
     * @param lGrab lGrab
     * @param rGrab rGrab
     */
    public Claw(Servo lArm, Servo rArm, Servo lGrab, Servo rGrab){
        this.lArm = lArm;
        this.rArm = rArm;
        this.lGrab = lGrab;
        this.rGrab = rGrab;
    }

    /**
     * Makes the claw go up
     */
    public void down(){
        lArm.setPosition(0);
        rArm.setPosition(0);
    }

    /**
     * raises arm half way
     */
    public void halfUp(){
        lArm.setPosition(0.47);
        rArm.setPosition(0.4);
    }

    /**
     * Makes claw go down
     */
    public void up(){
        lArm.setPosition(0.95);
        rArm.setPosition(0.81);
    }

    /**
     * Makes claw grab
     */
    public void grab(){
        rGrab.setPosition(.20);//.41
        lGrab.setPosition(0.28);//.45
    }

    /**
     * Makes claw release
     */
    public void release(){
        rGrab.setPosition(.13);//.6
        lGrab.setPosition(0.21);//1
    }

    /**
     * Used to set if left and right of claw need to be at different states.
     * True for grabbing, false for releasing
     * @param left left claw
     * @param right right claw
     */
    public void setState(boolean left,boolean right){
        if(left){//grab
            lGrab.setPosition(0.28);//.45
        }
        else{//release
            lGrab.setPosition(1);
        }

        if(right){//grab
            rGrab.setPosition(.20);//.41
        }
        else{//release
            rGrab.setPosition(.13);//.6
        }
    }

    /**
     * grabs stone vertically
     */
    public void grabVertical() {
        rGrab.setPosition(0.29);
        lGrab.setPosition(0.39);
    }

}
