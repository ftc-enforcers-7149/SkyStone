package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

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
        lArm.setPosition(0.36);
        rArm.setPosition(0.45);  //0
    }

    /**
     * raises arm half way
     */
    public void halfUp(){
        lArm.setPosition(0.66);//0.47
        rArm.setPosition(0.73);//0.4
    }

    /**
     * Makes claw go down
     */
    public void up(){
        lArm.setPosition(0.95);
        rArm.setPosition(1);
    }

    /**
     * Makes claw grab
     */
    public void grab(){
        rGrab.setPosition(0.23);//.26
        lGrab.setPosition(0.22);//0.28
    }

    /**
     * Makes claw release
     */
    public void release(){
        rGrab.setPosition(0.15);//.17
        lGrab.setPosition(0.14);//0.2
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
            lGrab.setPosition(0.20);
        }

        if(right){//grab
            rGrab.setPosition(0.26);//.41
        }
        else{//release
            rGrab.setPosition(0.17);//.6
        }
    }

    /**
     * grabs stone vertically
     */
    public void grabVertical() {
        rGrab.setPosition(0.36);
        lGrab.setPosition(0.36);//.46
    }

}
