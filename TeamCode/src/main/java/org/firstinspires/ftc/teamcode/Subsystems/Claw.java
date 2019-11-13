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
    public void up(){
        lArm.setPosition(0);
        rArm.setPosition(0);
    }

    /**
     * Makes claw go down
     */
    public void down(){
        lArm.setPosition(0.95);
        rArm.setPosition(0.81);
    }

    /**
     * Makes claw grab
     */
    public void grab(){
        rGrab.setPosition(0.44);
        lGrab.setPosition(0.45);
    }

    /**
     * Makes claw release
     */
    public void release(){
        rGrab.setPosition(1);
        lGrab.setPosition(1);
    }

    /**
     * Used to set if left and right of claw need to be at different states.
     * True for grabbing, false for releasing
     * @param left left claw
     * @param right right claw
     */
    public void setState(boolean left,boolean right){
        if(left){
            lGrab.setPosition(0.45);
        }
        else{
            lGrab.setPosition(1);
        }

        if(right){
            rGrab.setPosition(0.44);
        }
        else{
            rGrab.setPosition(1);
        }
    }

    /**
     * claw positions to grab skystone
     */
    public void skyStoneGrab(){

    }

}
