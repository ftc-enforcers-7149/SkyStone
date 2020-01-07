package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class FoundationV2 {
    private Servo fLFound, fRFound, bLFound, bRFound;

    /**
     *
     * @param fLFound fLFound
     * @param fRFound fRFounnd
     * @param bLFound bLFound
     * @param bRFound bRFound
     */
    public FoundationV2(Servo fLFound, Servo fRFound, Servo bLFound, Servo bRFound) {
        this.fLFound=fLFound;
        this.fRFound=fRFound;
        this.bRFound=bRFound;
        this.bLFound=bLFound;
    }

    /**
     * left foundation down
     */
    public void lDown() {
        fLFound.setPosition(0.79);  //0.46
        bLFound.setPosition(0.79);  //0.46
    }

    /**
     * left foundation up
     */
    public void lUp() {
        fLFound.setPosition(1);
        bLFound.setPosition(1);
    }

    /**
     * right foundation down
     */
    public void rDown() {
        fRFound.setPosition(0.75);  //0.4
        bRFound.setPosition(0.75);  //0.4
    }

    /**
     * right foundation up
     */
    public void rUp() {
        fRFound.setPosition(1);
        bRFound.setPosition(1);
    }

}
