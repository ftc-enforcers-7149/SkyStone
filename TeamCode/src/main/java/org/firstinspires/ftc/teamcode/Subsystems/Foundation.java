package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

public class Foundation {
    private Servo lFound, rFound;

    /**
     * Constructor
     * @param lFound lFound
     * @param rFound rFound
     */
    public Foundation(Servo lFound, Servo rFound) {
        this.lFound = lFound;
        this.rFound = rFound;
    }

    /**
     * Hardware Map. Initializing servos
     */

    public void up() {
        rFound.setPosition(0.95);
        lFound.setPosition(0.95);
    }

    /**
     * Servos moving foundation grabbers
     */

     public void down() {
         rFound.setPosition(0);
         lFound.setPosition(0);
     }
    /**
     * Servos moving foundation grabbers
     */
}
