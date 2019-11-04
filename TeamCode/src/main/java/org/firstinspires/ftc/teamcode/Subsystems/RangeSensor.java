package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RangeSensor {
    private DistanceSensor distanceL,distanceC,distanceR;

    /**
     * Main constructor
     * @param distanceL
     * @param distanceC
     * @param distanceR
     */
    public RangeSensor(DistanceSensor distanceL, DistanceSensor distanceC, DistanceSensor distanceR){
        this.distanceL=distanceL;
        this.distanceC=distanceC;
        this.distanceR=distanceR;
    }

    /**
     * returns left distance(cm)
     * @return
     */
    public double getLDistance(){
        return distanceL.getDistance(DistanceUnit.CM);
    }
    /**
     * returns center distance(cm)
     * @return
     */
    public double getCDistance(){
        return distanceC.getDistance(DistanceUnit.CM);
    }
    /**
     * returns right distance(cm)
     * @return
     */
    public double getRDistance(){
        return distanceR.getDistance(DistanceUnit.CM);
    }
}
