package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie;

public class PositionClass {

    public double positionX, positionY;

    public PositionClass() {
        positionX = 0;
        positionY = 0;
    }

    public PositionClass(double posX, double posY) {
        positionX = posX;
        positionY = posY;
    }


    public void manualUpdatePosition(double newPosX, double newPosY) {

        positionX = newPosX;
        positionY = newPosY;

    }

    public double getBotX() {
        return positionX;
    }

    public double getBotY(){
        return positionY;
    }

    public double[] getBotPosition() {

        double[] returnarray = new double[2];
        returnarray[0] = positionX;
        returnarray[1] = positionY;
        return returnarray;

    }



}
