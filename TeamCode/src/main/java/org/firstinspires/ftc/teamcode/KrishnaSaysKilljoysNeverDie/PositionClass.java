package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie;


//Simple positioning class for basic positioning.
public class PositionClass {


    //Class vars
    public double positionX, positionY;


    //Constructors
    public PositionClass() {
        positionX = 0;
        positionY = 0;
    }

    public PositionClass(double posX, double posY) {
        positionX = posX;
        positionY = posY;
    }


    //Manually updates position
    public void manualUpdatePosition(double newPosX, double newPosY) {

        positionX = newPosX;
        positionY = newPosY;

    }


    //Returns x
    public double getBotX() {
        return positionX;
    }


    //Returns Y
    public double getBotY(){
        return positionY;
    }


    //Returns x and y as a double array
    public double[] getBotPosition() {

        double[] returnarray = new double[2];
        returnarray[0] = positionX;
        returnarray[1] = positionY;
        return returnarray;

    }



}
