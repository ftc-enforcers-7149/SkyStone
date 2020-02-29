package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CustomEncoder {

    //Encoder sensor
    private DigitalChannel encoderA, encoderB;

    //Thread object
    private Thread thread;

    //Specific class calculations
    private boolean channelA = false, channelB = false;
    private boolean lastChannelA = false;

    private int counts = 0;

    //Directions
    public enum Direction {
        FORWARD,
        REVERSE
    }
    private Direction dir = Direction.FORWARD;

    //Constructor
    public CustomEncoder(HardwareMap hardwareMap, String nameA, String nameB) {
        encoderA = hardwareMap.digitalChannel.get(nameA);
        encoderB = hardwareMap.digitalChannel.get(nameB);

        encoderA.setMode(DigitalChannel.Mode.INPUT);
        encoderB.setMode(DigitalChannel.Mode.INPUT);

        thread = new EncoderThread();
    }

    //Get encoder counts
    public int getCounts() {
        if (dir == Direction.REVERSE) {
            return -counts;
        }
        return counts;
    }

    public void reset() {
        counts = 0;
    }


    //Miscellaneous getters & setters

    //Get raw voltage from the encoder's A channel
    public boolean getChannelA() {
        return channelA;
    }

    //Get raw voltage from the encoder's B channel
    public boolean getChannelB() {
        return channelB;
    }

    public void setDir(Direction dir) {
        this.dir = dir;
    }

    public Direction getDir() {
        return dir;
    }

    //Starts the thread
    public synchronized void start() {
        thread.start();

        lastChannelA = encoderA.getState();
    }

    //Stops the thread from running
    public synchronized void stop() {
        thread.interrupt();
    }

    //Access thread from outside this class
    public Thread getThread() {
        return thread;
    }

    //Custom thread that constantly gets voltage from encoder sensor
    private class EncoderThread extends Thread {

        public EncoderThread() {
            this.setName("EncoderThread");
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted()) {
                    channelA = encoderA.getState();
                    channelB = encoderB.getState();

                    //If a pulse has occurred
                    if (lastChannelA != channelA) {
                        if (channelB != channelA) {
                            counts++;
                        }
                        else {
                            counts--;
                        }
                    }

                    lastChannelA = channelA;
                }
            }
            catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}