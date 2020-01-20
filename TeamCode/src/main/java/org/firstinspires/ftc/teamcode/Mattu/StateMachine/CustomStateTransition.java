package org.firstinspires.ftc.teamcode.Mattu.StateMachine;

import org.firstinspires.ftc.robotcore.external.Event;
import org.firstinspires.ftc.robotcore.external.State;

public class CustomStateTransition {

    protected State from;
    protected Event event;
    protected State to;

    public CustomStateTransition(State from, Event event, State to)
    {
        this.from = from;
        this.event = event;
        this.to = to;
    }

    public String toString()
    {
        return "From: " + from + " To: " + to + " On Event: " + event.getName();
    }
}
