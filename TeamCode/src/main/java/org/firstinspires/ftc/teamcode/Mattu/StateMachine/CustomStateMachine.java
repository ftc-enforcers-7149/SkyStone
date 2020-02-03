package org.firstinspires.ftc.teamcode.Mattu.StateMachine;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Event;
import org.firstinspires.ftc.robotcore.external.State;

import java.util.ArrayList;
import java.util.HashMap;

public class CustomStateMachine {

    private final static String TAG = "StateMachine";

    protected State currentState;
    protected HashMap<State, ArrayList<CustomStateTransition>> stateGraph;
    protected ArrayList<Event> maskList;

    public CustomStateMachine()
    {
        stateGraph = new HashMap<>();
        maskList = new ArrayList<>();
    }

    /**
     * start
     *
     * Define the start state of the state machine.  Should be called
     * from the start method of the feature's state machine.
     *
     * @param state The initial state.
     */
    public void start(State state)
    {
        currentState = state;
    }

    /**
     * addTransition
     *
     * Adds a transition to the state machine.
     *
     * @param transition the transition to add.
     */
    public void addTransition(CustomStateTransition transition)
    {
        ArrayList<CustomStateTransition> edges = stateGraph.get(transition.from);
        if (edges == null) {
            edges = new ArrayList<>();
            edges.add(transition);
            stateGraph.put(transition.from, edges);
        } else {
            edges.add(transition);
        }
    }

    /**
     * consumeEvent
     *
     * Executes a state transition and returns the new state.
     * *
     * @param event The event to consume
     * @return The new state, or the current state if the current state does not have a
     *         matching event edge.
     */
    public State consumeEvent(Event event)
    {
        if (maskList.contains(event)) {
            RobotLog.ii(TAG, "Ignoring " + event.getName() + " masked");
            return currentState;
        }

        State newState = transition(event);
        if (newState != null) {
            RobotLog.ii(TAG, "Old State: " + currentState.toString() + ", Event: " + event.getName() + ", New State: " + newState.toString());
            currentState.onExit(event);
            currentState = newState;
            currentState.onEnter(event);
        }

        return currentState;
    }

    public void maskEvent(Event event)
    {
        if (!maskList.contains(event)) {
            RobotLog.ii(TAG, "Masking " + event.getName());
            maskList.add(event);
        }
    }

    public void unMaskEvent(Event event)
    {
        if (maskList.contains(event)) {
            RobotLog.ii(TAG, "Unmasking " + event.getName());
            maskList.remove(event);
        }
    }

    protected State transition(Event event)
    {
        ArrayList<CustomStateTransition> edges = stateGraph.get(currentState);

        if (edges == null) {
            RobotLog.vv(TAG, "State with no transitions: " + currentState.toString());
            return null;
        }

        for (CustomStateTransition edge: edges) {
            if (edge.event == event) {
                return edge.to;
            }
        }
        return null;
    }

    public String toString()
    {
        String str = "\n";

        for (HashMap.Entry<State, ArrayList<CustomStateTransition>> entry : stateGraph.entrySet()) {
            State state = entry.getKey();
            ArrayList<CustomStateTransition> edges = entry.getValue();
            str += state.toString() + "\n";
            for (CustomStateTransition edge : edges) {
                str += "\t" + edge.toString() + "\n";
            }
        }

        return str;
    }
}
