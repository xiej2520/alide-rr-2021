package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;


/* Event handling inspiration from: https://codereview.stackexchange.com/questions/203642/event-handling-gamepad-in-java */
public class ControllerState {
    public Gamepad gamepad;

    public double left_stick_x;
    public double left_stick_y;
    public double right_stick_x;
    public double right_stick_y;
    public double left_trigger;
    public double right_trigger;
    private Button dpad_up;
    private Button dpad_down;
    private Button dpad_left;
    private Button dpad_right;
    private Button x;
    private Button y;
    private Button a;
    private Button b;
    private Button left_bumper;
    private Button right_bumper;

    private Map<String, Button> buttonMap = new HashMap<>();
    private Map<ButtonEvent, EventHandler> buttonEventListeners = new HashMap<>();


    public ControllerState(Gamepad g) {
        gamepad = g;

        dpad_up = new Button();
        dpad_down = new Button();
        dpad_left = new Button();
        dpad_right = new Button();
        x = new Button();
        y = new Button();
        a = new Button();
        b = new Button();
        left_bumper = new Button();
        right_bumper = new Button();

        buttonMap.put("dpad_up", dpad_up);
        buttonMap.put("dpad_down", dpad_down);
        buttonMap.put("dpad_left", dpad_left);
        buttonMap.put("dpad_right", dpad_right);
        buttonMap.put("x", x);
        buttonMap.put("y", y);
        buttonMap.put("a", a);
        buttonMap.put("b", b);
        buttonMap.put("left_bumper", left_bumper);
        buttonMap.put("right_bumper", right_bumper);
    }

    public void updateControllerState() {
        dpad_up.update(gamepad.dpad_up);
        dpad_down.update(gamepad.dpad_down);
        dpad_left.update(gamepad.dpad_left);
        dpad_right.update(gamepad.dpad_right);
        x.update(gamepad.x);
        y.update(gamepad.y);
        a.update(gamepad.a);
        b.update(gamepad.b);
        left_bumper.update(gamepad.left_bumper);
        right_bumper.update(gamepad.right_bumper);

        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
    }

    public void addEventListener(String buttonName, ButtonState state, EventHandler handler) {
        buttonEventListeners.put(new ButtonEvent(buttonMap.get(buttonName), state), handler);
    }

    public void handleEvents() {
        for(Entry<ButtonEvent, EventHandler> entry: buttonEventListeners.entrySet()) {
            if (entry.getKey().checkEvent() == true) {
                entry.getValue().execute();
            }
        }
    }
}

class Button {
    private boolean value;
    private ButtonState state;

    public Button() {
        value = false;
        state = ButtonState.OFF;
    }

    public boolean getValue() {
        return value;
    }
    public ButtonState getState() {
        return state;
    }
    public void update(boolean newValue) {
        if (state == ButtonState.OFF) {
            if (newValue == false) {}
            else if (newValue == true) {
                value = true;
                state = ButtonState.PRESSED;
            }
        }
        else if (state == ButtonState.PRESSED) {
            if (newValue == false) {
                value = false;
                state = ButtonState.OFF;
            }
            else if (newValue = true) {
                state = ButtonState.HELD;
            }
        }
        else if (state == ButtonState.HELD) {
            if (newValue == false) {
                value = false;
                state = ButtonState.OFF;
            }
            else if (newValue == true) {}
        }
    }
}

class ButtonEvent {
    private Button button;
    private ButtonState state;
    public ButtonEvent(Button b, ButtonState s) {
        button = b;
        state = s;
    }
    public boolean checkEvent() {
        if (button.getState() == state) {
            return true;
        }
        else {
            return false;
        }
    }
}

/*
Example usage of event handler:

    controller1.addEventListener("x", ButtonState.PRESSED, new EventHandler() {
        public void execute() {
            roboto.setIntakeMode(!roboto.getIntakeMode());
        }
    });

Or replace with lambda:
    controller1.addEventListener("x", ButtonState.PRESSED, () -> roboto.setIntakeMode(!roboto.getIntakeMode()));
 */