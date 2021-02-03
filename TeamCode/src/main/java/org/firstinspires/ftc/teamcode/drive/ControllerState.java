package org.firstinspires.ftc.teamcode.drive;

import java.lang.reflect.Field;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerState {
    public Gamepad gamepad;

    public double left_stick_x;
    public double left_stick_y;
    public double right_stick_x;
    public double right_stick_y;
    public boolean dpad_up;
    public boolean dpad_down;
    public boolean dpad_left;
    public boolean dpad_right;
    public boolean x, x_prev;
    public boolean y, y_prev;
    public boolean a, a_prev;
    public boolean b, b_prev;
    public double left_trigger;
    public double right_trigger;
    public boolean left_bumper;
    public boolean right_bumper;

    public ControllerState(Gamepad g) {
        gamepad = g;
    }

    public void updateControllerState() {
        x_prev = x;
        y_prev = y;
        a_prev = a;
        b_prev = b;

        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        dpad_up = gamepad.dpad_up;
        dpad_down = gamepad.dpad_down;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        x = gamepad.x;
        y = gamepad.y;
        a = gamepad.a;
        b = gamepad.b;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
        left_bumper = gamepad.left_bumper;
        right_bumper = gamepad.right_bumper;
    }

}
