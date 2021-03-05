package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.AnalogCheck;
import org.firstinspires.ftc.teamcode.drive.ButtonState;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.ControllerState;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@Config
@TeleOp(group = "drive")
public class Teleop2Controllers extends LinearOpMode{

    private FtcDashboard dashboard;

    public static double shooterVelocity = -1200;

    @Override
    public void runOpMode() {

        Robot roboto = new Robot(hardwareMap);
        roboto.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        ControllerState controller1 = new ControllerState(gamepad1);
        ControllerState controller2 = new ControllerState(gamepad2);

        dashboard = roboto.drive.dashboard;

        // Controller 1
        // Small changes in robot position with dpad, will override joysticks
        controller1.addEventListener("dpad_up", ButtonState.HELD, () -> roboto.setVel(new Pose2d(0.1, 0, 0)));
        controller1.addEventListener("dpad_down", ButtonState.HELD, () -> roboto.setVel(new Pose2d(-0.1, 0, 0)));
        controller1.addEventListener("dpad_left", ButtonState.HELD, () -> roboto.setVel(new Pose2d(0, 0.1, 0)));
        controller1.addEventListener("dpad_right", ButtonState.HELD, () -> roboto.setVel(new Pose2d(0, -0.1, 0)));
        controller1.addEventListener("left_trigger", AnalogCheck.GREATER_THAN, 0.1, () -> roboto.setVel(new Pose2d(0, 0, 10)));
        controller1.addEventListener("right_trigger", AnalogCheck.GREATER_THAN, 0.1, () -> roboto.setVel(new Pose2d(0, 0, -10)));

        // Controller 2
        // Toggle intake on if shooter angle is less than 30, toggle off on other press
        controller2.addEventListener("x", ButtonState.PRESSED, () -> {
            if (roboto.getShooterAngleDeg() < 30 && !roboto.getIntakeMode()) { roboto.setIntakeMode(true); }
            else { roboto.setIntakeMode(false); }
        });
        // Reverse intake direction with b
        controller2.addEventListener("b", ButtonState.PRESSED, () -> roboto.setIntakeDirection(!roboto.getIntakeDirection()));
        // Shooter on iff a is pressed
        controller2.addEventListener("a", ButtonState.HELD, () -> roboto.setShooterVelocity(shooterVelocity));
        controller2.addEventListener("a", ButtonState.OFF, () -> roboto.setShooterVelocity(0));
        // Large change in shooter angle with dpad_up and dpad_down
        controller2.addEventListener("dpad_up", ButtonState.HELD, () -> roboto.setShooterAngleDeg(roboto.getShooterAngleDeg()+10));
        controller2.addEventListener("dpad_down", ButtonState.HELD, () -> roboto.setShooterAngleDeg(roboto.getShooterAngleDeg()-10));
        // Lower blocker if shooter is on and button is pressed
        controller2.addEventListener("left_trigger", AnalogCheck.GREATER_THAN, 0.1, () -> {
            if (roboto.getShooterVelocity() > 0) { roboto.setRingBlockerMode(false); }
        });
        controller2.addEventListener("left_trigger", AnalogCheck.LESS_THAN_EQUALS, 0.1, () -> roboto.setRingBlockerMode(true));
        // Push ring out if blocker is down when button is pressed
        controller2.addEventListener("right_trigger", AnalogCheck.GREATER_THAN, 0.1, () -> {
            if (!roboto.getRingBlockerMode()) { roboto.setRingPusherMode(true); }
        });
        controller2.addEventListener("right_trigger", AnalogCheck.LESS_THAN_EQUALS, 0.1, () -> roboto.setRingPusherMode(false));
        // Toggle wobble grabber with y
        controller2.addEventListener("y", ButtonState.PRESSED, () -> roboto.setWobbleGrabberMode(!roboto.getWobbleGrabberMode()));


        waitForStart();

        while (opModeIsActive()) {
            controller1.updateControllerState();
            controller2.updateControllerState();

            roboto.setVel(new Pose2d(
                    -Math.pow(controller1.getAnalogValue("left_stick_y"), 3),
                    -Math.pow(controller1.getAnalogValue("left_stick_x"), 3),
                    -Math.pow(controller1.getAnalogValue("right_stick_x"), 3)));

            roboto.drive.setDrivePower(roboto.getVel()); // Joystick driving

            controller1.handleEvents();
            controller2.handleEvents();

            // Allow dpad override
            roboto.drive.update();

            telemetry.update();

            Pose2d myPose = roboto.drive.getPoseEstimate();

        }
    }

}
