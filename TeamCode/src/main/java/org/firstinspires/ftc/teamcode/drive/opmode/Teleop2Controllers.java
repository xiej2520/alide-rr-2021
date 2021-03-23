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

    public static double shooterVelocity = 1600;
    public static double shooterAnglePreset = 30;

    @Override
    public void runOpMode() {

        Robot roboto = new Robot(hardwareMap);
        roboto.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        ControllerState controller1 = new ControllerState(gamepad1);
        ControllerState controller2 = new ControllerState(gamepad2);

        // Controller 1
        // Small changes in robot position with dpad, will override joysticks
        controller1.addEventListener("dpad_up", ButtonState.HELD, () -> roboto.setVel(new Pose2d(0.25, 0, 0)));
        controller1.addEventListener("dpad_down", ButtonState.HELD, () -> roboto.setVel(new Pose2d(-0.25, 0, 0)));
        controller1.addEventListener("dpad_left", ButtonState.HELD, () -> roboto.setVel(new Pose2d(0, 0.25, 0)));
        controller1.addEventListener("dpad_right", ButtonState.HELD, () -> roboto.setVel(new Pose2d(0, -0.25, 0)));
        controller1.addEventListener("left_trigger", AnalogCheck.GREATER_THAN, 0.1, () -> roboto.setVel(new Pose2d(0, 0, 0.2)));
        controller1.addEventListener("right_trigger", AnalogCheck.GREATER_THAN, 0.1, () -> roboto.setVel(new Pose2d(0, 0, -0.2)));

        // Controller 2
        // Toggle intake on if shooter angle is less than 30, toggle off on other press
        controller2.addEventListener("dpad_up", ButtonState.PRESSED, () -> {
            if (roboto.getShooterAngleDeg() < 30 && !roboto.getIntakeMode()) { roboto.setIntakeMode(true); }
            else { roboto.setIntakeMode(false); }
        });
        // Reverse intake direction with dpad_down
        controller2.addEventListener("dpad_down", ButtonState.PRESSED, () -> roboto.setIntakeDirection(!roboto.getIntakeDirection()));
        // Shooter on iff b is pressed
        controller2.addEventListener("b", ButtonState.HELD, () -> roboto.setShooterVelocity(shooterVelocity));
        controller2.addEventListener("b", ButtonState.OFF, () -> roboto.setShooterVelocity(0));
        // Large change in shooter angle with y and a
        controller2.addEventListener("y", ButtonState.PRESSED, () -> roboto.setShooterAngleDeg(roboto.getShooterAngleDeg()+10));
        controller2.addEventListener("a", ButtonState.PRESSED, () -> roboto.setShooterAngleDeg(roboto.getShooterAngleDeg()-10));
        // Small change in shooter angle with left stick y
        controller2.addEventListener("left_stick_y", AnalogCheck.LESS_THAN, -0.1, () -> roboto.setShooterAngleDeg(roboto.getShooterAngleDeg()-1));
        controller2.addEventListener("left_stick_y", AnalogCheck.GREATER_THAN, 0.1, () -> roboto.setShooterAngleDeg(roboto.getShooterAngleDeg()+1));
        // Preset shooter angle
        controller1.addEventListener("right_bumper", ButtonState.PRESSED, () -> roboto.setShooterAngleDeg(shooterAnglePreset));
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
        // Toggle wobble grabber with dpad_left
        controller2.addEventListener("dpad_left", ButtonState.PRESSED, () -> roboto.setWobbleGrabberMode(!roboto.getWobbleGrabberMode()));


        waitForStart();

        while (opModeIsActive()) {
            controller1.updateControllerState();
            controller2.updateControllerState();

            roboto.setVel(new Pose2d(
                    -Math.pow(controller1.getAnalogValue("left_stick_y"), 3),
                    -Math.pow(controller1.getAnalogValue("left_stick_x"), 3),
                    -Math.pow(controller1.getAnalogValue("right_stick_x"), 3)));


            controller1.handleEvents();
            controller2.handleEvents();

            // Allow dpad override
            roboto.drive.setDrivePower(roboto.getVel());
            roboto.drive.update();

            telemetry.update();

            Pose2d myPose = roboto.drive.getPoseEstimate();

        }
    }

}
