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
public class Teleop1Controller extends LinearOpMode{

    private FtcDashboard dashboard;

    public static double shooterVelocity = 1600; // Should be possible with 11 voltage battery
    public static double shooterAnglePreset = 27.5;
    public static boolean slowDrive = false;

    @Override
    public void runOpMode() {

        Robot roboto = new Robot(hardwareMap);
        roboto.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        ControllerState controller1 = new ControllerState(gamepad1);


        // Toggle intake on if shooter angle is less than 30, toggle off on other press
        controller1.addEventListener("dpad_up", ButtonState.PRESSED, () -> {
            if ( roboto.getShooterAngleDeg() < 30 && !roboto.getIntakeMode()) { roboto.setIntakeMode(true); }
            else { roboto.setIntakeMode(false); }
        });
        // Reverse intake direction with dpad_down
        controller1.addEventListener("dpad_down", ButtonState.PRESSED, () -> roboto.setIntakeDirection(!roboto.getIntakeDirection()));
        // Shooter on iff b is held
        controller1.addEventListener("b", ButtonState.HELD, () -> roboto.setShooterVelocity(shooterVelocity));
        controller1.addEventListener("b", ButtonState.OFF, () -> roboto.setShooterVelocity(0));
        // Large change in shooter angle with y and a
        controller1.addEventListener("y", ButtonState.PRESSED, () -> roboto.setShooterAngleDeg(roboto.getShooterAngleDeg()+10));
        controller1.addEventListener("a", ButtonState.PRESSED, () -> roboto.setShooterAngleDeg(roboto.getShooterAngleDeg()-2.5));
        // shooter angle to zero
        controller1.addEventListener("left_bumper", ButtonState.PRESSED, () -> roboto.setShooterAngleDeg(0));
        // Preset shooter angle
        controller1.addEventListener("right_bumper", ButtonState.PRESSED, () -> roboto.setShooterAngleDeg(shooterAnglePreset));
        // Lower blocker if shooter is on and button is pressed
        controller1.addEventListener("left_trigger", AnalogCheck.GREATER_THAN, 0.1, () -> {
            if (roboto.getShooterVelocity() > 0) { roboto.setRingBlockerMode(false); }
        });
        controller1.addEventListener("left_trigger", AnalogCheck.LESS_THAN_EQUALS, 0.1, () -> roboto.setRingBlockerMode(true));
        // Push ring out if blocker is down when button is pressed
        controller1.addEventListener("right_trigger", AnalogCheck.GREATER_THAN, 0.1, () -> {
            if (!roboto.getRingBlockerMode()) { roboto.setRingPusherModeTeleop(true); }
        });
        controller1.addEventListener("right_trigger", AnalogCheck.LESS_THAN_EQUALS, 0.1, () -> roboto.setRingPusherModeTeleop(false));
        // Toggle wobble grabber with dpad_left
        controller1.addEventListener("dpad_left", ButtonState.PRESSED, () -> roboto.setWobbleGrabberMode(!roboto.getWobbleGrabberMode()));
        // Toggle Slow Drive
        controller1.addEventListener("x", ButtonState.PRESSED, () -> slowDrive = !slowDrive);

        waitForStart();

        while (opModeIsActive()) {
            controller1.updateControllerState();

            double correctedVelY = -Math.pow(controller1.getAnalogValue("left_stick_y"), 3);
            double correctedVelX = -Math.pow(controller1.getAnalogValue("left_stick_x"), 3);
            double correctedVelW = -Math.pow(controller1.getAnalogValue("right_stick_x"), 3);
            if (slowDrive) {
                correctedVelY = Math.signum(correctedVelY) * Math.min(Math.abs(correctedVelY), 0.25);
                correctedVelX = Math.signum(correctedVelX) * Math.min(Math.abs(correctedVelX), 0.25);
                correctedVelW = Math.signum(correctedVelW) * Math.min(Math.abs(correctedVelW), 0.25);
            }

            roboto.setVel(new Pose2d(correctedVelY, correctedVelX, correctedVelW));

            controller1.handleEvents();

            // Allow dpad override
            roboto.drive.setDrivePower(roboto.getVel());
            roboto.drive.update();

            telemetry.addData("shooterFront Power", roboto.getShooterFrontPower());
            telemetry.addData("shooterBack Power", roboto.getShooterBackPower());
            telemetry.addData("left_stick_y", controller1.getAnalogValue("left_stick_y"));
            telemetry.addData("left_stick_x", controller1.getAnalogValue("left_stick_x"));
            telemetry.addData("right_stick_x", controller1.getAnalogValue("right_stick_x"));

            telemetry.addData("xVel", -Math.pow(controller1.getAnalogValue("left_stick_x"), 3));
            telemetry.update();

            Pose2d myPose = roboto.drive.getPoseEstimate();

        }
    }

}
