package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.AnalogCheck;
import org.firstinspires.ftc.teamcode.drive.ButtonState;
import org.firstinspires.ftc.teamcode.drive.EventHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.ControllerState;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@Config
@TeleOp(group = "drive")
public class Teleop1Controller extends LinearOpMode{

    private FtcDashboard dashboard;

    public static double shooterVelocity = -1200;
    public static double shooterAngleDelta = 0.005;
    public static double shooterAngleConfig = 0;

    public static double shooterAngle = 0;

    @Override
    public void runOpMode() {

        Robot roboto = new Robot(hardwareMap);
        ControllerState controller1 = new ControllerState(gamepad1);
        ControllerState controller2 = new ControllerState(gamepad2);

        dashboard = roboto.drive.dashboard;

        roboto.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        roboto.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        controller1.addEventListener("x", ButtonState.PRESSED, () -> roboto.setIntakeMode(!roboto.getIntakeMode()));
        controller1.addEventListener("b", ButtonState.HELD, () -> roboto.setShooterVelocity(shooterVelocity));
        controller1.addEventListener("b", ButtonState.OFF, () -> roboto.setShooterVelocity(0));
        controller1.addEventListener("dpad_up", ButtonState.HELD, () -> roboto.changeShooterAngle(-shooterAngleDelta));
        controller1.addEventListener("dpad_down", ButtonState.HELD, () -> roboto.changeShooterAngle(shooterAngleDelta));
        controller1.addEventListener("right_trigger", AnalogCheck.GREATER_THAN, 0.1, () -> {
            roboto.setRingPusherMode(true);
            roboto.setRingBlockerMode(false);
        });
        controller1.addEventListener("right_trigger", AnalogCheck.LESS_THAN_EQUALS, 0.1, () -> {
            roboto.setRingPusherMode(false);
            roboto.setRingBlockerMode (true);
        });
        controller1.addEventListener("y", ButtonState.PRESSED, () -> roboto.setShooterAngleDeg(roboto.getShooterAngleDeg()+10));
        controller1.addEventListener("a", ButtonState.PRESSED, () -> roboto.setShooterAngleDeg(roboto.getShooterAngleDeg()-10));
        controller1.addEventListener("right_bumper", ButtonState.PRESSED, () -> roboto.setShooterAngleDeg(shooterAngleConfig));

        waitForStart();

        while (opModeIsActive()) {
            controller1.updateControllerState();

            roboto.setVel(new Pose2d(
                    -controller1.getAnalogValue("left_stick_y"),
                    -controller1.getAnalogValue("left_stick_x"),
                    -controller1.getAnalogValue("right_stick_x")));

            roboto.drive.setDrivePower(roboto.getVel());
            roboto.drive.update();

            controller1.handleEvents();

            telemetry.update();

            Pose2d myPose = roboto.drive.getPoseEstimate();

        }
    }

}
