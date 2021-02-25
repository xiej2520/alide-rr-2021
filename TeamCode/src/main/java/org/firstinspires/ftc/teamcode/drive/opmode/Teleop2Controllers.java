package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.AnalogCheck;
import org.firstinspires.ftc.teamcode.drive.ButtonState;
import org.firstinspires.ftc.teamcode.drive.ControllerState;
import org.firstinspires.ftc.teamcode.drive.EventHandler;
import org.firstinspires.ftc.teamcode.drive.Robot;


@Config
@TeleOp(group = "drive")
public class Teleop2Controllers extends LinearOpMode{

    private FtcDashboard dashboard;

    public static double shooterVelocity = -1200;
    public static double shooterAngleDelta = 0.005;

    @Override
    public void runOpMode() {

        Robot roboto = new Robot(hardwareMap);
        ControllerState controller1 = new ControllerState(gamepad1);
        ControllerState controller2 = new ControllerState(gamepad2);

        dashboard = roboto.drive.dashboard;

        roboto.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        roboto.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));



        controller1.addEventListener("dpad_up", ButtonState.HELD, () -> roboto.setVel(new Pose2d(0.1, 0, 0)));
        controller1.addEventListener("dpad_down", ButtonState.HELD, () -> roboto.setVel(new Pose2d(-0.1, 0, 0)));
        controller1.addEventListener("dpad_left", ButtonState.HELD, () -> roboto.setVel(new Pose2d(0, 0.1, 0)));
        controller1.addEventListener("dpad_right", ButtonState.HELD, () -> roboto.setVel(new Pose2d(0, -0.1, 0)));

        controller2.addEventListener("x", ButtonState.PRESSED, () -> roboto.setIntakeMode(!roboto.getIntakeMode()));
        controller2.addEventListener("a", ButtonState.HELD, () -> roboto.setShooterVelocity(shooterVelocity));
        controller2.addEventListener("a", ButtonState.OFF, () -> roboto.setShooterVelocity(0));
        controller2.addEventListener("dpad_up", ButtonState.HELD, () -> roboto.changeShooterAngle(-shooterAngleDelta));
        controller2.addEventListener("dpad_down", ButtonState.HELD, () -> roboto.changeShooterAngle(shooterAngleDelta));
        controller2.addEventListener("right_trigger", AnalogCheck.GREATER_THAN, 0.1, () -> {
            roboto.setRingPusherMode(true);
            roboto.setRingBlockerMode(false);
        });
        controller2.addEventListener("right_trigger", AnalogCheck.LESS_THAN_EQUALS, 0.1, () -> {
            roboto.setRingPusherMode(false);
            roboto.setRingBlockerMode (true);
        });

        waitForStart();

        while (opModeIsActive()) {
            controller1.updateControllerState();
            controller2.updateControllerState();

            roboto.setVel(new Pose2d(
                    -controller1.getAnalogValue("left_stick_y"),
                    -controller1.getAnalogValue("left_stick_x"),
                    -controller1.getAnalogValue("right_stick_x")));

            roboto.drive.setDrivePower(roboto.getVel()); // Joystick driving


            roboto.drive.update();
            controller1.handleEvents();
            controller2.handleEvents();

            telemetry.update();

            Pose2d myPose = roboto.drive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterAngle", roboto.getShooterAngle());
            dashboard.sendTelemetryPacket(packet);
        }
    }

}
