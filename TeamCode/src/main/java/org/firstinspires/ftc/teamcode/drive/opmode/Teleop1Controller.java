package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
public class Teleop1 extends LinearOpMode{

    private FtcDashboard dashboard;

    public static double shooterVelocity = -1200;
    public static int shooterAngleDelta = 20;

    @Override
    public void runOpMode() {

        Robot roboto = new Robot(hardwareMap);
        ControllerState controller1 = new ControllerState(gamepad1);
        ControllerState controller2 = new ControllerState(gamepad2);

        dashboard = roboto.drive.dashboard;

        roboto.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        roboto.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));



        controller1.addEventListener("x", ButtonState.PRESSED, () -> roboto.setIntakeMode(!roboto.getIntakeMode()));
        controller1.addEventListener("a", ButtonState.HELD, () -> roboto.setShooterVelocity(shooterVelocity));
        controller1.addEventListener("a", ButtonState.OFF, () -> roboto.setShooterVelocity(0));
        controller1.addEventListener("dpad_up", ButtonState.HELD, () -> roboto.changeShooterAngle(-shooterAngleDelta));
        controller1.addEventListener("dpad_down", ButtonState.HELD, () -> roboto.changeShooterAngle(shooterAngleDelta));

        waitForStart();

        while (opModeIsActive()) {
            controller1.updateControllerState();
            controller2.updateControllerState();

            Pose2d vel = new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            roboto.drive.setDrivePower(vel);
            roboto.drive.update();


            if (controller1.right_trigger > 0.1) {
                roboto.setRingPusherMode(true);
                roboto.setRingBlockerMode(false);
            }
            else {
                roboto.setRingPusherMode(false);
                roboto.setRingBlockerMode (true);
            }

            controller1.handleEvents();

            telemetry.update();

            Pose2d myPose = roboto.drive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterAngle", roboto.getShooterAngle());
            dashboard.sendTelemetryPacket(packet);
        }
    }

}
