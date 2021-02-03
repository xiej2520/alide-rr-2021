package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.ControllerState;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@TeleOp(group = "drive")
public class Teleop1 extends LinearOpMode{

    private DcMotor intakeMotor;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        Robot roboto = new Robot(hardwareMap);
        ControllerState controller1 = new ControllerState(gamepad1);
        ControllerState controller2 = new ControllerState(gamepad2);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        roboto.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        roboto.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));


        waitForStart();

        while (opModeIsActive()) {
            controller1.updateControllerState();
            controller2.updateControllerState();

            Pose2d vel = new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            roboto.drive.setDrivePower(vel);
            roboto.drive.update();


            if (controller1.x_prev == false && controller1.x == true) {
                roboto.setIntakeMode(!roboto.getIntakeMode());
            }

            if (controller1.right_trigger > 0.1) {
                roboto.setRingPusherMode(true);
                roboto.setRingBlockerMode(false);
            }
            else {
                roboto.setRingPusherMode(false);
                roboto.setRingBlockerMode (true);
            }

            if (controller1.a == true) {
                roboto.setShooterVelocity(300);
            }
            else {
                roboto.setShooterVelocity(0);
            }

            if (gamepad1.dpad_up == true) {
                roboto.changeShooterAngle(10);
            }
            if (gamepad1.dpad_down == true) {
                roboto.changeShooterAngle(-10);
            }
            telemetry.addData("shooterAngleEncoder: ", roboto.shooterAngleTarget);
            telemetry.addData("ringBlockerMode: ", roboto.getRingBlockerMode());
            telemetry.addData("ringPusherMode: ", roboto.getRingPusherMode());
            telemetry.update();
            Pose2d myPose = roboto.drive.getPoseEstimate();


            TelemetryPacket packet = new TelemetryPacket();
            packet.put("c1.x", controller1.x);
            packet.put("c1.x_prev", controller1.x_prev);
            packet.put("c1.lt", controller1.left_trigger);

            dashboard.sendTelemetryPacket(packet);
        }
    }

}