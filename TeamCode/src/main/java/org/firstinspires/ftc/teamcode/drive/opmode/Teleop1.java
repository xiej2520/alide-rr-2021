package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.ControllerState;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(group = "drive")
public class Teleop1 extends LinearOpMode{

    private DcMotor intakeMotor;

    @Override
    public void runOpMode() {

        Robot roboto = new Robot(hardwareMap);
        ControllerState controller1 = new ControllerState(gamepad1);
        ControllerState controller2 = new ControllerState(gamepad2);

        roboto.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        roboto.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));


        waitForStart();

        while (opModeIsActive()) {
            controller1.updateControllerState();
            controller2.updateControllerState();

            Pose2d vel = new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            roboto.drive.setDrivePower(vel);
            roboto.drive.update();


            if (controller2.x_prev == false && controller2.x == true) {
                roboto.setIntakeMode(!roboto.getIntakeMode());
            }

            if (controller2.left_trigger > 0.1) {
                roboto.setRingBlockerMode(true);
            }
            else {
                roboto.setRingBlockerMode (false);
            }

            if (controller2.right_trigger > 0.1) {
                roboto.setRingPusherMode(true);
            }
            else {
                roboto.setRingPusherMode(false);
            }

            if (controller2.a == true) {
                roboto.setShooterVelocity(300);
            }

            if (controller2.dpad_up == true) {
                roboto.changeShooterAngle(10);
            }
            if (controller2.dpad_down == true) {
                roboto.changeShooterAngle(-10);
            }

            Pose2d myPose = roboto.drive.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());

        }
    }

}