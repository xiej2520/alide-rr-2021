package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(group = "drive")
public class Teleop1 extends LinearOpMode{

    private DcMotor intakeMotor;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        waitForStart();

        while (opModeIsActive()) {
            Pose2d vel = new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            drive.setDrivePower(vel);
            drive.update();
            if (gamepad1.a) {
                intakeMotor.setPower(1);
            }
            else {
                intakeMotor.setPower(0);
            }

            Pose2d myPose = drive.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());

        }
    }

}