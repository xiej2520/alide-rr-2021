package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class AutoTesting extends LinearOpMode {

    private FtcDashboard dashboard;

    public static Pose2d testP = new Pose2d(0, 0, Math.toRadians(0));
    public static Vector2d testV = new Vector2d(10, 10);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot roboto = new Robot(hardwareMap);

        Pose2d startPose = testP;
        drive.setPoseEstimate(startPose);

        dashboard = roboto.drive.dashboard;

        Trajectory testTraj = drive.trajectoryBuilder(testP) // move to launch line
                .splineTo(testV, Math.toRadians(0))
                .build();

        waitForStart();

        drive.followTrajectory(testTraj);
    }
}