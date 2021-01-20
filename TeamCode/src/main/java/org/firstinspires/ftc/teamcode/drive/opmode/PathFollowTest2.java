package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class PathFollowTest2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(25, 20), Math.toRadians(90))
                .splineTo(new Vector2d(0, 40), Math.toRadians(180))
                .splineTo(new Vector2d(-25, 20), Math.toRadians(270))
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .splineTo(new Vector2d(25, -20), Math.toRadians(270))
                .splineTo(new Vector2d(0, -40), Math.toRadians(180))
                .splineTo(new Vector2d(-25, -20), Math.toRadians(90))
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
    }
}