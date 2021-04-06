package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Config
@Autonomous(group = "drive")
public class AutoWebcam extends LinearOpMode {

    private FtcDashboard dashboard;

    public static Pose2d startP = new Pose2d(-60, -48, Math.toRadians(180));
    public static Pose2d launchPosP = new Pose2d(12, -48, Math.toRadians(180));

    public static Vector2d zoneAV = new Vector2d(0, -60);
    public static Vector2d middleStepV = new Vector2d(-24, -60);
    public static Vector2d launchPosV = new Vector2d(6, -48);
    // public static Vector2d ringsV = new Vector2d(-36, -48); // ????????????
    public static Vector2d launchLineV = new Vector2d(42, -36);

    public static double shooterAngle = 21;
    public static double vel = 1700;
    public static double turnBeforeShoot = 150;
    public static double turnBeforeRings = 130;
    public static double turnBeforeShoot2 = 180;
    public static double extraAdjustment = 30;

    public static int shootCount = 3;
    public static int shootWait = 500;

    // Visual
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AYsOcdn/////AAABmZvIp0IupUgmkjCypXeEgyNN2r1tPBypo//yA62DEjGwAFZEyYTqSxGiQeF+qTpOMd2SaBpAKsaOH0NkROVSvcZnI0BGq48ivhtxmAYjsby63zG5Pgdr+yc7QN3HysIVvIHyjMg7gRlAUaIThl/xzXMADaShmJBSbPKX/DCMxuHwreaWm/dNL9LlWRTMCwmu6Td1+5WPgn8xokj9YVBKdQHoYK56CisCZ2vbXM+Zu2OOqsX432Wg00HVg1aJJHYZu6enwHTgwg1OS2Gk/pDdI/i9AwqcYFnEjnm+e9N7nQjxKElu2PNTmBgPSE38kzEREIX1f2awS3N3Q8odGTcPOugSPKi2+DRJCB1QD33WQK3Q";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot roboto = new Robot(hardwareMap);

        Pose2d startPose = new Pose2d(-60, -48, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        // Trajectories (paths)
        Trajectory startToZoneA1 = drive.trajectoryBuilder(startP)
                .lineToConstantHeading(middleStepV)
                .build();

        Trajectory startToZoneA2 = drive.trajectoryBuilder(startToZoneA1.end())
                .lineToConstantHeading(zoneAV)
                .build();

        Trajectory zoneAToLaunchPos = drive.trajectoryBuilder(startToZoneA2.end())
                .splineToLinearHeading(launchPosP, Math.toRadians(0))
                .build();

        Trajectory launchToRing = drive.trajectoryBuilder(zoneAToLaunchPos.end())
                .forward(36)
                .build();

        Trajectory ringsToLaunchPos = drive.trajectoryBuilder(launchToRing.end()
                .plus(new Pose2d(0, 0, Math.toRadians(turnBeforeShoot2))))
                .splineTo(launchPosV, Math.toRadians(0))
                .build();

        Trajectory launchPosToLaunchLine = drive.trajectoryBuilder(ringsToLaunchPos.end())
                .splineToConstantHeading(launchLineV, Math.toRadians(0))
                .build();

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        waitForStart();

        int rings = 0;
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        tfod.shutdown();

        for (Recognition recognition : updatedRecognitions) {
            if (recognition.getLabel() == "Quad") {
                rings = 4;
            }
            else if (recognition.getLabel() == "Single") {
                rings = 1;
            }
        }



        for (int i = 0; i < rings; i++) {
            roboto.setRingPusherMode(true);
            sleep(shootWait);
            roboto.setRingPusherMode(false);
            sleep(shootWait);
        }
        sleep(10000);





        drive.followTrajectory(startToZoneA1);
        drive.followTrajectory(startToZoneA2);

        roboto.setShooterAngleDeg(roboto.shooterAngleDegMax);
        sleep(500);
        roboto.setWobbleGrabberMode(false);
        sleep(500);

        drive.followTrajectory(zoneAToLaunchPos);

        drive.turn(Math.toRadians(turnBeforeShoot));

        // shoot 5x
        roboto.autoStartShoot(shooterAngle, vel);
        sleep(1100);
        roboto.setRingBlockerMode(false);
        for (int i = 0; i < shootCount; i++) {
            roboto.setRingPusherMode(true);
            sleep(shootWait);
            roboto.setRingPusherMode(false);
            sleep(shootWait);
        }
        roboto.autoStopShoot();

        drive.turn(Math.toRadians(turnBeforeRings));

        roboto.setIntakeMode(true);

        sleep(100);

        drive.followTrajectory(launchToRing);

        sleep(100);

        drive.turn(Math.toRadians(turnBeforeShoot2));

        roboto.setIntakeMode(false);

        drive.followTrajectory(ringsToLaunchPos);

        drive.turn(Math.toRadians(180 - turnBeforeRings - extraAdjustment));

        // shoot 5x
        roboto.autoStartShoot(shooterAngle, vel);
        sleep(1100);
        roboto.setRingBlockerMode(false);
        for (int i = 0; i < shootCount - 1; i++) {
            roboto.setRingPusherMode(true);
            sleep(shootWait);
            roboto.setRingPusherMode(false);
            sleep(shootWait);
        }
        roboto.autoStopShoot();

        drive.followTrajectory(launchPosToLaunchLine);
        roboto.setShooterAngleDeg(roboto.shooterAngleDegMax);
        sleep(1500);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}