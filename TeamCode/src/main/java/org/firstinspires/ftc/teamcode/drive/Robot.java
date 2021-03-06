package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Robot {
    // Roadrunner sample mecanum drive implementation
    public SampleMecanumDrive drive;

    // Motors and Servos
    private DcMotorEx intakeTop; // top roller (big wheels)
    private DcMotorEx intakeBottom; // bottom roller (small wheels + ramp)

    public DcMotorEx shooterFront;
    public DcMotorEx shooterBack;
    private Servo shooterAngle;

    private Servo ringBlocker;
    private Servo ringPusher;

    private Servo wobbleGrabber;

    // Robot state variables
    private Pose2d vel;

    private boolean intakeMode; // true:on, false:off
    private boolean intakeDirection; // true:in, false:out

    private boolean ringBlockerMode;
    private boolean ringPusherMode;
    private boolean ringPusherModeTeleop;

    private double shooterAnglePos;
    private double shooterAngleDeg;
    private double shooterVelocity;

    private boolean wobbleGrabberMode; // true:closed, false: open

    // Config variables
    public static double shooterPower;

    public static double ringBlockerOffPos = 0.23;
    public static double ringBlockerOnPos = 0.35;
    public static double ringPusherOnPos = 0.55; // auto
    public static double ringPusherOnPosTeleop = 0.55; // teleop
    public static double ringPusherOffPos = .26;

    public static double shooterAnglePosMax = 0.75; // ~0 degrees, almost hits plastic
    public static double shooterAnglePosMin = 0.07; // 90 degrees up
    public static double shooterAngleDegMin = 0; // from horizontal
    public static double shooterAngleDegMax = 90; // from horizontal

    public static double wobbleGrabberPosMax = 0.3; // closed
    public static double wobbleGrabberPosMin = 0.6; // open


    public Robot(HardwareMap hardwareMap) {
        // initialize
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeTop = hardwareMap.get(DcMotorEx.class, "intake1");
        intakeBottom = hardwareMap.get(DcMotorEx.class, "intake2");

        shooterFront = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterBack = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterAngle = hardwareMap.get(Servo.class, "shooterAngle");

        ringBlocker = hardwareMap.get(Servo.class, "ringBlocker");
        ringPusher = hardwareMap.get(Servo.class, "ringPusher");

        wobbleGrabber = hardwareMap.get(Servo.class, "wobbleGrabber");


        // intake motors need to be opposite directions
        intakeTop.setDirection(DcMotor.Direction.REVERSE);

        // Using setVelocity()
        shooterFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterFront.setDirection(DcMotor.Direction.REVERSE);
        shooterBack.setDirection(DcMotor.Direction.REVERSE);
        shooterFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        vel = new Pose2d(0, 0, 0);

        setIntakeMode(false);
        setIntakeDirection(true);

        setShooterVelocity(0);
        setShooterAngle(shooterAnglePosMax);

        setRingBlockerMode(true);
        setRingPusherMode(false);

        setWobbleGrabberMode(true);
    }

    public boolean getIntakeMode() { return intakeMode; }
    public void setIntakeMode(boolean mode) {
        intakeMode = mode;
        if (intakeMode) {
            if (intakeDirection) {
                intakeTop.setPower(1);
                intakeBottom.setPower(1);
            }
            else {
                intakeTop.setPower(-1);
                intakeBottom.setPower(-1);
            }
        }
        else {
            intakeTop.setPower(0);
            intakeBottom.setPower(0);
        }
    }

    public boolean getIntakeDirection() { return intakeDirection; }
    public void setIntakeDirection(boolean direction) {
        intakeDirection = direction;
        // Intake power only updates when setIntakeMode is called
        if (getIntakeMode()) {
            setIntakeMode(getIntakeMode());
        }
    }

    public boolean getRingBlockerMode() { return ringBlockerMode; }
    public void setRingBlockerMode(boolean mode) {
        if (mode) {
            ringBlockerMode = true;
            ringBlocker.setPosition(ringBlockerOnPos);
        }
        else {
            ringBlockerMode = false;
            ringBlocker.setPosition(ringBlockerOffPos);
        }
    }

    public boolean getRingPusherMode() { return ringPusherMode; }
    public void setRingPusherMode(boolean mode) {
        if (mode) {
            ringPusherMode = true;
            ringPusher.setPosition(ringPusherOnPos);
        }
        else {
            ringPusherMode = false;
            ringPusher.setPosition(ringPusherOffPos);
        }
    }

    public boolean getRingPusherModeTeleop() { return ringPusherModeTeleop; }
    public void setRingPusherModeTeleop(boolean mode) {
        if (mode) {
            ringPusherModeTeleop = true;
            ringPusher.setPosition(ringPusherOnPosTeleop);
        }
        else {
            ringPusherModeTeleop = false;
            ringPusher.setPosition(ringPusherOffPos);
        }
    }

    public Pose2d getVel() { return vel; }
    public void setVel(Pose2d v) {
        vel = v;
    }

    public double getShooterVelocity() { return shooterVelocity; }
    public double getShooterFrontPower() { return shooterFront.getPower(); }
    public double getShooterBackPower() { return shooterBack.getPower(); }
    public void setShooterVelocity(double v) {
        shooterVelocity = v;
        shooterFront.setVelocity(v);
        shooterBack.setVelocity(v);
    }

    public double getShooterAnglePos() { return shooterAnglePos; }
    public double getShooterAngleDeg() { return shooterAngleDeg; }
    public void setShooterAngle(double pos) {
        double limitedPos = Math.min(pos, shooterAnglePosMax);
        limitedPos = Math.max(limitedPos, shooterAnglePosMin);

        shooterAnglePos = limitedPos;
        shooterAngleDeg = linearMap(limitedPos, shooterAnglePosMax, shooterAnglePosMin, shooterAngleDegMin, shooterAngleDegMax);
        shooterAngle.setPosition(limitedPos);
    }
    public void setShooterAngleDeg(double angle) {
        double limitedAngle = Math.min(angle, shooterAngleDegMax);
        limitedAngle = Math.max(limitedAngle, shooterAngleDegMin);

        setShooterAngle(linearMap(limitedAngle, shooterAngleDegMin, shooterAngleDegMax, shooterAnglePosMax, shooterAnglePosMin));
    }

    public double linearMap(double val, double oldMin, double oldMax, double newMin, double newMax) {
        return newMin + (newMax - newMin) / (oldMax-oldMin) * (val - oldMin);
    }

    public boolean getWobbleGrabberMode() { return wobbleGrabberMode; }
    public void setWobbleGrabberMode(boolean mode) {
        if (mode) {
            wobbleGrabberMode = true;
            wobbleGrabber.setPosition(wobbleGrabberPosMax);
        }
        else {
            wobbleGrabberMode = false;
            wobbleGrabber.setPosition(wobbleGrabberPosMin);
        }
    }


    public void autoStartShoot(double angle, double v) {
        setShooterAngleDeg(angle);
        setShooterVelocity(v);
    }

    public void autoStopShoot() {
        setShooterAngleDeg(0);
        setShooterVelocity(0);
        setRingBlockerMode(true);
    }

}
