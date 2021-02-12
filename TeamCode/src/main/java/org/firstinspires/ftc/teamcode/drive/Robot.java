package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Robot {
    public SampleMecanumDrive drive;


    /* Motors and Servos */
    private DcMotorEx intake;
    private DcMotorEx shooterAngle;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private Servo ringBlocker;
    private Servo ringPusher;

    /* Robot state variables */
    private boolean intakeMode;
    private boolean ringBlockerMode;
    private boolean ringPusherMode;
    private int shooterAnglePos;

    /* Config variables */
    public static double ringBlockerOffPos = 0.23;
    public static double ringBlockerOnPos = 0.35;
    public static double ringPusherOnPos = 0.12;
    public static double ringPusherOffPos = 0.6;

    public static int shooterAngleMaxPos = -320;


    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooterAngle = hardwareMap.get(DcMotorEx.class, "shooterAngle");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        ringBlocker = hardwareMap.get(Servo.class, "ringBlocker");
        ringPusher = hardwareMap.get(Servo.class, "ringPusher");


        shooterAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterAngle.setTargetPosition(0);
        shooterAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        intakeMode = false;
        ringBlockerMode = true;
        ringPusherMode = false;

        setRingBlockerMode(true);
        setRingPusherMode(false);

    }

    public boolean getIntakeMode() {
        return intakeMode;
    }
    public void setIntakeMode(boolean mode) {
        if (mode == true) {
            intakeMode = true;
            intake.setPower(1);
        }
        else {
            intakeMode = false;
            intake.setPower(0);
        }
    }

    public boolean getRingBlockerMode() {
        return ringBlockerMode;
    }
    public void setRingBlockerMode(boolean mode) {
        if (mode == true) {
            ringBlockerMode = true;
            ringBlocker.setPosition(ringBlockerOnPos);
        }
        else {
            ringBlockerMode = false;
            ringBlocker.setPosition(ringBlockerOffPos);
        }
    }

    public boolean getRingPusherMode() {
        return ringPusherMode;
    }
    public void setRingPusherMode(boolean mode) {
        if (mode == true) {
            ringPusherMode = true;
            ringPusher.setPosition(ringPusherOnPos);
        }
        else {
            ringPusherMode = false;
            ringPusher.setPosition(ringPusherOffPos);
        }
    }

    public void setShooterVelocity(double v) {
        shooter1.setVelocity(v);
        shooter2.setVelocity(v);
    }

    public void changeShooterAngle(int delta) {
        if (shooterAngle.getTargetPosition() + delta < shooterAngleMaxPos) {
            shooterAngle.setTargetPosition(shooterAngleMaxPos);
        }
        else if (shooterAngle.getTargetPosition() + delta > 0) {
            shooterAngle.setTargetPosition(0);
        }
        else {
            shooterAngle.setTargetPosition(shooterAngle.getCurrentPosition() + delta);
        }

        if (shooterAngle.getCurrentPosition() < shooterAngle.getTargetPosition()) {
            shooterAngle.setPower(0.1);
        }
        else if (shooterAngle.getCurrentPosition() > shooterAngle.getTargetPosition()) {
            shooterAngle.setPower(-0.1);
        }
        else {
            shooterAngle.setPower(0);
        }

        shooterAnglePos = shooterAngle.getCurrentPosition();
    }
    public int getShooterAngle() {
        return shooterAnglePos;
    }

}