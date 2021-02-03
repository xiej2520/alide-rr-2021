package org.firstinspires.ftc.teamcode.drive;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Robot {
    public SampleMecanumDrive drive;


    private DcMotorEx intake;
    private DcMotorEx shooterAngle;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private Servo ringBlocker;
    private Servo ringPusher;

    private boolean intakeMode;
    private boolean ringBlockerMode;
    private boolean ringPusherMode;


    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooterAngle = hardwareMap.get(DcMotorEx.class, "shooterAngle");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        ringBlocker = hardwareMap.get(Servo.class, "ringBlocker");
        ringPusher = hardwareMap.get(Servo.class, "ringPusher");


        shooterAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMode = false;
        ringBlockerMode = false;
        ringPusherMode = false;
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
            ringBlocker.setPosition(1);
        }
        else {
            ringBlockerMode = false;
            ringBlocker.setPosition(0);
        }
    }

    public boolean getRingPusherMode() {
        return ringBlockerMode;
    }
    public void setRingPusherMode(boolean mode) {
        if (mode == true) {
            ringPusherMode = true;
            ringPusher.setPosition(1);
        }
        else {
            ringPusherMode = false;
            ringPusher.setPosition(0);
        }
    }

    public void setShooterVelocity(double v) {
        shooter1.setVelocity(v);
        shooter2.setVelocity(v);
    }

    public void changeShooterAngle(double d) {
        if (shooterAngle.getTargetPosition() + d < 0) {
            shooterAngle.setTargetPosition(0);
        }
        else if (shooterAngle.getTargetPosition() + d > 400) {
            shooterAngle.setTargetPosition(400);
        }
        else {
            shooterAngle.setTargetPosition(shooterAngle.getCurrentPosition() + (int) d);
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

    }

}
