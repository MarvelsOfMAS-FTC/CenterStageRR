package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BaseRobotMethods {
    BaseConstants $ = new BaseConstants();
    boolean rightclose = false;
    boolean leftclose = false;
    boolean changedRight = false;
    boolean changedLeft = false;
    Servo wrist, leftclaw, rightclaw, hook, drone;
    DcMotorEx arml, armr, sliders, hanging;

    public BaseRobotMethods(HardwareMap hardwareMap) {
        // initialise
        wrist = hardwareMap.servo.get("wrist");
        leftclaw = hardwareMap.servo.get("right");
        rightclaw = hardwareMap.servo.get("left");
        arml = hardwareMap.get(DcMotorEx.class,"arml");
        armr = hardwareMap.get(DcMotorEx.class, "armr");
        sliders = hardwareMap.get(DcMotorEx.class, "sliders");
        hanging = hardwareMap.get(DcMotorEx.class, "hanging");
        hook = hardwareMap.servo.get("hook");
        drone = hardwareMap.servo.get("drone");
        arml.setDirection(DcMotor.Direction.REVERSE);
        armr.setDirection(DcMotor.Direction.FORWARD);
        arml.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanging.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setDirection(Servo.Direction.REVERSE);
    }

    public void rightclaw() {
        rightclaw.setPosition(-0.7); // Open right claw
        rightclose = false;
        //telemetry.update();


    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rightclaw.setPosition($.RIGHT_CLAW_OPEN); // Open right claw
            rightclose = false;
            leftclaw.setPosition($.LEFT_CLAW_OPEN); // Open left claw
            leftclose = false;
            return false;
        }
    }

    public Action openClaw() {
        return new OpenClaw();
    }

    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rightclaw.setPosition($.RIGHT_CLAW_CLOSE); // Close right claw
            rightclose = true;
            leftclaw.setPosition($.LEFT_CLAW_CLOSE); // Close left claw
            leftclose = true;
            return false;
        }
    }

    public Action closeClaw() {
        return new CloseClaw();
    }

    public class OpenRightClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rightclaw.setPosition($.RIGHT_CLAW_OPEN); // Open right claw
            rightclose = false;
            return false;
        }
    }

    public Action openRightClaw() {
        return new OpenRightClaw();
    }

    public class OpenLeftClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            leftclaw.setPosition($.LEFT_CLAW_OPEN);
            leftclose = false;
            return false;
        }
    }

    public Action openLeftClaw() {

        return new OpenLeftClaw();

    }

    public class CloseRightClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rightclaw.setPosition($.RIGHT_CLAW_CLOSE); // Close right claw
            rightclose = true;
            return false;
        }
    }

    public Action CloseRightClaw() {
        return new CloseRightClaw();
    }

    public class CloseLeftClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            leftclaw.setPosition($.LEFT_CLAW_CLOSE); // Close right claw
            leftclose = true;
            return false;
        }
    }

    public Action CloseLeftClaw() {
        return new CloseLeftClaw();
    }

    public class Low implements Action  {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            sliders.setTargetPosition(2000);
            sliders.setPower(1);
            sliders.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
}






