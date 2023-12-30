package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BaseRobotMethods {
    boolean rightclose = false;
    boolean leftclose = false;
    boolean changedRight = false;
    boolean changedLeft = false;
    Servo wrist, leftclaw, rightclaw, hook, drone;
    DcMotor arml, armr, sliders, hanging;

    public BaseRobotMethods(HardwareMap hardwareMap) {
        // initialise
        wrist = hardwareMap.servo.get("wrist");
        leftclaw = hardwareMap.servo.get("right");
        rightclaw = hardwareMap.servo.get("left");
        arml = hardwareMap.dcMotor.get("ArmL");
        armr = hardwareMap.dcMotor.get("ArmR");
        sliders = hardwareMap.dcMotor.get("Slider");
        hanging = hardwareMap.dcMotor.get("hanging");
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
    public class rightClaw implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            rightclaw.setPosition(-0.7); // Open right claw
            rightclose = false;
            return false;
        }
    }
    public Action rightClaw(){
        return new rightClaw();
    }

}











