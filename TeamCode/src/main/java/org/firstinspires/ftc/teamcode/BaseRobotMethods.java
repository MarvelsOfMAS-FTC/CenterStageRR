package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import android.util.Size;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.variable.VariableType;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class BaseRobotMethods {
    BaseConstants.Params $ = new BaseConstants.Params();
    boolean rightclose = false;
    boolean leftclose = false;
    boolean changedRight = false;
    boolean changedLeft = false;
    Servo wrist, leftclaw, rightclaw, hookR, hookL; //drone
    DcMotorEx arm, slider, hangingL, hangingR;

    //VARIABLES---------------------------------------------------------------------------------------------------------------
    private static final boolean USE_WEBCAM = true;
    ElapsedTime timer = new ElapsedTime();
    boolean holdWrist = false;
    boolean resetEncoder = false;
    private AprilTagProcessor aprilTag;
    public FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private VisionPortal visionPortalBack;
    private VisionPortal visionPortalApril;
    private ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode parent;
    public Telemetry telemetry;


    public BaseRobotMethods(HardwareMap hardwareMap) {
        // initialise
        wrist = hardwareMap.servo.get("wrist");
        leftclaw = hardwareMap.servo.get("left");
        rightclaw = hardwareMap.servo.get("right");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        hangingL = hardwareMap.get(DcMotorEx.class, "hangingL");
        hangingR = hardwareMap.get(DcMotorEx.class, "hangingR");
        hookR = hardwareMap.servo.get("hookR");
        hookL = hardwareMap.servo.get("hookL");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangingL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setDirection(Servo.Direction.REVERSE);

        //CAMERA INIT
        visionProcessor = new FirstVisionProcessor();
        initCamera(hardwareMap);
    }

    //CAMERA COMMANDS ----------------------------------------------------------------------------------------------------
    public void initCamera(HardwareMap hardwareMap){
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessor(visionProcessor)
                .setCameraResolution(new Size(864, 480)) // 544 288
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();
    }

    public class InitRobot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rightclaw.setPosition($.RIGHT_CLAW_CLOSE);
            rightclose = true;
            leftclaw.setPosition($.LEFT_CLAW_CLOSE);
            leftclose = true;

            wrist.setPosition($.WRIST_HOME_POSITION);

            arm.setTargetPosition($.ARMHOME);
            arm.setPower($.DEFAULT_SPEED);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            return false;
        }
    }

    public Action initRobot() {
        return new InitRobot();
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

    public Action closeRightClaw() {
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

    public Action closeLeftClaw() {
        return new CloseLeftClaw();
    }

    public class Low implements Action  {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slider.setTargetPosition($.LOW);
            slider.setPower($.DEFAULT_SPEED);
            slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action low(){
        return  new Low();
    }

    public class Score implements Action{ //SCORE ON BACKBOARD

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition($.WRIST_SCORE_POSITION);
            return false;
        }
    }
    public Action score(){
        return  new Score();
    }

    public class Home_Pos implements Action{ //SCORE ON BACKBOARD

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition($.WRIST_HOME_POSITION);
            return false;
        }
    }
    public Action home_pos (){
        return  new Score();
    }

    public class Place implements Action{ //SCORE ON SPIKE MARK

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition($.WRIST_PLACE_POSITION);
            return false;
        }
    }

    public Action place(){
        return  new Place();
    }

    public class StopMotors implements Action{ //stop motors from becoming shawerma

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slider.setPower(0);
            arm.setPower(0);
            return false;
        }
    }

    public Action stopMotors(){
        return  new StopMotors();
    }

    public class ScoreArm implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            telemetryPacket.put("Arm Encoder", arm.getCurrentPosition());
            telemetryPacket.put("Arm Target", $.BACKDROP);

            arm.setTargetPosition($.BACKDROP);
            arm.setPower($.DEFAULT_SPEED);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action scoreArm(){
        return  new ScoreArm();
    }
}










