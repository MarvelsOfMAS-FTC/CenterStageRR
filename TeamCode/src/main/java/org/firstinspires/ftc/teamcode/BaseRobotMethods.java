package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import android.util.Size;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
public class BaseRobotMethods extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    BaseConstants.Params $ = new BaseConstants.Params();
    private AprilTagProcessor aprilTag;
    public FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    DcMotorEx intake, climbl, climbr, extend, fl, fr, bl, br;
    Servo wrist, lhook, rhook, droneLaunch, score, finger;
    private NormalizedColorSensor lcolor;
    private NormalizedColorSensor rcolor;
    private DistanceSensor ldistance;
    private DistanceSensor rdistance;
    TouchSensor elevatorLimit;
    public LinearOpMode parent;
    public Telemetry telemetry;
    //HARDWARE SETUP-------------------------------------------------------------------------------------------------
    public BaseRobotMethods(HardwareMap hardwareMap) {
        //MOTORS INIT
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        climbl = hardwareMap.get(DcMotorEx.class, "lclimb");
        climbr = hardwareMap.get(DcMotorEx.class, "rclimb");
        extend = hardwareMap.get(DcMotorEx.class, "extend");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extend.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extend.setTargetPosition($.HOME);
        extend.setPower($.EXT_PWR);
        extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        climbl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climbr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climbl.setPower($.ZERO);
        climbr.setPower($.ZERO);
        climbl.setTargetPosition($.HOME);
        climbr.setTargetPosition($.HOME);
        climbl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climbr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climbl.setPower($.FULL_PWR);
        climbr.setPower($.FULL_PWR);

        wrist = hardwareMap.get(Servo.class, "wrist");
        lhook = hardwareMap.get(Servo.class, "lhook");
        rhook = hardwareMap.get(Servo.class, "rhook");
        score = hardwareMap.get(Servo.class, "score");
        droneLaunch = hardwareMap.get(Servo.class, "launch");
        finger = hardwareMap.get(Servo.class, "finger");
        elevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorLimit");

        score.setPosition($.SCORE_HOME);

        lcolor = hardwareMap.get(NormalizedColorSensor.class, "ldist");
        rcolor = hardwareMap.get(NormalizedColorSensor.class, "rdist");
        ldistance = (DistanceSensor) lcolor;
        rdistance = (DistanceSensor) rcolor;

        //SERVO POS
        wrist.setPosition($.WRIST_IN);
        finger.setPosition($.FINGER_GND);
        droneLaunch.setPosition($.DRONE_HOME);
        ldistance = (DistanceSensor) lcolor;
        rdistance = (DistanceSensor) rcolor;
        //CAMERA INIT
        visionProcessor = new FirstVisionProcessor();

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1);
        initCamera(hardwareMap);
    }
    //CAMERA COMMANDS ----------------------------------------------------------------------------------------------------
    public void initCamera(HardwareMap hardwareMap){
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcamback"))
                .addProcessors(getVisionProcessor(), getAprilTag())
                .setCameraResolution(new Size(864, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();
        visionPortal.setProcessorEnabled(getAprilTag(),false);
        visionPortal.setProcessorEnabled(getVisionProcessor(), true);
        FtcDashboard.getInstance().startCameraStream(getPortal(), 60);
    }
    public VisionProcessor getVisionProcessor(){
        return visionProcessor;
    }
    public VisionPortal getPortal(){
        return visionPortal;
    }
    public AprilTagProcessor getAprilTag(){
        return aprilTag;
    }
    public void useApriltag(){
        getPortal().setProcessorEnabled(visionProcessor, false);
        getPortal().setProcessorEnabled(aprilTag, true);
    }
    public void closeCamera(){
        visionPortal.close();
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    //GENERAL ROBOT ACTIONS ------------------------------------------------------------------------
    public class Home implements Action{ //pull in everything together to home pos
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extend.setPower($.EXT_PWR);
            extend.setTargetPosition($.EXT_HOME);
            climbl.setTargetPosition($.CLIMB_HOME);
            climbr.setTargetPosition($.CLIMB_HOME);
            score.setPosition($.SCORE_HOME);
            finger.setPosition($.FINGER_IN);
            return false;
        }
    }
    public Action home(){
        return new Home();
    }

    public class Retract implements Action{ //pulls in outtake only
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower($.EXT_PWR);
            extend.setTargetPosition($.EXT_RETRACT);
            finger.setPosition($.FINGER_IN);
            return false;
        }
    }
    public Action retract(){
        return new Retract();
    }
    private void Retract(){
        extend.setPower($.EXT_PWR);
        extend.setTargetPosition($.EXT_RETRACT);
        finger.setPosition($.FINGER_IN);
    }

    //INTAKE ACTIONS -------------------------------------------------------------------------------
    public class IntakeStop implements Action{ //tell intake motor to stop spinning and fix extend motor
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            extend.setTargetPosition($.HOME);
//            extend.setPower($.EXT_PWR);
            intake.setPower($.ZERO); //stop intake
            return false;
        }
    }
    public Action intakeStop() {
        return new IntakeStop();
    }

    public class IntakeGround implements Action{ //place intake on ground to feed
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            climbl.setTargetPosition($.CLIMB_INT_GND);
            climbr.setTargetPosition($.CLIMB_INT_GND);
            wrist.setPosition($.WRIST_GND);
            intake.setPower($.INTAKE_PWR); //turn intake on full speed
            return false;
        }
    }

    public Action intakeGround() {
        return new IntakeGround();
    }

    public class IntakeLevel3 implements Action{ //place intake on ground to feed
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            climbl.setTargetPosition($.CLIMB_INT_LVL_3);
            climbr.setTargetPosition($.CLIMB_INT_LVL_3);
            wrist.setPosition($.WRIST_GND);
            intake.setPower($.INTAKE_PWR); //turn intake on full speed
            return false;
        }
    }
    public Action intakeLevel3() {
        return new IntakeLevel3();
    }

    public class IntakeLevel5 implements Action{ //drop intake to top of pixel stack
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            climbl.setTargetPosition($.CLIMB_INT_LVL_5);
            climbr.setTargetPosition($.CLIMB_INT_LVL_5);
            wrist.setPosition($.WRIST_LVL_5);
            intake.setPower($.INTAKE_PWR); //turn intake on full speed
            return false;
        }
    }
    public Action intakeLevel5() {
        return new IntakeLevel5();
    }

    public class IntakeUp implements Action{ //brings intake up and gets the climber motors in pos to transfer
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            score.setPosition($.SCORE_TRANSFER);
            intake.setPower($.ZERO);
            climbl.setTargetPosition($.HOME);
            climbr.setTargetPosition($.HOME);
            climbl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            climbr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            climbl.setPower($.FULL_PWR);
            climbr.setPower($.FULL_PWR);
            extend.setTargetPosition($.EXT_HOME);
            wrist.setPosition($.WRIST_IN);
            return false;
        }
    }

    public Action intakeUp(){return new IntakeUp();}

    public boolean leftSensed(){
        return ldistance.getDistance(DistanceUnit.CM) < 1.3;
    } //sensor on intake left
    public boolean rightSensed(){
        return rdistance.getDistance(DistanceUnit.CM) < 1.3;
    } //sensor on intake right
    public boolean intakeFull(){
        return leftSensed()&rightSensed();
    } //if both sensors true

    //TRANSFER ACTIONS -----------------------------------------------------------------------------
    public class TransferA implements Action{ //intake up in a wrapper for first transfer step

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Actions.runBlocking(new SequentialAction(
                    intakeUp()
            ));
            return false;
        }
    }
    public Action transferA(){return new TransferA();}

    public class TransferB implements Action{ //pull in score bucket all the way final step

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setTargetPosition($.HOME);
            extend.setPower($.EXT_PWR);
            score.setPosition($.SCORE_HOME);
            return false;
        }
    }
    public Action transferB(){return new TransferB();}

    public class TransferC implements Action{ //spit pixels out into scoring bucket last step
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower($.FULL_PWR_INV);
            return false;
        }
    }
    public Action transferC(){return new TransferC();}

    //PRELOAD ACTIONS ------------------------------------------------------------------------------
    public class SpikeExtend implements Action { //extend outtake to spike mark pos
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            finger.setPosition($.FINGER_GND);
            extend.setPower($.EXT_PWR);
            extend.setTargetPosition($.EXT_PRELOAD);
            return false;
        }
    }
    public Action spikeExtend(){
        return new SpikeExtend();
    }

    public class SpikeScore implements Action { // lift climber motors to clear finger pixel
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            finger.setPosition($.FINGER_GND);
            climbl.setTargetPosition($.CLIMB_LOW);
            climbr.setTargetPosition($.CLIMB_LOW);
            return false;
        }
    }

    public Action spikeScore() {
        return new SpikeScore();
    }

    public class FingerHome implements Action{ //brings finger in before going to home pos
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            finger.setPosition($.FINGER_IN);
            return false;
        }
    }
    public Action fingerHome(){
        return new FingerHome();
    }

    //BACKDROP SCORING ACTIONS ---------------------------------------------------------------------
    public class Low implements Action { // extend to low target on backdrop to score
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower($.EXT_PWR);
            extend.setTargetPosition($.EXT_LOW);

            climbl.setTargetPosition($.CLIMB_LOW);
            climbr.setTargetPosition($.CLIMB_LOW);

            score.setPosition($.SCORE_LOW);
            return false;
        }
    }
    public Action low(){
        return new Low();
    }
    public class Mid implements Action{ // extend to medium target on backdrop to score
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower($.EXT_PWR);
            extend.setTargetPosition($.EXT_MID);

            climbl.setTargetPosition($.CLIMB_MID);
            climbr.setTargetPosition($.CLIMB_MID);

            score.setPosition($.SCORE_MID);
            return false;
        }
    }
    public Action mid(){
        return new Mid();
    }

    public class High implements Action{ // extend to high target on backdrop to score
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower($.EXT_PWR);
            extend.setTargetPosition($.EXT_HIGH);
            climbl.setTargetPosition($.CLIMB_HIGH);
            climbr.setTargetPosition($.CLIMB_HIGH);
            score.setPosition($.SCORE_HIGH);
            return false;
        }
    }
    public Action high(){return new High();}

    //MISC ACTIONS ---------------------------------------------------------------------------------
    public void sendTelemetry(String Key, Object Value){
        TelemetryPacket packet = new TelemetryPacket();
        packet.put(Key,Value);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    public void newLine(){
        sendTelemetry("\u200B","\u200B");
    }

    public static double Tiles(double amt_of_tiles){
        return (double) amt_of_tiles*24;
    }
}