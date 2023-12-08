
package org.firstinspires.ftc.teamcode;
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
public class BaseRobotMethods{
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
    DcMotorEx intake, climbl, climbr, extend, fl, fr, bl, br;
    Servo wrist, elbowl, elbowr, lhook, rhook, score, finger, droneLaunch;
    private ElapsedTime runtime = new ElapsedTime();
    boolean passiveIntake = false;
    TouchSensor elevatorLimit;
    private int timeout = 7;
    double scoreHome = 0.95;
    public LinearOpMode parent;
    public Telemetry telemetry;
    public double elbowHome = 0.0;
    //HARDWARE SETUP-------------------------------------------------------------------------------------------------
    public BaseRobotMethods(HardwareMap hardwareMap) {
        //MOTORS INIT
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        climbl = hardwareMap.get(DcMotorEx.class, "lclimb");
        climbr = hardwareMap.get(DcMotorEx.class, "rclimb");
        extend = hardwareMap.get(DcMotorEx.class, "extend");

        fl = hardwareMap.get(DcMotorEx.class, "lf");
        fr = hardwareMap.get(DcMotorEx.class, "rf");
        bl = hardwareMap.get(DcMotorEx.class, "lb");
        br = hardwareMap.get(DcMotorEx.class, "rb");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extend.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extend.setTargetPosition(0);
        extend.setPower(0.0);
        extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extend.setTargetPosition(0); //it doesn't crash when we keep this in so idk
        extend.setVelocityPIDFCoefficients(25.0,0.0,0.0,0.0);
        extend.setPositionPIDFCoefficients(25.0);
        extend.setPower(1.0);

        climbl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climbr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climbl.setPower(0.0);
        climbr.setPower(0.0);
        climbl.setTargetPosition(0);
        climbr.setTargetPosition(0);
        climbl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climbr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climbl.setPower(1.0);
        climbr.setPower(1.0);

        elbowl = hardwareMap.get(Servo.class, "larm");
        elbowr = hardwareMap.get(Servo.class, "rarm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        lhook = hardwareMap.get(Servo.class, "lhook");
        rhook = hardwareMap.get(Servo.class, "rhook");
        score = hardwareMap.get(Servo.class, "score");
        droneLaunch = hardwareMap.get(Servo.class, "launch");
        finger = hardwareMap.get(Servo.class, "finger");
        elevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorLimit");

        score.setPosition(scoreHome);

        //SERVO POS
        elbowl.setPosition(0.55 + elbowHome);
        elbowr.setPosition(0.30 + elbowHome);
        //wrist.setPosition(0.725);
        finger.setPosition(0.825);
        droneLaunch.setPosition(0.3);


        //CAMERA INIT
        visionProcessor = new FirstVisionProcessor();
        initCamera(hardwareMap);
    }
    //CAMERA COMMANDS ----------------------------------------------------------------------------------------------------
    public void initCamera(HardwareMap hardwareMap){
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcamback"))
                .addProcessor(visionProcessor)
                .setCameraResolution(new Size(864, 480)) // 544 288
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();
    }
    public void stopMovement(){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
    public class Home implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower(1);
            extend.setTargetPosition(0);
            climbl.setTargetPosition(5);
            climbr.setTargetPosition(5);
            score.setPosition(scoreHome);
            finger.setPosition(0.84);
            return false;
        }
    }
    public Action home(){
        return new Home();
    }

    public class Retract implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower(0.75);
            extend.setTargetPosition(75);
            climbl.setTargetPosition(450);
            climbr.setTargetPosition(450);
            score.setPosition(scoreHome);
            finger.setPosition(0.84);
            return false;
        }
    }
    public Action retract(){
        return new Retract();
    }


    public class IntakeStop implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(0); //stop intake
            return false;
        }
    }
    public Action intakeStop() {
        return new IntakeStop();
    }

    public class IntakeGround implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            elbowl.setPosition(0.786);//  INTAKE DOWN and TURN ON
            elbowr.setPosition(0.536);
            wrist.setPosition(0.343);
            intake.setPower(-1.0); //turn intake on full speed
            passiveIntake = true;
            return false;
        }
    }
    public Action intakeGround() {
        return new IntakeGround();
    }
    public class IntakeLevel5 implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            elbowl.setPosition(0.4484);//  INTAKE DOWN and TURN ON
            elbowr.setPosition(0.777);
            wrist.setPosition(0.5273);
            intake.setPower(-1.0); //turn intake on full speed
            passiveIntake = true;
            return false;
        }
    }
    public Action intakeLevel5() {
        return new IntakeLevel5();
    }
    public class Transfer implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extend.setPower(-0.5);//set 50% speed elevator in
            //then transfer and reset elevator encoder
            intake.setPower(0.75);
            extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extend.setTargetPosition(0);
            extend.setPower(1.0);
            extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action transfer(){return new Transfer();}
    //wait until elevator limit switch is pressed
    public class IntakeUp implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            elbowl.setPosition(0.55 + elbowHome);
            elbowr.setPosition(0.30 + elbowHome);
            wrist.setPosition(0.83);
            return false;
        }
    }
    public Action intakeUp(){return new IntakeUp();}
    public class SpikeExtend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setTargetPosition(300);
            while (extend.isBusy()) {
                finger.setPosition(-0.00007 * extend.getCurrentPosition() + 0.44);
            }
            return false;
        }
        // (int extendticks)
    }
    public Action spikeExtend(){
        return new SpikeExtend();
    }
    public class SpikeScore implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            climbl.setTargetPosition(300);
            climbr.setTargetPosition(300);
            return false;
        }
    }
    public Action spikeScore(){
        return new SpikeScore();
    }
    public class FingerHome implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            finger.setPosition(0.825);
            return false;
        }
    }
    public Action fingerHome(){
        return new FingerHome();
    }
    public class Low implements Action { //(int extendTarget)
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower(0.65);
            extend.setTargetPosition(0);
            climbl.setTargetPosition(0); //260
            climbr.setTargetPosition(0);
            score.setPosition(scoreHome);
            return false;
        }
    }
    public Action low(){
        return new Low();
    }
    public class Mid implements Action{ //(int extendTarget)
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower(0.65);
            extend.setTargetPosition(450);
            climbl.setTargetPosition(400); //525
            climbr.setTargetPosition(400);
            score.setPosition(0.34);
            return false;
        }
    }
    public Action mid(){
        return new Mid();
    }
    public static double Tiles(double amt_of_tiles){
        return (double) amt_of_tiles*24;
    }
}