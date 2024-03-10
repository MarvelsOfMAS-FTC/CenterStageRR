package org.firstinspires.ftc.teamcode;
import android.util.Size;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
public class BaseRobotMethods{
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    BaseConstants.Params $ = new BaseConstants.Params();
    private AprilTagProcessor aprilTag;
    public FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    DcMotorEx intake, climbl, climbr, extend, fl, fr, bl, br;
    Servo wrist, lhook, rhook, droneLaunch, droneFlip, score, finger;
    private NormalizedColorSensor lcolor;
    private NormalizedColorSensor rcolor;
    private DistanceSensor ldistance;
    private DistanceSensor rdistance;
    
    boolean passiveIntake = false;
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
        extend.setTargetPosition($.HOME);
        extend.setPower($.ZERO);
        extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extend.setTargetPosition($.HOME); //it doesn't crash when we keep this in so idk
        extend.setVelocityPIDFCoefficients(25.0,0.0,0.0,0.0);
        extend.setPositionPIDFCoefficients(25.0);
        extend.setPower($.FULL_PWR);

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
        bl.setPower($.ZERO);
        fl.setPower($.ZERO);
        fr.setPower($.ZERO);
        br.setPower($.ZERO);
    }
    public class StopMotor implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            fr.setMotorDisable();
            fl.setMotorDisable();
            br.setMotorDisable();
            bl.setMotorDisable();
            return false;
        }
    }
    public Action stopMotors(){return new StopMotor();}
    public class Home implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower($.FULL_PWR);
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

    public class Retract implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower($.EXT_PWR_OUT);
            extend.setTargetPosition($.EXT_RETRACT);
            finger.setPosition($.FINGER_IN);
            return false;
        }
    }
    public Action retract(){
        return new Retract();
    }
    private void Retract(){
        extend.setPower($.EXT_PWR_OUT);
        extend.setTargetPosition($.EXT_RETRACT);

        finger.setPosition($.FINGER_IN);
    }

    public class IntakeStop implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower($.ZERO); //stop intake
            return false;
        }
    }
    public Action intakeStop() {
        return new IntakeStop();
    }

    public class IntakeGround implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            climbl.setTargetPosition($.CLIMB_INT_GND);
            climbr.setTargetPosition($.CLIMB_INT_GND);
            wrist.setPosition($.WRIST_GND);
            intake.setPower($.FULL_PWR_INV); //turn intake on full speed
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
            climbl.setTargetPosition($.CLIMB_INT_LVL_5);
            climbr.setTargetPosition($.CLIMB_INT_LVL_5);
            wrist.setPosition($.WRIST_LVL_5);
            intake.setPower($.FULL_PWR_INV); //turn intake on full speed
            passiveIntake = true;
            return false;
        }
    }
    public Action intakeLevel5() {
        return new IntakeLevel5();
    }


    public class IntakeLevel3 implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            climbl.setTargetPosition($.CLIMB_INT_LVL_3);
            climbr.setTargetPosition($.CLIMB_INT_LVL_3);
            wrist.setPosition(0.395);
            intake.setPower($.FULL_PWR_INV); //turn intake on full speed
            passiveIntake = true;
            return false;
        }
    }
    public Action intakeLevel3() {
        return new IntakeLevel3();
    }

    /*
     * Deprecated use IntakeUp instead...
     */
    public class Transfer implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //then transfer and reset elevator encoder
            //intake.setPower($.FULL_PWR);
            IntakeUp();
            return false;
        }
    }

    public Action transfer(){return new Transfer();}
    //wait until elevator limit switch is pressed
    public class IntakeUp implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            //wait until elevator limit switch is pressed
            Retract();
            //then transfer and reset elevator encoder
            intake.setPower($.WRIST_TRANSFER_PWR);
            wrist.setPosition($.WRIST_IN);
            score.setPosition($.SCORE_HOME);
            extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extend.setTargetPosition($.EXT_HOME);
            extend.setPower($.FULL_PWR);
            extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    private void IntakeUp(){
        Retract();
        //then transfer and reset elevator encoder
        intake.setPower($.WRIST_TRANSFER_PWR);
        wrist.setPosition($.WRIST_IN);

        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setTargetPosition($.EXT_HOME);
        extend.setPower($.FULL_PWR);
        extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public Action intakeUp(){return new IntakeUp();}

    public class SpikeExtend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower($.EXT_PWR_OUT);
            extend.setTargetPosition($.EXT_PRELOAD);
            return false;
        }
    }
    public Action spikeExtend(){
        return new SpikeExtend();
    }
    public class SpikeScore implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            climbl.setTargetPosition($.CLIMB_LOW);
            climbr.setTargetPosition($.CLIMB_LOW);
            return false;
        }
    }

    public Action spikeScore() {
        return new SpikeScore();
    }

    public Action spikeUp() {
        return new SpikeUp();
    }

    public class SpikeUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            finger.setPosition($.FINGER_GND);

            climbl.setTargetPosition(($.CLIMB_LOW));
            climbr.setTargetPosition(($.CLIMB_LOW));
            return false;
        }
    }

    public class FingerHome implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            finger.setPosition($.FINGER_IN);
            return false;
        }
    }
    public Action fingerHome(){
        return new FingerHome();
    }
    public class Low implements Action { //(int extendTarget)
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower($.EXT_PWR_OUT);
            extend.setTargetPosition($.EXT_LOW);
            climbl.setTargetPosition($.CLIMB_LOW);
            climbr.setTargetPosition($.CLIMB_LOW);
            score.setPosition($.SCORE_BACKDROP);
            return false;
        }
    }
    public Action low(){
        return new Low();
    }
    public class Mid implements Action{ //(int extendTarget)
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            extend.setPower($.FULL_PWR);
            extend.setTargetPosition($.EXT_MID);
            climbl.setTargetPosition($.CLIMB_MID);
            climbr.setTargetPosition($.CLIMB_MID);
            score.setPosition($.SCORE_BACKDROP);
            return false;
        }
    }
    public Action mid(){
        return new Mid();
    }

    public class High implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setPower($.FULL_PWR);
            extend.setTargetPosition($.EXT_HIGH);
            climbl.setTargetPosition($.CLIMB_HIGH);
            climbr.setTargetPosition($.CLIMB_HIGH);
            score.setPosition($.SCORE_BACKDROP);
            return false;
        }
    }
    public Action High(){return new High();}

    public static double Tiles(double amt_of_tiles){
        return (double) amt_of_tiles*24;
    }
}