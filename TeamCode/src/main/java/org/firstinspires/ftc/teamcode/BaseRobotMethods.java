package org.firstinspires.ftc.teamcode;
import android.util.Size;

import androidx.annotation.NonNull;

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
    ElapsedTime timer = new ElapsedTime(); //not currently used
    boolean holdWrist = false;
    boolean resetEncoder = false;

    private AprilTagProcessor aprilTag;

    public FirstVisionProcessor visionProcessor;

    private VisionPortal visionPortal;
    private VisionPortal visionPortalBack;

    private VisionPortal visionPortalApril;

    DcMotorEx intake, climbl, climbr, extend, fl, fr, bl, br;
    Servo wrist, elbowl, elbowr, lhook, rhook, score, finger;
    private ElapsedTime runtime = new ElapsedTime();
    boolean passiveIntake = false;
    TouchSensor elevatorLimit;
    private int timeout = 7;
    double scoreHome = 0.95;
    public LinearOpMode parent;
    public Telemetry telemetry;

    static final double COUNTS_PER_MOTOR_REV    = 1440 ;
    public double elbowhome = 0.25;// eg: TETRIX Motor Encoder
    public double elbowHome = 0.0;
    static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)) / 3.5625;

    //HARDWARE SETUP-------------------------------------------------------------------------------------------------
    public BaseRobotMethods(HardwareMap hardwareMap) {// init all hardware

        visionProcessor = new FirstVisionProcessor();

        //MOTORS INIT
        elevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorLimit");
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

        climbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //SERVOS INIT
        elbowl = hardwareMap.get(Servo.class, "larm");
        elbowr = hardwareMap.get(Servo.class, "rarm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        lhook = hardwareMap.get(Servo.class, "lhook");
        rhook = hardwareMap.get(Servo.class, "rhook");
        score = hardwareMap.get(Servo.class, "score");
        finger = hardwareMap.get(Servo.class, "finger");

        //CAMERA INIT
        initCamera(hardwareMap);
    }

    //CAMERA COMMANDS ----------------------------------------------------------------------------------------------------
    public void initCamera(HardwareMap hardwareMap){
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcamback"))
                .addProcessor(visionProcessor)
                .setCameraResolution(new Size(640, 360)) // 544 288
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();
    }
    //MOTOR COMMANDS-------------------------------------------------------------------------------------------------------
    public void setMotorMode(DcMotor.RunMode mode){
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }
    public void setMotorPosition(int pos1, int pos2, int pos3, int pos4){
        fl.setTargetPosition(pos1); //set encoder ticks target
        fr.setTargetPosition(pos2);
        bl.setTargetPosition(pos3);
        br.setTargetPosition(pos4);
    }
    public void setMotorPower(double speed1, double speed2, double speed3, double speed4){
        fl.setPower(speed1); //set motor power target
        fr.setPower(speed2);
        bl.setPower(speed3);
        br.setPower(speed4);

        //run motors until one of them stops
        while(parent.opModeIsActive() &&  (runtime.seconds() < timeout) && (fl.isBusy()
                && fr.isBusy() && bl.isBusy() && br.isBusy())){

            telemetry.addData("encoder-fwd-left", fl.getCurrentPosition() + "busy=" + fl.isBusy());
            telemetry.addData("encoder-fwd-right", fr.getCurrentPosition() + "busy=" + fr.isBusy());
            telemetry.addData("encoder-bkw-left", bl.getCurrentPosition() + "busy=" + bl.isBusy());
            telemetry.addData("encoder-bkw-right", br.getCurrentPosition() + "busy=" + br.isBusy());
            telemetry.update();
        }

        stopMovement();
        runtime.reset();
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

    public class GroundWrist implements Action{

        public Action init(){
            elbowl.setPosition(0.975);//  INTAKE DOWN and TURN ON
            elbowr.setPosition(0.325);
            wrist.setPosition(0.35);

            intake.setPower(-1.0); //turn intake on full speed

            passiveIntake = true;
            timer.reset();
            holdWrist = false;
            return new GroundWrist();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
    public Action groundWrist() {
        return new GroundWrist().init();
    }

    public void initPos(){
        score.setPosition(scoreHome);
        elbowl.setPosition(elbowhome + 0.3);//  INTAKE UP // Transfer
        elbowr.setPosition((1 - elbowhome));
    }

    public class Transfer implements Action{
        public Action init(){
            while(!elevatorLimit.isPressed())
            {
                extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extend.setPower(-0.5);//set 50% speed elevator in
            }
            //then transfer and reset elevator encoder
            intake.setPower(0.75);

            extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extend.setTargetPosition(0);
            extend.setPower(1.0);
            extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            return new Transfer();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }

        //wait until elevator limit switch is pressed

    }
    public Action transfer(){return new Transfer().init();}
    public class IntakeDown implements Action{
        public Action init(){
            runtime.reset();
            double elbowHome = 0.0;
            elbowl.setPosition(elbowHome + .32);//  INTAKE UP // Transfer
            elbowr.setPosition((.28 + elbowHome));

            wrist.setPosition(0.33);

            while(runtime.seconds()<0.5)
            {

                //elbowHome = Math.min(runtime.seconds()*elbowHome*2+elbowHome,0.68);

                elbowl.setPosition(Math.min(runtime.seconds()*elbowHome*4+elbowHome,0.27));
                elbowr.setPosition(Math.min(runtime.seconds()*elbowHome*4+elbowHome,0.27));
            }
            return new IntakeDown();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
        public Action intakeDown(){return new IntakeDown().init();}

    }

    public class IntakeUp implements Action{
        public Action init(){
            elbowl.setPosition(.32);//  INTAKE UP // Transfer
            elbowr.setPosition(.28);
            wrist.setPosition(0.4);
            return new IntakeUp();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }


    }
    public Action intakeUp(){return new IntakeUp().init();}

    public class SpikeExtend implements Action {
        public Action init(int extendTicks) {
            extend.setPower(1);
            extend.setTargetPosition(extendTicks);
            while (extend.getCurrentPosition() > 50) {
                finger.setPosition(-0.000235294 * extend.getCurrentPosition() + 0.37);
            }
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return new SpikeExtend();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
        // (int extendticks)
    }
    public Action spikeExtend( int extendticks){
        return new SpikeExtend().init(extendticks);
    }
    public class Retract implements Action {
        public Action init(double power) {
            int clpos = climbl.getCurrentPosition();
            int crpos = climbr.getCurrentPosition();
            climbl.setTargetPosition(clpos+240);
            climbr.setTargetPosition(crpos+240);
            //   extend.setTargetPosition(0);
            climbl.setPower(power);
            climbr.setPower(power);
            extend.setPower(power);
            climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            new SleepAction(0.35);
            finger.setPosition(0.84);
            extend.setTargetPosition(0);
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbl.setTargetPosition(0);
            climbr.setTargetPosition(0);
            climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            score.setPosition(0.95);
            return new Retract();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
         // (double power)
    }
    public Action retract(double power){
        return new Retract().init(power);
    }
    public class Extend implements Action{ //int extendticks
        public Action init(int extendticks) {
            extend.setTargetPosition(extendticks);
            climbl.setTargetPosition(climbl.getCurrentPosition() + 80);
            climbr.setTargetPosition(climbr.getCurrentPosition() + 80);
            climbl.setPower(1);
            climbr.setPower(1);
            extend.setPower(1);
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            new SleepAction(1);
            extend.setPower(0);
            climbl.setPower(0);
            climbr.setPower(0);
            return new Extend();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }

    }
    public Action extend(int extendticks){
        return new Extend().init(extendticks);
    }

    public class Low implements Action { //(int extendTarget)
        public Action init(int extendTarget) {
            extend.setTargetPosition(extendTarget);
            climbl.setTargetPosition(260);
            climbr.setTargetPosition(260);
            score.setPosition(0.38);
            extend.setPower(1);
            climbl.setPower(1);
            climbr.setPower(1);
            score.setPosition(0.38);
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return new Low();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
    public Action low(int extendTarget){
        return new Low().init(extendTarget);
    }

    public class Mid implements Action{ //(int extendTarget)
        public Action init(int extendTarget) {
            extend.setTargetPosition(extendTarget);
            climbl.setTargetPosition(525);
            climbr.setTargetPosition(525);
            score.setPosition(0.31);
            extend.setPower(1);
            climbl.setPower(1);
            climbr.setPower(1);
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return new Mid();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }


    }
    public Action mid(int extendTarget){
        return new Mid().init(extendTarget);
    }


    public class Home implements Action{ //(double power)
        public Action init(double power) {
            while(!elevatorLimit.isPressed())
            {
                extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extend.setPower(-0.5);//set 50% speed elevator in
            }

            extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extend.setPower(0);
            climbl.setTargetPosition(0);
            climbr.setTargetPosition(0);
            score.setPosition(0.93);
            climbl.setPower(power);
            climbr.setPower(power);
            climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            new SleepAction(0.35);
            climbl.setPower(0);
            climbr.setPower(0);
            elbowl.setPosition(0.58 - elbowHome);//  INTAKE UP // Transfer
            elbowr.setPosition(0.36 + elbowHome);
            return new Home();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }

    }
    public Action home(double power){
        return new Home().init(power);
    }

    public class IntakeToPos implements Action {
        public Action init(int pos){
            if (pos == 5) {
                wrist.setPosition(0.485);
                elbowl.setPosition(0.95);
                elbowr.setPosition(0.35);
            } else if (pos == 4) {
                wrist.setPosition(0.455);
                elbowl.setPosition(0.965);
                elbowr.setPosition(0.335);
            } else if (pos == 3) {
                wrist.setPosition(0.475);
                elbowl.setPosition(0.965);
                elbowr.setPosition(0.335);
            } else if (pos == 2) {
                wrist.setPosition(0.42);
                elbowl.setPosition(0.965);
                elbowr.setPosition(0.335);
            }
            return new IntakeToPos();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
    public Action intakeToPos(int pos){
        return new IntakeToPos().init(pos);
    }
    public static double Tiles(double amt_of_tiles){
        return (double) amt_of_tiles*24;
    }
}
