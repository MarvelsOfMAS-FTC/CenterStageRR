package org.firstinspires.ftc.teamcode;
import android.util.Size;

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


public class BaseRobotMethods extends LinearOpMode{

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


        //grab motors
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

        //grab servos
        elbowl = hardwareMap.get(Servo.class, "larm");
        elbowr = hardwareMap.get(Servo.class, "rarm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        lhook = hardwareMap.get(Servo.class, "lhook");
        rhook = hardwareMap.get(Servo.class, "rhook");
        score = hardwareMap.get(Servo.class, "score");
        finger = hardwareMap.get(Servo.class, "Finger");

        //set intake position
        double elbowHome = (0.0);
        elbowl.setPosition(elbowHome + .32);//  INTAKE UP // Transfer
        elbowr.setPosition((.28 + elbowHome));
        wrist.setPosition(0.28); //0.75
        //initCamera(hardwareMap);

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

    public void groundWrist(){
        elbowl.setPosition(0.975);//  INTAKE DOWN and TURN ON
        elbowr.setPosition(0.325);
        wrist.setPosition(0.35);

        intake.setPower(-1.0); //turn intake on full speed

        passiveIntake = true;
        timer.reset();
        holdWrist = false;
    }

    public void initPos(){
        score.setPosition(scoreHome);

        elbowl.setPosition(elbowhome + 0.3);//  INTAKE UP // Transfer
        elbowr.setPosition((1 - elbowhome));
    }
    public static double Tiles(double amt_of_tiles){
        return (double) amt_of_tiles*24;
    }
    public void transfer(){

        //wait until elevator limit switch is pressed
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
    }
    public void intakeDown(){
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
    }

    public void intakeUp(){
        elbowl.setPosition(.32);//  INTAKE UP // Transfer
        elbowr.setPosition(.28);
        wrist.setPosition(0.4);
    }
    public void spikeExtend(int extendticks) {
        extend. setTargetPosition(extendticks);
    }
    public void extend(int extendticks) {
        extend.setTargetPosition(extendticks);
        climbl.setTargetPosition(climbl.getCurrentPosition()+80);
        climbr.setTargetPosition(climbr.getCurrentPosition()+80);
    }
    public void low() {
        extend.setTargetPosition(350);
        climbl.setTargetPosition(260);
        climbr.setTargetPosition(260);
        score.setPosition(0.38);
        extend.setPower(1);
        climbl.setPower(1);
        climbr.setPower(1);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void mid() {
        extend.setTargetPosition(450);
        climbl.setTargetPosition(525);
        climbr.setTargetPosition(525);
        score.setPosition(0.31);
        extend.setPower(1);
        climbl.setPower(1);
        climbr.setPower(1);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void home(double power) {
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
        sleep(250);
        climbl.setPower(0);
        climbr.setPower(0);
        elbowl.setPosition(0.58 - elbowHome);//  INTAKE UP // Transfer
        elbowr.setPosition(0.36 + elbowHome);
    }

    public void intakeToPos(int pos) {
        if(pos == 5) {
            wrist.setPosition(0.485);
            elbowl.setPosition(0.95);
            elbowr.setPosition(0.35);
        } else if(pos == 4) {
            wrist.setPosition(0.455);
            elbowl.setPosition(0.965);
            elbowr.setPosition(0.335);
        } else if(pos == 3) {
            wrist.setPosition(0.475);
            elbowl.setPosition(0.965);
            elbowr.setPosition(0.335);
        } else if(pos == 2) {
            wrist.setPosition(0.42);
            elbowl.setPosition(0.965);
            elbowr.setPosition(0.335);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {}
}
