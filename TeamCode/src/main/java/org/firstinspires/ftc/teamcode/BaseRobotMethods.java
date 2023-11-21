package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class BaseRobotMethods extends LinearOpMode{

    //VARIABLES---------------------------------------------------------------------------------------------------------------
    private ElapsedTime     runtime = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;

    private FirstVisionProcessor visionProcessor;

    private VisionPortal visionPortal;
    private VisionPortal visionPortalBack;

    private VisionPortal visionPortalApril;

    public DcMotorEx arm;

    static final double COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)) / 3.5625;
    public DcMotorEx climbl;
    public DcMotorEx climbr;
    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;
    public DcMotorEx extend;

    public DcMotorEx intake;

    public LinearOpMode parent;
    public Telemetry telemetry;

    int climbTarget = 5;

    public Servo score;
    public Servo elbowl;
    public Servo elbowr;
    public Servo wrist;
    public Servo launch;
    public String placement = "None";
    TouchSensor elevatorLimit; //elevator limit switch used to reset elevator encoder

     private int timeout = 7;

     public double elbowhome = 0.25;

    double scoreHome = 0.95; //home position for scoring mec

    boolean passiveIntake = false; //if true then the intake should run in (to keep game element inside intake)
    ElapsedTime timer = new ElapsedTime(); //not currently used
    boolean holdWrist = false;
    boolean resetEncoder = false; //this does the same thing as resetElevatorEncoder but while transfering, yeah sorry :(


    //HARDWARE SETUP-------------------------------------------------------------------------------------------------
    public BaseRobotMethods(HardwareMap hardwareMap) {// init all hardware
        visionProcessor = new FirstVisionProcessor();


        fl = hardwareMap.get(DcMotorEx.class, "lf");
        fr = hardwareMap.get(DcMotorEx.class, "rf");
        bl = hardwareMap.get(DcMotorEx.class, "lb");
        br = hardwareMap.get(DcMotorEx.class, "rb");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        climbl = hardwareMap.get(DcMotorEx.class, "climbl");
        climbr = hardwareMap.get(DcMotorEx.class, "climbr");
        intake = hardwareMap.get(DcMotorEx.class, "intake");


        score = hardwareMap.get(Servo.class, "score");
        elbowl = hardwareMap.get(Servo.class, "elbowl");
        elbowr = hardwareMap.get(Servo.class, "elbowr");
        wrist = hardwareMap.get(Servo.class, "wrist");
        launch = hardwareMap.get(Servo.class, "launch");


        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(0);

        arm.setTargetPosition(0);
        arm.setPower(1.0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(0); //it doesn't crash when we keep this in so idk

        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);

        climbl.setPower(1.0);
        climbr.setPower(1.0);
        climbl.setTargetPosition(5);
        climbr.setTargetPosition(5);
        climbTarget = 5;
        climbl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climbr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //CAMERA COMMANDS ----------------------------------------------------------------------------------------------------
    public void initCamera(){
        // visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcamfront"))
                .addProcessor(visionProcessor)
                .setCameraResolution(new Size(544, 288))
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

    //ARM COMMANDS--------------------------------------------------------------------------------------------
    public void moveArm(int distance, double power){
        arm.setTargetPosition(distance);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);

    }

    public void stopArm(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(0);
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

    public void low() {
        extend.setTargetPosition(350);
        climbl.setTargetPosition(260);
        climbr.setTargetPosition(260);
        score.setPosition(0.38);
    }

    public void mid() {
        extend.setTargetPosition(450);
        climbl.setTargetPosition(525);
        climbr.setTargetPosition(525);
        score.setPosition(0.31);
    }

    public void home() {
        extend.setTargetPosition(0);
        climbl.setTargetPosition(0);
        climbr.setTargetPosition(0);
        score.setPosition(0.93);
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
    public void runOpMode() throws InterruptedException {

    }
}
