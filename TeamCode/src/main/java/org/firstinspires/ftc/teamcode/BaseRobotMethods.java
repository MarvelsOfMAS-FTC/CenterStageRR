package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BaseRobotMethods {

    //VARIABLES---------------------------------------------------------------------------------------------------------------
    private ElapsedTime     runtime = new ElapsedTime();

    public DcMotorEx arm;
    public DcMotorEx climbl;
    public DcMotorEx climbr;
    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;

    public DcMotorEx intake;

    int climbTarget = 5;

    public Servo score;
    public Servo elbowl;
    public Servo elbowr;
    public Servo wrist;
    public Servo launch;
    TouchSensor elevatorLimit; //elevator limit switch used to reset elevator encoder


  //  public LinearOpMode parent;
   // public Telemetry telemetry;
     private int timeout = 7;

     public double elbowhome = 0.25;

    double scoreHome = 0.95; //home position for scoring mec

    boolean passiveIntake = false; //if true then the intake should run in (to keep game element inside intake)
    ElapsedTime timer = new ElapsedTime(); //not currently used
    boolean holdWrist = false;
    boolean resetEncoder = false; //this does the same thing as resetElevatorEncoder but while transfering, yeah sorry :(




    //HARDWARE SETUP-------------------------------------------------------------------------------------------------
    public BaseRobotMethods(HardwareMap hardwareMap) {// init all hardware
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

    public void transfer(){
        elbowl.setPosition(0.0);
        elbowr.setPosition(0.0);
        intake.setPower(-1);
    }
    public void home(){
        elbowl.setPosition(elbowhome + 0.3);//  INTAKE UP // Transfer
        elbowr.setPosition((1 - elbowhome));
        score.setPosition(scoreHome);
        wrist.setPosition(0.84);
        if(elevatorLimit.isPressed()&&timer.milliseconds()>700)
        {
            intake.setPower(.75);
        }
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(-0.5);//set 50% speed elevator in
        resetEncoder = true;
        elbowhome = (0.26 + ((Math.max(climbl.getTargetPosition(),climbl.getCurrentPosition()) / 1925.0) * 0.4));//set elbow position based on climb position
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
}
