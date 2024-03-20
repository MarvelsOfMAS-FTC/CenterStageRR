package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import android.util.Size;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous()
public class MoMEncoderAuto extends LinearOpMode {


  double DESIRED_DISTANCE = 20.0; //  this is how close the camera should get to the target (inches)
  private int DESIRED_TAG_ID = 5; // Choose the tag you want to approach or set to -1 for ANY tag.
  private VisionPortal visionPortal; // Used to manage the video source.
  private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection process.
  private AprilTagDetection desiredTag = null; // Used to hold the data for a detected AprilTag
  static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections = RevHubOrientationOnRobot.LogoFacingDirection.values();
  static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections = RevHubOrientationOnRobot.UsbFacingDirection.values();
  static int LAST_DIRECTION = logoFacingDirections.length - 1;
  static float TRIGGER_THRESHOLD = 0.2f;
  
  double aprilTagX = 0.0;
  double aprilTagY = 0.0;
  double aprilTagXError = 999.0;
  double aprilTagYError = 999.0;
  double xPosition = 0.0;
  double oldAprilTagSum = 0.0;
  double aprilTagSum = 0.0;
  
  boolean tagChange = true;
  boolean autoAim = false;
  boolean targetFound = false;
  
  IMU imu;
  
  int logoFacingDirectionPosition;
  int usbFacingDirectionPosition;
  boolean orientationIsValid = true;

  TouchSensor elevatorLimit; //elevator limit switch used to reset elevator encoder

  private static final boolean USE_WEBCAM = true;

  private FirstVisionProcessor visionProcessor;



  DcMotorEx intake, climbl, climbr, extend, fl, fr, bl, br;
  Servo wrist, lhook, rhook, score, finger;

  public String alliance = "None";
  public String side = "None";
  public String placement = "MIDDLE"; //this is the variable for which spot the robot should score
  public int waitTime = 0;
  public boolean innerPath = false;
  public boolean firstTile = false;
  public boolean altPark = false;
  public boolean scoreYellow = true;
  private ElapsedTime runtime = new ElapsedTime();


static final double COUNTS_PER_MOTOR_REV = 1440; // eg: TETRIX Motor Encoder
static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
static final double COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
  (WHEEL_DIAMETER_INCHES * 3.1415)) / 3.5625;

@Override
public void runOpMode() {
  
  imu = hardwareMap.get(IMU.class, "imu");
  logoFacingDirectionPosition = 0; // Up
  usbFacingDirectionPosition = 2; // Forward
  updateOrientation();
  boolean justChangedLogoDirection = false;
  boolean justChangedUsbDirection = false;
  boolean targetFound = false; // Set to true when an AprilTag target is detected
  
  visionProcessor = new FirstVisionProcessor();
    
  aprilTag = new AprilTagProcessor.Builder().build();
  aprilTag.setDecimation(1);
  
  visionPortal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "webcamback"))
    .addProcessors(visionProcessor, aprilTag)
    .setCameraResolution(new Size(864, 480))
    .setStreamFormat(VisionPortal.StreamFormat.YUY2)
    .setAutoStopLiveView(true)
    .build();
      
  visionPortal.setProcessorEnabled(aprilTag, false);
  visionPortal.setProcessorEnabled(visionProcessor, true);

  //grab motors
  intake = hardwareMap.get(DcMotorEx.class, "intake");
  climbl = hardwareMap.get(DcMotorEx.class, "lclimb");
  climbr = hardwareMap.get(DcMotorEx.class, "rclimb");
  extend = hardwareMap.get(DcMotorEx.class, "extend");
  fl = hardwareMap.get(DcMotorEx.class, "lf");
  fr = hardwareMap.get(DcMotorEx.class, "rf");
  bl = hardwareMap.get(DcMotorEx.class, "lb");
  br = hardwareMap.get(DcMotorEx.class, "rb");

  imu = hardwareMap.get(IMU.class, "imu");

  elevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorLimit");

  //set brake mode for everything
  intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  climbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  climbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

  //grab servos
  wrist = hardwareMap.get(Servo.class, "wrist");
  lhook = hardwareMap.get(Servo.class, "lhook");
  rhook = hardwareMap.get(Servo.class, "rhook");
  score = hardwareMap.get(Servo.class, "score");
  finger = hardwareMap.get(Servo.class, "finger");

  //set intake position
  double scoreHome = 0.95;

  score.setPosition(scoreHome);

  //set the intake to starting position
  wrist.setPosition(0.85);

  //reverse drivetrainmotors
  fl.setDirection(DcMotorEx.Direction.REVERSE);
  bl.setDirection(DcMotorEx.Direction.REVERSE);

  //reset drivetrain encoders
  fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  //run drivetrain with encoders
  fl.setPower(0.0);
  fr.setPower(0.0);
  bl.setPower(0.0);
  br.setPower(0.0);

  fl.setTargetPosition(0);
  fr.setTargetPosition(0);
  bl.setTargetPosition(0);
  br.setTargetPosition(0);

  fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
  fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
  bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
  br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

  //extend
  extend.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
  extend.setTargetPosition(0);
  extend.setPower(1.0);
  extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

  //climb stuff
  climbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  climbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  climbl.setPower(1.0);
  climbr.setPower(1.0);
  climbl.setTargetPosition(5);
  climbr.setTargetPosition(5);
  climbl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
  climbr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
  
  sleep(1000);

  while (!opModeIsActive() && !isStopRequested()) {
    //Running pipeline
    if (gamepad1.x) {
      alliance = "Blue";
    } 
    else if (gamepad1.b) {
      alliance = "Red";
    }
  
    if (gamepad1.right_bumper) {
      side = "Far";
    } 
    else if (gamepad1.left_bumper)
    {
      side = "Close";
    }
    if(gamepad1.dpad_left) {
      firstTile = true;
      innerPath = false;
    }
    else if(gamepad1.dpad_right)
    {
      innerPath = true;
      firstTile = false;
    }
    else if (gamepad1.guide){
      innerPath = false;
      firstTile = false;
    }
    if(gamepad1.dpad_up)
    {
      waitTime = waitTime + 1;
      sleep(250);
    }
    else if (gamepad1.dpad_down)
    {
      waitTime = waitTime - 1;
      if (waitTime < 0){waitTime = 0;}
      sleep(250);
    }
    if(gamepad1.start){
      altPark = true;
    }else if(gamepad1.back){
      altPark = false;
    }

    if(gamepad1.y){
      scoreYellow = false;
    }else if(gamepad1.a){
      scoreYellow = true;
    }
    telemetry.addData("-- Black Out Auto --", "");
    telemetry.addData("Placement: ", visionProcessor.getSelection());
    telemetry.addData("Alliance: ", alliance);
    telemetry.addData("Side", side);
    telemetry.addData("", "");
    telemetry.addData("Inner Path", innerPath);
    telemetry.addData("First Tile Path: ", firstTile);
    telemetry.addData("Left Park: ", altPark);
    telemetry.addData("", "");
    telemetry.addData("WaitTime: ", waitTime);
    telemetry.addData("Score Yellow?: ", scoreYellow);
    telemetry.addData("", "");
    telemetry.addData("IMU : ",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    telemetry.addData("", "");
    
    telemetry.addData("■ for Blue Alliance", "");
    telemetry.addData("O for Red Alliance", "");
    telemetry.addData("X for Score Yellow + Purple", "");
    telemetry.addData("∆ for Score Purple Only", "");
    telemetry.addData("LB for Close Side", "");
    telemetry.addData("RB for Far Side", "");
    telemetry.addData("DPAD L for First Tile ", "");
    telemetry.addData("DPAD R for Inner Path", "");
    telemetry.addData("Home/Guide for Center Stage Path", "");
    telemetry.addData("DPAD UP for +1 Sec Wait", "");
    telemetry.addData("DPAD DOWN for -1 Sec Wait", "");
    telemetry.addData("Start for Alt Park", "");
    telemetry.addData("Back for Normal Park", "");
    telemetry.update();
  }
  
  imu.resetYaw();
  
  visionPortal.setProcessorEnabled(visionProcessor, false);
  visionPortal.setProcessorEnabled(aprilTag, true);
  

  

  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE && alliance == "Blue" && side == "Far") {
    telemetry.addLine("Running Far Blue Center");
    telemetry.update();

    drive(0.3, -11, 0, 5.0);
    drive(0.3, 0, -10, 5.0);
    scorePurple();
    drive(0.7, 0, 0, 5.0);
    if(firstTile == true){
      drive(0.7, 6, 0, 5.0);
      drive(0.7, 0, -90, 5.0);
      wrist.setPosition(0.85);
    }else if(innerPath == false){
      drive(0.7, -35, 0, 5.0);
      drive(0.7, 0, -90, 5.0);
      drive(0.7, -3.5, -90, 5.0);
    }else if (innerPath == true){
      drive(0.7, -10.25, 0, 5.0);
      drive(0.7, 0, -90, 5.0);
      wrist.setPosition(0.85);
    }
    if(scoreYellow == true){
      scoreYellow(0.0);
    }
  }
  
  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE && alliance == "Blue" && side == "Close") {
    telemetry.addLine("Running Close Blue Center");
    telemetry.update();
    
    drive(0.3, -12, 0, 5.0);
    drive(0.3, 0, -10, 5.0);
    scorePurple();
    drive(0.7, 0, -90, 5.0);
    drive(0.7, -20, -90, 5.0);
    strafe(0.7, -5, -90, 0.5);

    scoreYellow(0.0);
    
  }

  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE && alliance == "Red" && side == "Far") {
    telemetry.addLine("Running Far Red Center");
    telemetry.update();

    drive(0.3, -13, 0, 5.0);
    drive(0.3, 0, -10, 5.0);
    scorePurple();
    drive(0.7, 0, 0, 5.0);
    if (firstTile == true){
      drive(0.7, 7, 0, 5.0);
      drive(0.7, 0, 90, 5.0);
      wrist.setPosition(0.85);
    }else if(innerPath == false){
      drive(0.7, -36, 0, 5.0);
      drive(0.7, 0, 90, 5.0);
      drive(0.7, -4, 90, 5.0);  
    } else if (innerPath == true){
      drive(0.7, -10.2, 0, 5.0);
      drive(0.7, 0, 90, 5.0);
      wrist.setPosition(0.85);
    } 
    if(scoreYellow == true){
      scoreYellow(-0.5);
    }else{
      
    }
  }
  
  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE && alliance == "Red" && side == "Close") {
    telemetry.addLine("Running Close Red Center");
    telemetry.update();
    
    drive(0.5, -13, 0, 5.0);
    drive(0.5, 0, -10, 5.0);
    scorePurple();
    drive(0.9, 0, 90, 5.0);
    drive(0.9, -20, 90, 5.0);
    scoreYellow(0.0);
  }

  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT && alliance == "Blue" && side == "Far") {
    telemetry.addLine("Running Far Blue Left");
    telemetry.update();

    drive(0.3, -7, 0, 5.0);
    drive(0.3, 0, -28, 5.0);
    scorePurple();
    drive(0.7, 0, 0, 5.0);
    if(firstTile == true){
      drive(0.7, 2, 0, 5.0);
      drive(0.7, 0, -90, 5.0);
      wrist.setPosition(0.85);
    }
    else if(innerPath == false){
      drive(0.7, -43, 0, 5.0);
      drive(0.7, 0, -90, 5.0);
      drive(0.7, -6, -90, 5.0);
    } else if (innerPath == true){
      drive(0.7, -17.5, 0, 5.0);
      drive(0.7, 0, -90, 5.0);
      wrist.setPosition(0.85);
    }
    if(scoreYellow == true){
      scoreYellow(4.5);
    }else{
      
    }
  }
  
  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT && alliance == "Blue" && side == "Close") {
    telemetry.addLine("Running Close Blue Left");
    telemetry.update();
    
    drive(0.3, -6.3, 0, 5.0);
    drive(0.3, 0, -35, 5.0);
    scorePurple();
    drive(0.7, 0, -90, 5.0);
    drive(0.7, -20, -90, 5.0);
    strafe(0.7, -15, -90, 5.0);
    scoreYellow(4.5);
    
  }

  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT && alliance == "Red" && side == "Far") {
    telemetry.addLine("Running Far Red Left");
    telemetry.update();

    
    drive(0.3, -7.5, 0, 5.0);
    drive(0.3, 0, -35.5, 5.0);
    scorePurple();
    drive(0.7, 0, 0, 5.0);
    if(firstTile == true){
      drive(0.7, 4, 0, 5.0);
      drive(0.7, 0, 90, 5.0);
      wrist.setPosition(0.85);
    }else if(innerPath == false){
      drive(0.7, -40, 0, 5.0);
      drive(0.7, 0, 90, 5.0);
      drive(0.7, -5, 90, 5.0);
    } else if(innerPath == true){
      drive(0.7, -15, 0, 5.0);
      drive(0.7, 0, 90, 5.0);
      wrist.setPosition(0.85);
    }
    if(scoreYellow == true){
      scoreYellow(5.0);
    }else{
      
    }
  }
  
  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT && alliance == "Red" && side == "Close") {
    telemetry.addLine("Running Close Red Left");
    telemetry.update();
    
    drive(0.3, -7.5, 0, 5.0);
    drive(0.3, 0, -35.5, 5.0);
    scorePurple();
    drive(1.0, 0, 90, 5.0);
    drive(1.0, -20, 90, 5.0);
    strafe(1.0, 12, 90, 5.0);
    scoreYellow(5.0);
  }

  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.RIGHT && alliance == "Blue" && side == "Far") {
    telemetry.addLine("Running Far Blue Right");
    telemetry.update();

    drive(0.3, -3.5, 0, 5.0);
    drive(0.3, 0, 17.5, 5.0);
    scorePurple();
    
    drive(0.7, 0, 0, 5.0);
    if(firstTile == true){
      drive(0.7, 2, 0, 5.0);
      drive(0.7, 0, -90, 5.0);
      wrist.setPosition(0.85);
    }else if(innerPath == false){
      drive(0.7, -45.5, 0, 5.0);
      drive(0.7, 0, -90, 5.0);
      drive(0.7, -5, -90, 5.0);
    } else if(innerPath == true) {
      drive(0.7, -16, 0, 5.0);
      drive(0.7, 0, -90, 5.0);
      wrist.setPosition(0.85);
    }
    if(scoreYellow == true){
      scoreYellow(-4.5);
    }
  }
  
  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.RIGHT && alliance == "Blue" && side == "Close") {
    telemetry.addLine("Running Close Blue Right");
    telemetry.update();
    
    drive(0.3, -5, 0, 5.0);
    drive(0.3, 0, 21, 5.0);
    scorePurple();
    drive(0.7, 0, -90, 5.0);
    drive(0.7, -20, -90, 5.0);
    strafe(0.7, -27, -90, 5.0);
    scoreYellow(-4.5);
  }

  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.RIGHT && alliance == "Red" && side == "Far") {
    telemetry.addLine("Running Far Red Right");
    telemetry.update();

    drive(0.3, -8.5, 0, 5.0);
    drive(0.3, 0, 20, 5.0);
    scorePurple();
    drive(0.7, 0, 0, 5.0);
    if(firstTile == true){
      drive(0.7, 4, 0, 5.0);
      drive(0.7, 0, 90, 5.0);
      wrist.setPosition(0.85);
    }
    else if(innerPath == false) {
      drive(0.7, -41, 0, 5.0);
      drive(0.7, 0, 90, 5.0);
      drive(0.7, -3, 90, 5.0);
    } else if(innerPath == true){
      sleep(1000);
      drive(0.7, -16, 0, 5.0);
      sleep(1000);
      drive(0.7, 0, 90, 5.0);
      wrist.setPosition(0.85);
    }
    if(scoreYellow == true){
      scoreYellow(-5.0);
    }
  }
  
  if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.RIGHT && alliance == "Red" && side == "Close") {
    telemetry.addLine("Running Close Red Right");
    telemetry.update();
    
    drive(0.3, -9, 0, 5.0);
    drive(0.3, 0, 21, 5.0);
    scorePurple();
    drive(0.7, 0, 90, 5.0);
    drive(0.7, -20, 90, 5.0);
    strafe(0.7, 17, 90, 5.0);
    scoreYellow(-5.0);
  }

  telemetry.addLine("DID NOT MAKE A SELECTION");
  telemetry.update();
  requestOpModeStop();
}

public void transfer() {

  //wait until elevator limit switch is pressed
  while (!elevatorLimit.isPressed()) {
    extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    extend.setPower(-0.5); //set 50% speed elevator in
  }
  //then transfer and reset elevator encoder
  intake.setPower(0.75);
  wrist.setPosition(0.83);

  extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  extend.setTargetPosition(0);
  extend.setPower(1.0);
  extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
}


public void intakeDown() {
  //  INTAKE DOWN and TURN ON
  climbl.setTargetPosition(700);
  climbr.setTargetPosition(700);
  wrist.setPosition(0.43);
}

public void intakeUp() {
  wrist.setPosition(0.84);
}

public void low() {
  extend.setTargetPosition(350);
  climbl.setTargetPosition(300);
  climbr.setTargetPosition(300);
  score.setPosition(0.43);
}

public void level2() {
  extend.setTargetPosition(400);
  climbl.setTargetPosition(500);
  climbr.setTargetPosition(500);
  score.setPosition(0.43);
}

public void mid() {
  extend.setTargetPosition(450);
  climbl.setTargetPosition(525);
  climbr.setTargetPosition(525);
  score.setPosition(0.43);
}

public void home() {
  climbl.setTargetPosition(600);
  climbr.setTargetPosition(600);
  sleep(650);
  finger.setPosition(0.9);
  sleep(250);
  extend.setTargetPosition(0);
  climbl.setTargetPosition(0);
  climbr.setTargetPosition(0);
  score.setPosition(0.95);
}

public void scorePurple() {
  extend.setTargetPosition(300);
  sleep(750);
  home();
}

public void scoreYellow(double aprilTagTrackLocation) {
  sleep(waitTime * 1000);
  if(alliance == "Blue")
  {
    if(side == "Far") {
      if(innerPath==true||firstTile==true){
        drive(0.7,-15,-90,5.0);
        drive(0.7, -44, -90, 5.0);
      }else{
        drive(0.7, -59, -90, 5.0);
      }
      if(firstTile == true){
        strafe(0.5, -25, -90, 5.0);
      }else if ((innerPath == false)){
        strafe(0.7, 25, -90, 5.0);
      }
      
      level2();
      aprilTagTrack(aprilTagTrackLocation);
      drive(0.7, 5, -90, 5.0);
      home();      

      if(visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT){
          if(altPark == true){  
            strafe(0.8, -35, -90, 5.0);
          }else{  
            strafe(0.8, 30, -90, 5.0);
          }
      }else if(visionProcessor.getSelection() == FirstVisionProcessor.Selected.RIGHT){
        if(altPark == true){  
            strafe(0.8, -20, -90, 5.0);
          }else{  
            strafe(0.8, 37, -90, 5.0);
          }
      }else{
        if(altPark == true){  
            strafe(0.8, -27, -90, 5.0);
          }else{  
            strafe(0.8, 30, -90, 5.0);
          }
      } 
      drive(0.8, -7, -90, 5.0);

    }
    else{
      low();
      aprilTagTrack(aprilTagTrackLocation);
      home();
      if(visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT){
          if(altPark == true){  
            strafe(0.8, -35, -90, 5.0);
          }else{  
            strafe(0.8, 20, -90, 5.0);
          }
      }else if(visionProcessor.getSelection() == FirstVisionProcessor.Selected.RIGHT){
        if(altPark == true){  
            strafe(0.8, -20, -90, 5.0);
          }else{  
            strafe(0.8, 37, -90, 5.0);
          }
      }else{
        if(altPark == true){  
            strafe(0.8, -27, -90, 5.0);
          }else{  
            strafe(0.8, 30, -90, 5.0);
          }
        
      drive(0.8, -10, -90, 5.0);
      }
    }
  }
  else //red
  {
    if (side == "Far") {
      
      if(firstTile||innerPath){
        drive(0.7,-15,90,5.0);
        drive(0.7, -51, 90, 5.0);
      }else{
        drive(0.7, -65, 90, 5.0);
      }
      
      
      if(firstTile){
        strafe(0.5, 22, 90, 5.0);
        
      }else if(innerPath == false){
        strafe(0.7, -25, 90, 5.0);
        
      } else{
        //strafe(0.7, -10, 90,5.0);
        
      }
        level2();
        aprilTagTrack(aprilTagTrackLocation);
        drive(0.7, 5, 90, 5.0);
        home();
        
      if(visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT){
        if(altPark == true){  
            strafe(0.8, 20, 90, 5.0);
          }else{  
            strafe(1.0, -37, 90, 5.0);
            drive(0.8, -10, 90, 5.0);         
            }
      }else if(visionProcessor.getSelection() == FirstVisionProcessor.Selected.RIGHT){
        if(altPark == true){  
            strafe(0.8, 35, 90, 5.0);
          }else{  
            strafe(1.0, -25, 90, 5.0);
            drive(0.8, -10, 90, 5.0);          
            
          }
        
      }else{
        if(altPark == true){  
            strafe(0.8, 27, 90, 5.0);
          }else{  
            strafe(1.0, -30, 90, 5.0);
            drive(0.8, -10, 90, 5.0);          
            
          }
        
      }
      drive(0.8, -7, 90, 5.0);

    }
    else //red close
    {
      low();
      aprilTagTrack(aprilTagTrackLocation);
      home();
      
      if(visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT){
        if(altPark == true){  
            strafe(0.8, 20, 90, 5.0);
          }else{  
            strafe(1.0, -37, 90, 5.0);

          }
        
      }else if(visionProcessor.getSelection() == FirstVisionProcessor.Selected.RIGHT){
        if(altPark == true){  
            strafe(0.8, 35, 90, 5.0);
          }else{  
            strafe(1.0, -25, 90, 5.0);

          }
        
      }else{
        if(altPark == true){  
            strafe(0.8, 27, 90, 5.0);
          }else{  
            strafe(1.0, -30, 90, 5.0);
            
          }
      drive(0.8, -10, 90, 5.0);          

      }
    }
  }
  sleep(99999);
  
}

public void intakeToPos(int pos) {
  if (pos == 5) {
    intake.setPower(-1.0);
    wrist.setPosition(0.4484);
    climbl.setPower(1.0);
    climbr.setPower(1.0);
    climbl.setTargetPosition(800);
    climbr.setTargetPosition(800);
  }
}

public void drive(double speed, double inches, double angle, double timeoutS) {
  // Ensure that the OpMode is still active
  if (opModeIsActive()) {

    fl.setTargetPosition(fl.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH) + (int)((getAngle() + angle) * 6.5));
    fr.setTargetPosition(fr.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH) - (int)((getAngle() + angle) * 6.5));
    bl.setTargetPosition(bl.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH) + (int)((getAngle() + angle) * 6.5));
    br.setTargetPosition(br.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH) - (int)((getAngle() + angle) * 6.5));

    // reset the timeout time and start motion.
    runtime.reset();

    while (opModeIsActive() &&
      (runtime.seconds() < timeoutS) &&
      (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {

      double realSpeed = Math.min(runtime.seconds() * speed * 2, speed);

      fl.setPower(realSpeed);
      bl.setPower(realSpeed);
      fr.setPower(realSpeed);
      br.setPower(realSpeed);

      // Display it for the driver.
      telemetry.addData("Running to", " %7d :%7d", fl.getTargetPosition(), fr.getTargetPosition());
      telemetry.addData("Currently at", " at %7d :%7d", fl.getCurrentPosition(), fr.getCurrentPosition());
      telemetry.addData("Angle: ", getAngle());
      telemetry.update();
    }

    // Stop all motion;
    fl.setPower(0);
    bl.setPower(0);
    fr.setPower(0);
    br.setPower(0);

    sleep(250); // optional pause after each move.
  }
}

public void strafe(double speed, double inches, double angle, double timeoutS) {

  // Ensure that the OpMode is still active
  if (opModeIsActive()) {

    // Determine new target position, and pass to motor controller
    fl.setTargetPosition((fl.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH)) + (int)((getAngle() + angle) * 6.5));
    bl.setTargetPosition((bl.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH)) + (int)((getAngle() + angle) * 6.5));
    fr.setTargetPosition((fr.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH)) - (int)((getAngle() + angle) * 6.5));
    br.setTargetPosition((br.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH)) - (int)((getAngle() + angle) * 6.5));

    // reset the timeout time and start motion.
    runtime.reset();
    fl.setPower(speed);
    bl.setPower(speed);
    fr.setPower(speed);
    br.setPower(speed);

    while (opModeIsActive() &&
      (runtime.seconds() < timeoutS) &&
      (fl.isBusy() && fr.isBusy())) {

      // Display it for the driver.
      telemetry.addData("Running to", " %7d :%7d", fl.getTargetPosition(), fr.getTargetPosition());
      telemetry.addData("Currently at", " at %7d :%7d", fl.getCurrentPosition(), fr.getCurrentPosition());
      telemetry.update();
    }

    // Stop all motion;
    fl.setPower(0);
    bl.setPower(0);
    fr.setPower(0);
    br.setPower(0);

    sleep(250); // optional pause after each move.
  }
}


private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }
        
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Camera Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    
void updateOrientation() {
  RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
  RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
  try {
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
    imu.initialize(new IMU.Parameters(orientationOnRobot));
    orientationIsValid = true;
  } catch (IllegalArgumentException e) {
    orientationIsValid = false;
  }
}

public void aprilTagTrack(double xPosition){
    
    double xError = 999;
    double yError = 999;
    double headingError = 0;

    while(((Math.abs(xError) > 0.5 || Math.abs(yError) > 0.5 || (aprilTagY > 15 && extend.getTargetPosition() != 0) || !targetFound) && opModeIsActive() ) ){
                    targetFound = false;
                    desiredTag  = null;
                    aprilTagY = 0.0;
                    aprilTagX = 0.0;
        
                    // Step through the list of detected tags and look for a matching tag
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        // Look to see if we have size info on this tag.
                        if (detection.metadata != null) {
                            
                            int detectionId = detection.id;
                            
                            //make sure we don't auto track the other april tags
                            if(detectionId > 6)
                            {
                                targetFound = false;
                            }
                            else
                            {
                                targetFound = true;
                            }
                            
                            if (detection.id > 3) {
                                detectionId = detectionId - 3;
                            }
                            aprilTagY = aprilTagY + detection.ftcPose.y;
                            aprilTagX = aprilTagX + (-(detectionId - 2) * 3.8) + detection.ftcPose.x;
                            telemetry.addData("Found: ", "Tag ID %d", detectionId);
                        } else {
                            // This tag is NOT in the library, so we don't have enough information to track to it.
                            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        }
                    }
                    
                    aprilTagY = aprilTagY / currentDetections.size();
                    aprilTagX = aprilTagX / currentDetections.size();
        
                    // Tell the driver what we see, and what to do.
                    if (targetFound) {
                        telemetry.addData("Average Y Value: ", aprilTagY);
                        telemetry.addData("Average X Value: ", aprilTagX);
                    }
                    
                    autoAim = true;
                    
                    if (true) {
                        
                        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

                        
                        
                        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                        
                        if(alliance == "Blue")
                        {
                           headingError    =  orientation.getYaw(AngleUnit.DEGREES) - 90;
                        }
                        else
                        {
                           headingError    =  orientation.getYaw(AngleUnit.DEGREES) + 90;
                        }
                        
                        xError  = (aprilTagX - xPosition);
                        
                        if(aprilTagY < 26 && extend.getCurrentPosition() > 200 && Math.abs(xError) < 4)
                        {
                            DESIRED_DISTANCE = 12.5;
                        }
                        else
                        {
                            DESIRED_DISTANCE = 25;
                        } 
                        
                        yError      = (aprilTagY - DESIRED_DISTANCE);
        
                        
                        if(autoAim){
                            aprilTagSum = aprilTagX + aprilTagY;
                            if(aprilTagSum != oldAprilTagSum) {

                                if(headingError < 10)
                                {
                                    double forwardTicks = 10;//8
                                    
                                    double strafeTicks = 12;//10
                                    
                                    
                                    fl.setTargetPosition(fl.getCurrentPosition() - (int)(yError * forwardTicks) - (int)(xError *strafeTicks) + (int)(headingError*6.5));
                                    fr.setTargetPosition(fr.getCurrentPosition() - (int)(yError * forwardTicks) + (int)(xError *strafeTicks) - (int)(headingError*6.5));
                                    bl.setTargetPosition(bl.getCurrentPosition() - (int)(yError * forwardTicks) + (int)(xError *strafeTicks) + (int)(headingError*6.5));
                                    br.setTargetPosition(br.getCurrentPosition() - (int)(yError * forwardTicks) - (int)(xError *strafeTicks) - (int)(headingError*6.5));
                                    
                                    
                                }
                                else
                                {
                                    fl.setTargetPosition(fl.getCurrentPosition() + (int)(headingError*6.5));
                                    fr.setTargetPosition(fr.getCurrentPosition() - (int)(headingError*6.5));
                                    bl.setTargetPosition(bl.getCurrentPosition() + (int)(headingError*6.5));
                                    br.setTargetPosition(br.getCurrentPosition() - (int)(headingError*6.5));
                                }
                                
                                fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                                fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                                bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                                br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                                
                                double motorPower = 1.0;
                                fl.setPower(motorPower);
                                fr.setPower(motorPower);
                                bl.setPower(motorPower);
                                br.setPower(motorPower);
                            }
                        }
                        
                    }
                    
                    
                    oldAprilTagSum = aprilTagSum;
                    telemetry.addData("AprilTagX ", aprilTagX);
                    telemetry.addData("AprilTagY ", aprilTagX);
                    telemetry.addData("Heading ", headingError);
                    telemetry.update();
     
                }
            
}

public void testDrive() 
{
  double timeoutS = 10.0;
  int speed = 2000;
  int ticks = 2000;
  double targetHeading = 0.0;
  
  int distTraveled = 0;
  
  int startingLocation = fl.getCurrentPosition();
  
  int targetLocation = startingLocation + ticks;
  int distToTarget = targetLocation;
  

  fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
  fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
  bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
  br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

  // reset the timeout time and start motion.
  runtime.reset();

  while (opModeIsActive() &&
    (runtime.seconds() < timeoutS) && Math.abs(distTraveled) < targetLocation ) {
      
    distTraveled = fl.getCurrentPosition() - startingLocation;
    distToTarget = targetLocation - distTraveled;

    double realSpeed = speed * (0.1 + ( Math.min((double)(distTraveled)/200, (double)(distToTarget)/1500) ));
    //realSpeed = Math.min(realSpeed * speed, speed);

    driveYXH(1.0,0.0,90.0,realSpeed);

    // Display it for the driver.
    telemetry.addData("Running to", " %7d", targetLocation);
    telemetry.addData("Currently at", " %7d", distTraveled);
    telemetry.addData("Dist To Target: ", distToTarget);
    telemetry.addData("Angle: ", getAngle());
    telemetry.addData("Speed: ", realSpeed);
    telemetry.update();
  }

  // Stop all motion;
  fl.setPower(0);
  bl.setPower(0);
  fr.setPower(0);
  br.setPower(0);

  sleep(9999999); // optional pause after each move.
  
}

public void driveYXH(double ry, double rx, double target, double velocity){
        double heading = getHeading();
        double error = AngleUnit.normalizeDegrees(heading - target);
        driveYXW(ry, rx, error * 0.03, velocity);
    }

    public double getHeading(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    } 


public void driveYXW(double ry, double rx, double rw, double velocity) {
        rx = rx * 1.1;  // help correct strafing
        double denominator = Math.max(Math.abs(ry) + Math.abs(rx) + Math.abs(rw), 1) / velocity;
        int lfPower = (int)((ry + rx + rw) / denominator);
        int lbPower = (int)((ry - rx + rw) / denominator);
        int rfPower = (int)((ry - rx - rw) / denominator);
        int rbPower = (int)((ry + rx - rw) / denominator);

        fl.setVelocity(lfPower);
        bl.setVelocity(lbPower);
        fr.setVelocity(rfPower);
        br.setVelocity(rbPower);
    }


public double getAngle() {
  return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
}

}
