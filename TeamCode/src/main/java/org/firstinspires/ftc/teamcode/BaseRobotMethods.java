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


public class BaseRobotMethods{

    //VARIABLES---------------------------------------------------------------------------------------------------------------
    private static final boolean USE_WEBCAM = true;
    double DESIRED_DISTANCE = 20.0; //  this is how close the camera should get to the target (inches)
    private int DESIRED_TAG_ID = 5;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
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
    ElapsedTime timer = new ElapsedTime();
    boolean holdWrist = false;
    boolean resetEncoder = false;
    public FirstVisionProcessor visionProcessor;
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

        droneLaunch.setPosition(0.4);
        score.setPosition(scoreHome);

        //set the intake to starting position
        elbowl.setPosition(0.29 + elbowHome);
        elbowr.setPosition(0.25 + elbowHome);
        wrist.setPosition(0.725);
        finger.setPosition(0.46);

        elevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorLimit");

        //CAMERA INIT
        visionProcessor = new FirstVisionProcessor();
        initCamera(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward

        updateOrientation();

        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;

        boolean targetFound = false;    // Set to true when an AprilTag target is detected

        if (USE_WEBCAM) setManualExposure(3, 250);

        visionPortal.setProcessorEnabled(visionProcessor, true);
    }

    //CAMERA COMMANDS ----------------------------------------------------------------------------------------------------
    public void initCamera(HardwareMap hardwareMap){
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcamback"))
                .addProcessors(visionProcessor,aprilTag)
                .setCameraResolution(new Size(640, 360)) // 544 288
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();
    }

    public void aprilTagTrack(int trackLocation)
    {
        visionPortal.setProcessorEnabled(visionProcessor, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        double headingStart = Math.round(orientation.getYaw(AngleUnit.DEGREES)/90) * 90;
        double headingError = 0.0;

        while(!parent.isStopRequested() && (Math.abs(aprilTagYError) < 0.25 || Math.abs(aprilTagXError) < 0.25)) {
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {

                    int detectionId = detection.id;

                    //make sure we don't auto track the other april tags
                    if (detectionId > 6) {
                        targetFound = false;
                    } else {
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

            autoAim = false;
            if (targetFound && aprilTagY < 50.0) {
                autoAim = true;
            }

            if (autoAim) {

                headingError = orientation.getYaw(AngleUnit.DEGREES) - headingStart;

                xPosition = (trackLocation-2)*3.8;

                double aprilTagXError = -(aprilTagX - xPosition);

                if (aprilTagY < 26 && extend.getCurrentPosition() > 200) {
                    DESIRED_DISTANCE = 13 + Math.abs(aprilTagXError);
                } else {
                    DESIRED_DISTANCE = 25;
                }

                double aprilTagYError = -(aprilTagY - DESIRED_DISTANCE);


                if (autoAim) {
                    aprilTagSum = aprilTagX + aprilTagY;
                    if (aprilTagSum != oldAprilTagSum) {

                        if (Math.abs(orientation.getYaw(AngleUnit.DEGREES)) < 10) {
                            double forwardTicks = 8;

                            double strafeTicks = 10;

                            if (aprilTagXError < 5 && fl.getVelocity() < 100) {
                                strafeTicks = 10;
                            }
                            fl.setTargetPosition(fl.getCurrentPosition() + (int) (aprilTagYError * forwardTicks) + (int) (aprilTagXError * strafeTicks) - (int) (headingError * 6.5));
                            fr.setTargetPosition(fr.getCurrentPosition() + (int) (aprilTagYError * forwardTicks) - (int) (aprilTagXError * strafeTicks) + (int) (headingError * 6.5));
                            bl.setTargetPosition(bl.getCurrentPosition() + (int) (aprilTagYError * forwardTicks) - (int) (aprilTagXError * strafeTicks) - (int) (headingError * 6.5));
                            br.setTargetPosition(br.getCurrentPosition() + (int) (aprilTagYError * forwardTicks) + (int) (aprilTagXError * strafeTicks) + (int) (headingError * 6.5));

                        } else {
                            fl.setTargetPosition(fl.getCurrentPosition() - (int) (headingError * 6.5));
                            fr.setTargetPosition(fr.getCurrentPosition() + (int) (headingError * 6.5));
                            bl.setTargetPosition(bl.getCurrentPosition() - (int) (headingError * 6.5));
                            br.setTargetPosition(br.getCurrentPosition() + (int) (headingError * 6.5));
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
            aprilTagY = 0.0;
            aprilTagX = 0.0;
            oldAprilTagSum = aprilTagSum;

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Average Y Value: ", aprilTagY);
                telemetry.addData("Average X Value: ", aprilTagX);
                telemetry.addData("Heading Start", headingStart);
                telemetry.addData("Heading Error: ", headingError);
            }

            telemetry.update();
        }
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


    public class Home implements Action{ //(double power)
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
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

    public class IntakeGround implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            elbowl.setPosition(0.59);//  INTAKE DOWN and TURN ON
            elbowr.setPosition(0.55);
            wrist.setPosition(0.3);

            intake.setPower(-1.0); //turn intake on full speed

            passiveIntake = true;
            return false;
        }
    }
    public Action intakeGround() {
        return new IntakeGround();
    }

    public class Transfer implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
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
            return false;
        }
    }
    public Action transfer(){return new Transfer();}
    //wait until elevator limit switch is pressed


    public class IntakeUp implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            elbowl.setPosition(0.29 + elbowHome);
            elbowr.setPosition(0.25 + elbowHome);
            wrist.setPosition(0.725);
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
            finger.setPosition(0.84);
            return false;
        }
    }
    public Action fingerHome(){
        return new FingerHome();
    }


    public class Low implements Action { //(int extendTarget)
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setTargetPosition(350);
            climbl.setTargetPosition(260);
            climbr.setTargetPosition(260);
            score.setPosition(0.4);
            return false;
        }
    }
    public Action low(){
        return new Low();
    }

    public class Mid implements Action{ //(int extendTarget)
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extend.setTargetPosition(450);
            climbl.setTargetPosition(525);
            climbr.setTargetPosition(525);
            score.setPosition(0.34);
            return false;
        }


    }
    public Action mid(){
        return new Mid();
    }


    public class IntakeToPos implements Action {
        public Action init(int pos){
            if (pos == 5) {
                //wrist.setPosition(0.485);
                //elbowl.setPosition(0.95);
                //elbowr.setPosition(0.35);
            } else if (pos == 4) {
                //wrist.setPosition(0.455);
                //elbowl.setPosition(0.965);
                //elbowr.setPosition(0.335);
            } else if (pos == 3) {
                //wrist.setPosition(0.475);
                //elbowl.setPosition(0.965);
                //elbowr.setPosition(0.335);
            } else if (pos == 2) {
                //wrist.setPosition(0.42);
                //elbowl.setPosition(0.965);
                //elbowr.setPosition(0.335);
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

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Camera Waiting");
            telemetry.update();
            while (!parent.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                parent.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!parent.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                parent.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            parent.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            parent.sleep(20);
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

    public static double Tiles(double amt_of_tiles){
        return (double) amt_of_tiles*24;
    }
}
