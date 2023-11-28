package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.variable.VariableType;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous
public class AutonClose extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    public String side = "None";
    public String placement = "None"; //this is the variable for which spot the robot should score

    //START POS
    double startposx = 12;
    double startposy = 72;
    double startheading = Math.toRadians(90);

    //TAG POS
    double tagheading = Math.toRadians(100);
    double tagoffset = 0;
    ElapsedTime consoletime = new ElapsedTime();
    public final String PREFIX= "["+consoletime.now(TimeUnit.SECONDS)+"]";

    @Override
    public void runOpMode() throws InterruptedException {
        //ROBOT INITIALIZATION ---------------------------------------------------------------------
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startposx, startposy, startheading));
        drive.updatePoseEstimate();

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        telemetry.addData("Placement: ", robot.visionProcessor.getSelection());
        telemetry.update();

        //WAIT FOR START CODE ----------------------------------------------------------------------
        while (!opModeIsActive() && !isStopRequested())
        {
            //Running pipeline
            if(gamepad1.x) {
                side = "Blue";
            } else if(gamepad1.b) {
                side = "Red";
            }



            telemetry.addData("--Frostbite Close Auto--",true);
            telemetry.addData("Placement: ", robot.visionProcessor.getSelection());
            telemetry.addData("Side: ", side);
            telemetry.addData("Position Estimate",drive.updatePoseEstimate());
            telemetry.addData("Press X for Blue Side",true);
            telemetry.addData("Press B for Red Side",true);
            telemetry.update();
        }


        //EXECUTE ACTIONS -----------------------------------------------------------------
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("----------- Console Logger V1.0 -----------");
            telemetry.addData(PREFIX,"Running!");
            telemetry.update();

            //SELECT TEAM ELEMENT SIDE
            if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE) {
                tagheading = Math.toRadians(100);
                tagoffset = 0;
                telemetry.addData(PREFIX,"Found Middle setting tag heading and offset!");
            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagheading = Math.toRadians(125); //130
                tagoffset = 4;
                telemetry.addData(PREFIX,"Found Left setting tag heading and offset!");
            } else {
                tagheading = Math.toRadians(75); //60
                tagoffset = -4;
                telemetry.addData(PREFIX,"Found Right setting tag heading and offset!");
            }
            telemetry.addData(PREFIX,"Done With Camera Loading spike Traj");

            //SCORE SPIKE MARK PIXEL & DRIVE TO BACKDROP
            Action spikeMark = drive.actionBuilder(drive.pose)
                    //SCORE MARK PIXEL
                    .afterTime(0, robot.spikeExtend())
                    .afterTime(1, robot.spikeScore())
                    .afterTime(1.25, robot.fingerHome())
                    .afterTime(1.5, robot.home())

                    //PREPARE BACKDROP PIXEL
                    .afterTime(3, robot.low())

                    //THE HEADING IS CONTROLLED BY THE VISION CODE
                    .lineToYLinearHeading(55, tagheading)
                    .waitSeconds(0.5)
                    .splineToLinearHeading(new Pose2d(36, 38 + tagoffset, Math.toRadians(180)), Math.toRadians(tagheading))
                    .build();

            Actions.runBlocking(spikeMark);
            telemetry.addData(PREFIX,"Running spike traj");
            drive.updatePoseEstimate();
            telemetry.addData(PREFIX,"Updating Drive Pos Estimate");
            telemetry.addData(PREFIX,"Loading backDrop Traj");

            //SCORE BACKDROP PIXEL
            Action backDrop = drive.actionBuilder(drive.pose)
                    //CLEAR PIXEL AFTER SCORING
                    .afterTime(0.5, robot.mid())

                    //PUSH INTO BACKDROP & SCORE
                    .lineToX(44)
                    .strafeToConstantHeading(new Vector2d(36,38-tagoffset))
                    .build();
            telemetry.addData(PREFIX,"Running backdrop traj");
            Actions.runBlocking(backDrop);
            drive.updatePoseEstimate();
            telemetry.addData(PREFIX,"Updating Drive Pos Estimate");
            telemetry.addData(PREFIX,"Loading cyclePixel traj");

            //CYCLE PIXEL STACK
            Action cyclePixel = drive.actionBuilder(drive.pose)
                    //TUCK IN SCORE BUCKET & WHIP OUT INTAKE
                    .afterTime(0, robot.home())
                    .afterTime(1.75, robot.intakeLevel5())
                    .afterTime(3.25, robot.intakeUp())
                    .afterTime(4, robot.transfer())
                    .afterTime(5, robot.mid())

                    //GOTO STACK AND RETURN
                    .lineToX(-55)
                    .waitSeconds(0.75)
                    .lineToX(42)
                    .waitSeconds(0.5)
                    .build();
            telemetry.addData(PREFIX,"Running cyclePixel traj");
            Actions.runBlocking(cyclePixel);
            drive.updatePoseEstimate();
            telemetry.addData(PREFIX,"Updating Drive Pos Estimate");

            Action cyclePixel1 = drive.actionBuilder(drive.pose)
                    //TUCK IN SCORE BUCKET & WHIP OUT INTAKE
                    .afterTime(0.5, robot.home())
                    .afterTime(1.75, robot.intakeGround())
                    .afterTime(3.25, robot.intakeUp())
                    .afterTime(4, robot.transfer())
                    .afterTime(5, robot.mid())

                    //GOTO STACK AND RETURN
                    .lineToX(-55)
                    .waitSeconds(0.75)
                    .lineToX(42)
                    .waitSeconds(0.5)
                    .build();
            telemetry.addData(PREFIX,"Running cyclePixel1 traj");
            Actions.runBlocking(cyclePixel1);
            drive.updatePoseEstimate();
            telemetry.addData(PREFIX,"Updating Drive Pos Estimate");

            Action cyclePixel2 = drive.actionBuilder(drive.pose)
                    //TUCK IN SCORE BUCKET & WHIP OUT INTAKE
                    .afterTime(0.5, robot.home())
                    .afterTime(1.75, robot.intakeGround())
                    .afterTime(3.25, robot.intakeUp())
                    .afterTime(4, robot.transfer())
                    .afterTime(5, robot.mid())
                    .afterTime(7, robot.home())

                    //GOTO STACK AND RETURN
                    .lineToX(-55)
                    .waitSeconds(0.75)
                    .lineToX(42)
                    .waitSeconds(2)
                    .build();
            telemetry.addData(PREFIX,"Running cyclePixel2 traj");

            Actions.runBlocking(cyclePixel2);
            drive.updatePoseEstimate();
            telemetry.addData(PREFIX,"Updating Drive Pos Estimate");
            double timeleft = 30 - consoletime.now(TimeUnit.SECONDS);
            telemetry.addData(PREFIX,"Done Auton! With "+timeleft+" Left");

            break;
        }
    }
}