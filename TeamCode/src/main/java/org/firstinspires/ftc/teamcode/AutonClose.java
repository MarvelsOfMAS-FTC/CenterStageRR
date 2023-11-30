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
    //START POS
    double startposx = 12;
    double startposy = 72;
    double startheading = Math.toRadians(90);

    //TAG POS
    double tagheading = Math.toRadians(100);
    double tagoffset = 0;
    ElapsedTime consoletime = new ElapsedTime();
    public final String PREFIX= "["+consoletime.now(TimeUnit.SECONDS)+"]";
    double sideHeading =0;

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
                sideHeading=0;
            } else if(gamepad1.b) {
                side = "Red";
                sideHeading=180;
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
            consoletime.reset();
            consoletime.startTime();
            telemetry.addData(PREFIX,"Running!");
            telemetry.update();

            //SELECT TEAM ELEMENT SIDE
            if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE) {
                tagheading = Math.toRadians(100+sideHeading);
                tagoffset = 0;
                telemetry.addData(PREFIX,"Found Middle setting tag heading and offset!");
            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagheading = Math.toRadians(125+sideHeading); //130
                tagoffset = 6;
                telemetry.addData(PREFIX,"Found Left setting tag heading and offset!");
            } else {
                tagheading = Math.toRadians(60+sideHeading); //60
                tagoffset = -5;
                telemetry.addData(PREFIX,"Found Right setting tag heading and offset!");
            }
            telemetry.addData(PREFIX,"Done With Camera Loading spike Traj");
            telemetry.update();
            if(side.equalsIgnoreCase("Blue")) {
                //SCORE SPIKE MARK PIXEL & DRIVE TO BACKDROP
                Action spikeMark = drive.actionBuilder(new Pose2d(startposx,startposy,startheading))
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
                telemetry.addData(PREFIX, "Running spike traj");
                telemetry.update();
                drive.updatePoseEstimate();
                telemetry.addData(PREFIX, "Updating Drive Pos Estimate");
                telemetry.addData(PREFIX, "Loading backDrop Traj");
                telemetry.update();

                //SCORE BACKDROP PIXEL
                Action backDrop = drive.actionBuilder(drive.pose)
                        //CLEAR PIXEL AFTER SCORING
                        .afterTime(0.5, robot.mid())
                        .afterTime(3, robot.home())
                        //PUSH INTO BACKDROP & SCORE
                        .lineToX(44)
                        .strafeToConstantHeading(new Vector2d(36, 38 - tagoffset))

                        .build();
                telemetry.addData(PREFIX, "Running backdrop traj");
                telemetry.update();
                Actions.runBlocking(backDrop);
                drive.updatePoseEstimate();
                telemetry.addData(PREFIX, "Updating Drive Pos Estimate");
                telemetry.addData(PREFIX, "Loading cyclePixel traj");
                telemetry.update();

                //CYCLE PIXEL STACK
                /*Action cyclePixel = drive.actionBuilder(drive.pose)
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
                telemetry.addData(PREFIX, "Running cyclePixel traj");
                telemetry.update();
                Actions.runBlocking(cyclePixel);
                drive.updatePoseEstimate();
                telemetry.addData(PREFIX, "Updating Drive Pos Estimate");
                telemetry.update();

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
                telemetry.addData(PREFIX, "Running cyclePixel1 traj");
                telemetry.update();
                Actions.runBlocking(cyclePixel1);
                drive.updatePoseEstimate();
                telemetry.addData(PREFIX, "Updating Drive Pos Estimate");
                telemetry.update();

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
                telemetry.addData(PREFIX, "Running cyclePixel2 traj");
                telemetry.update();

                Actions.runBlocking(cyclePixel2);
                drive.updatePoseEstimate();
                telemetry.addData(PREFIX, "Updating Drive Pos Estimate");
                telemetry.update();
                double timeleft = 30 - consoletime.now(TimeUnit.SECONDS);
                telemetry.addData(PREFIX, "Done Auton! With " + timeleft + " Left");
                telemetry.update();*/
            }else if (side.equalsIgnoreCase("Red")){
                //SCORE SPIKE MARK PIXEL & DRIVE TO BACKDROP
                telemetry.addData(PREFIX, "Red Side");
                Action spikeMark = drive.actionBuilder(new Pose2d(startposx,-startposy,startheading+sideHeading))
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
                telemetry.addData(PREFIX, "Running spike traj");
                telemetry.update();
                drive.updatePoseEstimate();
                telemetry.addData(PREFIX, "Updating Drive Pos Estimate");
                telemetry.addData(PREFIX, "Loading backDrop Traj");
                telemetry.update();

                //SCORE BACKDROP PIXEL
                Action backDrop = drive.actionBuilder(drive.pose)
                        //CLEAR PIXEL AFTER SCORING
                        .afterTime(0.5, robot.mid())
                        .afterTime(3, robot.home())
                        //PUSH INTO BACKDROP & SCORE
                        .lineToX(44)
                        .strafeToConstantHeading(new Vector2d(36, 38 - tagoffset))

                        .build();
                telemetry.addData(PREFIX, "Running backdrop traj");
                telemetry.update();
                Actions.runBlocking(backDrop);
                drive.updatePoseEstimate();
                telemetry.addData(PREFIX, "Updating Drive Pos Estimate");
                telemetry.addData(PREFIX, "Loading cyclePixel traj");
                telemetry.update();
            }else{
                telemetry.addLine("Nothing Selected Not Running Auton :( Next Time Use X or B");
            }
            telemetry.update();
            break;
        }
    }
}