package org.firstinspires.ftc.teamcode;


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

@Autonomous
public class AutonFar extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    public String side = "None";
    public String placement = "None"; //this is the variable for which spot the robot should score
    public boolean grabStack = true; //grabstack means robot will go pick up 2 pixels and deposit

    //START POS
    double startposx = 0;
    double startposy = 0;
    double startheading = Math.toRadians(90);

    //TAG POS
    double tagposx=0;
    double tagposy=-5;
    double tagheading = Math.toRadians(0);

    @Override
    public void runOpMode() throws InterruptedException {
        //ROBOT INITIALIZATION ---------------------------------------------------------------------
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startposx, startposy, startheading));
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        //INIT POSITIONS
        robot.home();
        robot.finger.setPosition(0.37);

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        telemetry.addData("Placement: ", robot.visionProcessor.getSelection());
        telemetry.update();
        robot.climbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.climbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //WAIT FOR START CODE ----------------------------------------------------------------------
        while (!opModeIsActive() && !isStopRequested())
        {
            //Running pipeline
            if(gamepad1.x) {
                side = "Blue";
            } else if(gamepad1.b) {
                side = "Red";
            }

            //determine if we should grab the stack
            if(gamepad1.a) {
                grabStack = true;
            } else if(gamepad1.y) {
                grabStack = false;
            }

            telemetry.addData("--Frostbite Close Auto--",true);
            telemetry.addData("Placement: ", robot.visionProcessor.getSelection());
            telemetry.addData("Side: ", side);
            telemetry.addData("GrabStack?: ", grabStack);

            telemetry.addData("Press X for Blue Side",true);
            telemetry.addData("Press B for Red Side",true);
            telemetry.addData("Press A for Grabbing Stack",true);
            telemetry.addData("Press Y for NOT Grabbing Stack",true);
            telemetry.update();
        }

        //EXECUTE ACTIONS -----------------------------------------------------------------
        while (opModeIsActive() && !isStopRequested()) {

            //SELECT TEAM ELEMENT SIDE
            if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE) {
                tagheading = 0;
            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagheading= 90;
            } else {
                tagheading= -90;
            }

            //ROADRUNNER TRAJECTORIES BUILD
            Action strafeLeft = drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(24,0))
                    .turn(Math.toRadians(90))
                    .build();

            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            strafeLeft
                    ),
                    new SequentialAction(
                            new SleepAction(2),
                            robot.low()
                    )
                )
            );

            drive.updatePoseEstimate();
            Action splineStraight = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(48,48),90)
                    .build();

            Actions.runBlocking(new SequentialAction(
                            new ParallelAction(
                                    splineStraight
                            ),
                            new SequentialAction(
                                    robot.home(),
                                    new SleepAction(2),
                                    robot.low()

                            )
                    )
            );

            break;
        }
    }

}

