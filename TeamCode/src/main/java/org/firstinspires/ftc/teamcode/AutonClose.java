package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutonClose extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    double startposx = 12;
    double startposy = 72;
    double startheading = Math.toRadians(90);
    double tagposx=0;
    double tagposy=0;

    @Override
    public void runOpMode() throws InterruptedException {
        //ROBOT INITIALIZATION ---------------------------------------------------------------------
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startposx, startposy, startheading));

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        telemetry.addData("--Frostbite Close Auto--",true);
        telemetry.addData("Placement: ", robot.visionProcessor.getSelection());
        telemetry.update();

        //EXECUTE ACTIONS -----------------------------------------------------------------
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE) {
                tagposy = -24;
            }else if(robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT){
                tagposy = -48;
            }else{
                tagposy = -72;
            }
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToY(tagposy)
                            //.splineTo(new Vector2d(startposx+24,startposy+24), startheading)
                            .build());
        }

    }
}
