package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class TestBuildRR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
        telemetry.update();

        //WAIT FOR START CODE ----------------------------------------------------------------------
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("","");
            telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
            telemetry.update();
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
            telemetry.update();
            if(gamepad1.a){
                Actions.runBlocking(new SequentialAction(
                        robot.low()
                ));
            }
            if(gamepad1.b){
                Actions.runBlocking(new SequentialAction(
                        robot.home()
                ));
            }
            if(gamepad1.x){
                Actions.runBlocking(new SequentialAction(
                        robot.intakeGround()
                ));
            }
            if(gamepad1.y){
                Actions.runBlocking(new SequentialAction(
                        robot.intakeUp()
                ));
            }
            if (gamepad1.dpad_up){
                Actions.runBlocking(new SequentialAction(
                        robot.mid()
                ));
            }
            if (gamepad1.dpad_down){
                Actions.runBlocking(new SequentialAction(
                        robot.spikeExtend()
                ));
            }
            if (gamepad1.dpad_left){
                Actions.runBlocking(new SequentialAction(
                        robot.spikeScore()
                ));
            }
            if (gamepad1.dpad_right){
                Actions.runBlocking(new SequentialAction(
                        robot.retract()
                ));
            }

            telemetry.update();
        }

    }
}
