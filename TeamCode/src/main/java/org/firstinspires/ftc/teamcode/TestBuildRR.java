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
        BaseConstants.Params $ = new BaseConstants.Params();

        robot.finger.setPosition($.FINGER_IN);

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
            if(gamepad1.a){
                Actions.runBlocking(new SequentialAction(
                        robot.low()
                ));
                sleep(500);
            }
            if(gamepad1.b){
                Actions.runBlocking(new SequentialAction(
                        robot.home()
                ));
                sleep(500);
            }
            if(gamepad1.x){
                Actions.runBlocking(new SequentialAction(
                        robot.intakeGround()
                ));
                sleep(500);
            }
            if(gamepad1.y){
                Actions.runBlocking(new SequentialAction(
                        robot.intakeUp()
                ));
                sleep(500);
            }
            if (gamepad1.dpad_up){
                Actions.runBlocking(new SequentialAction(
                        robot.mid()
                ));
                sleep(500);
            }
            if (gamepad1.dpad_down){
                Actions.runBlocking(new SequentialAction(
                        robot.spikeExtend()
                ));
                sleep(500);
            }
            if (gamepad1.dpad_left){
                Actions.runBlocking(new SequentialAction(
                        robot.spikeScore()
                ));
                sleep(500);
            }
            if (gamepad1.dpad_right){
                Actions.runBlocking(new SequentialAction(
                        robot.retract()
                ));
                sleep(500);
            }
            sleep(10);
            telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
            telemetry.addData("", "");
            telemetry.addData("ClimbL Pos: ", robot.climbl.getCurrentPosition());
            telemetry.addData("ClimbR Pos: ", robot.climbr.getCurrentPosition());
            telemetry.addData("Climb Target: ", robot.climbl.getTargetPosition());
            telemetry.addData("", "");
            telemetry.addData("Extend Pos: ", robot.extend.getCurrentPosition());
            telemetry.addData("Extend Pos: ", robot.extend.getCurrentPosition());
            telemetry.addData("Extend Target: ", robot.extend.getTargetPosition());
            telemetry.addData("Extend Power: ", robot.extend.getPower());
            telemetry.addData("", "");
            telemetry.addData("Finger: ", robot.finger.getPosition());
            telemetry.addData("Wrist: ", robot.wrist.getPosition());
            telemetry.update();
        }

    }
}
