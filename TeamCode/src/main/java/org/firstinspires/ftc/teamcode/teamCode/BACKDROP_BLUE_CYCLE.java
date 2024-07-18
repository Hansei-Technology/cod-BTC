package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;


@Autonomous
public class BACKDROP_BLUE_CYCLE extends LinearOpMode {
    SampleMecanumDrive drive;
    Robot robot;


    //public Robotel robotel;

    @Override
    public void runOpMode() throws InterruptedException {{
                drive = new SampleMecanumDrive(hardwareMap);
                Pose2d START_POSE = new Pose2d(14, 65, Math.toRadians(-90));
                robot = new Robot(this);
                robot.start();
                robot.arm.goMid();
                robot.joint.goToUp();
                robot.claw.toggleLeft();
                robot.claw.toggleRight();
                drive.setPoseEstimate(START_POSE);
                waitForStart();

                robot.arm.goDown();
                robot.lift.goTOPos(robot.lift.MaxPoz);

                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(START_POSE)

                        .lineToLinearHeading(new Pose2d(13, 60, Math.toRadians(-110)))//pune pixel pe right
                        .addTemporalMarker(() -> {
                            robot.claw.toggleRight();
                        })
                        .waitSeconds(0)
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> {
                            robot.arm.goMid();
                            robot.lift.goMid();
                            robot.joint.goToMid();
                        })
                        .waitSeconds(0)
                                .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(50, 32, Math.toRadians(0)))
                        .addTemporalMarker(() -> {
                            robot.claw.toggleLeft();
                        })
                                .waitSeconds(0)
                                .lineTo(new Vector2d(40, 32))
                                .waitSeconds(0.2)
                        .addTemporalMarker(() -> {
                            robot.lift.goTOPos(0);
                        })
                        .waitSeconds(0)
                        .lineToLinearHeading(new Pose2d(40, 0, Math.toRadians(180)))
                        .addTemporalMarker( () -> {
                            robot.lift.goDown();
                            robot.arm.goMid();
                        })
                        .waitSeconds(5)
                        .build()
                );
        if(isStopRequested()) {
            robot.interrupt();
            stop();
        }
        robot.interrupt();
            }}}

