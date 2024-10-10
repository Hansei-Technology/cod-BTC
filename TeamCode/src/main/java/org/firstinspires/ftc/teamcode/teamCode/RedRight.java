package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class RedRight extends LinearOpMode {
    public static double PRELOAD_x = 31.2, PRELOAD_y = 0, PRELOAD_heading = 0;
    public static double SAFE_x = 11.7, SAFE_y = -37, SAFE_heading = 270;
    public static double SAFE2_x = 50, SAFE2_y = -37, SAFE2_heading = 180;
    public static double ELEM1_x = 50, ELEM1_y = -48, ELEM1_heading = 180;
    public static double ELEM2_x = 50, ELEM2_y = -62, ELEM2_heading = 180;
    public static double ELEM3_x = 50, ELEM3_y = -67, ELEM3_heading = 180;
    public static double HUMAN_x = 5, HUMAN_y = -67, HUMAN_heading = 190;
    public static double HUMAN2_x = -5, HUMAN2_y = -68, HUMAN2_heading = 180;
    @Override
    public void runOpMode() throws InterruptedException {

        OrizController oriz = new OrizController(hardwareMap);
        LiftController lift = new LiftController(hardwareMap);
        ClawController1Servo claw = new ClawController1Servo(hardwareMap);
        JointController1Servo joint = new JointController1Servo(hardwareMap);


        Pose2d PRELOAD = new Pose2d(PRELOAD_x, PRELOAD_y, Math.toRadians(PRELOAD_heading));
        Pose2d SAFE = new Pose2d(SAFE_x, SAFE_y, Math.toRadians(SAFE_heading));
        Pose2d SAFE2 = new Pose2d(SAFE2_x, SAFE2_y, Math.toRadians(SAFE2_heading));
        Pose2d ELEM1 = new Pose2d(ELEM1_x, ELEM1_y, Math.toRadians(ELEM1_heading));
        Pose2d ELEM2 = new Pose2d(ELEM2_x, ELEM2_y, Math.toRadians(ELEM2_heading));
        Pose2d ELEM3 = new Pose2d(ELEM3_x, ELEM3_y, Math.toRadians(ELEM3_heading));
        Pose2d HUMAN = new Pose2d(HUMAN_x, HUMAN_y, Math.toRadians(HUMAN_heading));
        Pose2d HUMAN2 = new Pose2d(HUMAN2_x, HUMAN2_y, Math.toRadians(HUMAN2_heading));


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        oriz.goToPos(0.4);
        claw.close();
        joint.goToLevel();

        waitForStart();
        lift.goToHigh();

        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineToLinearHeading(PRELOAD)
                        .addTemporalMarker( () -> {
                            lift.goToMid();
                        })
                        .waitSeconds(0)
                        .waitSeconds(0.3)
                .addTemporalMarker( () -> {
                    claw.open();
                })
                .waitSeconds(0.5)
                        .waitSeconds(0)
                .waitSeconds(0.5)
                    .addTemporalMarker( () -> {
                        lift.goDown();
                     })
                    .waitSeconds(0)
                .lineToLinearHeading(SAFE)
                .lineToLinearHeading(SAFE2)
                .lineToLinearHeading(ELEM1)
                .lineToLinearHeading(HUMAN)
                .lineToLinearHeading(SAFE2)
                .lineToLinearHeading(ELEM2)
                .lineToLinearHeading(HUMAN)
                .build());


        while(opModeIsActive()) {
            lift.update();
            drive.update();
        }




    }
}
