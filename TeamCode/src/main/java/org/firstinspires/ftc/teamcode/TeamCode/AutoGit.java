package org.firstinspires.ftc.teamcode.TeamCode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class AutoGit extends LinearOpMode {
    public static double PRELOAD_x = 31.2, PRELOAD_y = 0, PRELOAD_heading = 0;
    public static double SAFE_x = 11.7, SAFE_y = -37, SAFE_heading = 270;
    public static double SAFE2_x = 50, SAFE2_y = -37, SAFE2_heading = 180;
    public static double ELEM1_x = 50, ELEM1_y = -48, ELEM1_heading = 180;
    public static double ELEM2_x = 50, ELEM2_y = -62, ELEM2_heading = 180;
    public static double ELEM3_x = 50, ELEM3_y = -67, ELEM3_heading = 180;
    public static double HUMAN_x = 5, HUMAN_y = -67, HUMAN_heading = 190;
    public static double HUMAN2_x = -5, HUMAN2_y = -68, HUMAN2_heading = 180;

    @Override
    public void runOpMode() throws InterruptedException{
        Oriz oriz = new Oriz(hardwareMap);
        Vert vert = new Vert(hardwareMap);
        Cleste cleste = new Cleste(hardwareMap);
        Joint joint = new Joint(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d PRELOAD = new Pose2d(PRELOAD_x, PRELOAD_y, Math.toRadians(PRELOAD_heading));
        Pose2d SAFE = new Pose2d(SAFE_x, SAFE_y, Math.toRadians(SAFE_heading));
        Pose2d SAFE2 = new Pose2d(SAFE2_x, SAFE2_y, Math.toRadians(SAFE2_heading));
        Pose2d ELEM1 = new Pose2d(ELEM1_x, ELEM1_y, Math.toRadians(ELEM1_heading));
        Pose2d ELEM2 = new Pose2d(ELEM2_x, ELEM2_y, Math.toRadians(ELEM2_heading));
        Pose2d ELEM3 = new Pose2d(ELEM3_x, ELEM3_y, Math.toRadians(ELEM3_heading));
        Pose2d HUMAN = new Pose2d(HUMAN_x, HUMAN_y, Math.toRadians(HUMAN_heading));
        Pose2d HUMAN2 = new Pose2d(HUMAN2_x, HUMAN2_y, Math.toRadians(HUMAN2_heading));

        oriz.goToPoz(0.4);
        cleste.close();
        joint.goToLevel();

        waitForStart();

        vert.goToHigh();

        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineToLinearHeading(PRELOAD)
                        .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    vert.goToMid();
                })
                .waitSeconds(0)
                .waitSeconds(0.3)
                .addTemporalMarker( () -> {
                    cleste.open();
                })
                .waitSeconds(0.5)
                .waitSeconds(0)
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    vert.goDown();
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
            vert.update();
            drive.update();
        }
    }
}
