package org.firstinspires.ftc.teamcode.TeamCode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
@Config
public class Auto2Specimene extends LinearOpMode {

    public static double PRELOAD_x = 31, PRELOAD_y = 0, PRELOAD_heading = 0;
    public static double SAFE_x = 11.7, SAFE_y = -37, SAFE_heading = 270;
    public static double SAFE2_x = 50, SAFE2_y = -37, SAFE2_heading = 180;
    public static double ELEM1_x = 50, ELEM1_y = -48, ELEM1_heading = 180;
    public static double HUMAN_x = 5, HUMAN_y = -65, HUMAN_heading = 190;
    public static double SAFE3_x = 19, SAFE3_y = -65, SAFE3_heading = 180;
    public static double SPECIMEN_x = 5, SPECIMIEN_y = -65  , SPECIMEN_heading = 180;
    public static double PRELOAD2_x = 34.5, PRELOAD2_y = 0, PRELOAD2_heading = 0;
    //public static double clesteInchis = 0.7;
    //public static double clesteDeschis = 0.3;
    //public Servo motorCleste;

    public void runOpMode() throws InterruptedException{



        Oriz oriz = new Oriz(hardwareMap);
        Vert vert = new Vert(hardwareMap);
        //motorCleste = hardwareMap.get(Servo.class, "s3");
        Cleste1 cleste1 = new Cleste1(hardwareMap);
        Cleste2 cleste2 = new Cleste2(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d PRELOAD = new Pose2d(PRELOAD_x, PRELOAD_y, Math.toRadians(PRELOAD_heading));
        Pose2d SAFE = new Pose2d(SAFE_x, SAFE_y, Math.toRadians(SAFE_heading));
        Pose2d SAFE2 = new Pose2d(SAFE2_x, SAFE2_y, Math.toRadians(SAFE2_heading));
        Pose2d ELEM1 = new Pose2d(ELEM1_x, ELEM1_y, Math.toRadians(ELEM1_heading));
        Pose2d HUMAN = new Pose2d(HUMAN_x, HUMAN_y, Math.toRadians(HUMAN_heading));
        Pose2d SAFE3 = new Pose2d(SAFE3_x, SAFE3_y, Math.toRadians(SAFE3_heading));
        Pose2d SPECIMEN = new Pose2d(SPECIMEN_x, SPECIMIEN_y, Math.toRadians(SPECIMEN_heading));
        Pose2d PRELOAD2 = new Pose2d(PRELOAD2_x, PRELOAD2_y, Math.toRadians(PRELOAD2_heading));


        oriz.goToPoz(0.4);
        cleste2.goToLevel();
        cleste1.close();
        //motorCleste.setPosition(0.7);

        waitForStart();

        cleste1.setStatus(Cleste1.Status.INCHIS);

        vert.goToHigh();

        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .waitSeconds(0.3)
                .lineToLinearHeading(PRELOAD)
                .addTemporalMarker( () -> {
                    vert.goToMid();
                })
                .waitSeconds(0)
                .waitSeconds(0.4)
                        .waitSeconds(0)
                        .addTemporalMarker( () -> {
                            cleste1.open();
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
                        .lineToLinearHeading(SAFE3)
                        .waitSeconds(1.5)
                        .lineToLinearHeading(SPECIMEN)
                        .addTemporalMarker( () -> {
                            vert.goToCapuUrsului();
                        })
                        .addTemporalMarker( () -> {
                            vert.goToHigh();
                        })
                        .waitSeconds(0.1)
                        .waitSeconds(0)
                        .addTemporalMarker( () -> {
                            cleste1.close();
                        })
                        .lineToLinearHeading(PRELOAD2)
                .addTemporalMarker( () -> {
                    vert.goToMid();
                })
                .waitSeconds(0)
                .waitSeconds(0.5)
                        .waitSeconds(0)
                        .addTemporalMarker( () -> {
                            cleste1.open();
                        })
                .waitSeconds(0.5)
                .waitSeconds(0)
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    vert.goDown();
                })
                .build());

        while(opModeIsActive()) {
            vert.update();
            drive.update();
        }

    }
}
