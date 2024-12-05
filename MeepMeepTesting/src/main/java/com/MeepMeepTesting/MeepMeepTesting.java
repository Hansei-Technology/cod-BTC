package com.MeepMeepTesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static double PRELOAD_x = 14, PRELOAD_y = 0, PRELOAD_heading = 0;
    public static double SAFE_x = 23, SAFE_y = -18, SAFE_heading = 180;
    public static double SAFE2_x = 36, SAFE2_y = -23, SAFE2_heading = 180;
    public static double ELEM1_x = 25, ELEM1_y = -24, ELEM1_heading = 180;
    public static double HUMAN_x = 3, HUMAN_y = -24, HUMAN_heading = 180;
    public static double SAFE3_x = 10, SAFE3_y = -24, SAFE3_heading = 180;
    public static double SPECIMEN_x = 2, SPECIMIEN_y = -24  , SPECIMEN_heading = 180;
    public static double PRELOAD2_x = 14, PRELOAD2_y = 0, PRELOAD2_heading = 0;
    public static double ELEM2_x = 25, ELEM2_y = -30, ELEM2_heading = 180;
    public static double HUMAN2_x = 3, HUMAN2_y = -30, HUMAN2_heading = 180;
    public static double ELEM3_x = 25, ELEM3_y = -33, ELEM3_heading = 270;
    public static double HUMAN3_x = 2, HUMAN3_y = -33, HUMAN3_heading = 270;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d PRELOAD = new Pose2d(PRELOAD_x, PRELOAD_y, Math.toRadians(PRELOAD_heading));
        Pose2d SAFE = new Pose2d(SAFE_x, SAFE_y, Math.toRadians(SAFE_heading));
        Pose2d SAFE2 = new Pose2d(SAFE2_x, SAFE2_y, Math.toRadians(SAFE2_heading));
        Pose2d ELEM1 = new Pose2d(ELEM1_x, ELEM1_y, Math.toRadians(ELEM1_heading));
        Pose2d HUMAN = new Pose2d(HUMAN_x, HUMAN_y, Math.toRadians(HUMAN_heading));
        Pose2d SAFE3 = new Pose2d(SAFE3_x, SAFE3_y, Math.toRadians(SAFE3_heading));
        Pose2d SPECIMEN = new Pose2d(SPECIMEN_x, SPECIMIEN_y, Math.toRadians(SPECIMEN_heading));
        Pose2d PRELOAD2 = new Pose2d(PRELOAD2_x, PRELOAD2_y, Math.toRadians(PRELOAD2_heading));
        Pose2d ELEM2 = new Pose2d(ELEM2_x, ELEM2_y, Math.toRadians(ELEM2_heading));
        Pose2d HUMAN2 = new Pose2d(HUMAN2_x, HUMAN2_y, Math.toRadians(HUMAN2_heading));
        Pose2d ELEM3 = new Pose2d(ELEM3_x, ELEM3_y, Math.toRadians(ELEM3_heading));
        Pose2d HUMAN3 = new Pose2d(HUMAN3_x, HUMAN3_y, Math.toRadians(HUMAN3_heading));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(PRELOAD)
                                .waitSeconds(0)
                                .waitSeconds(0.4)
                                .waitSeconds(0)
                                .waitSeconds(0.5)
                                .waitSeconds(0)
                                .waitSeconds(0.5)
                                .waitSeconds(0)
                                .setReversed(true)
                                .splineToLinearHeading(SAFE, 270)
                                .lineToLinearHeading(ELEM1)
                                .lineToLinearHeading(HUMAN)
                                .lineToLinearHeading(ELEM1)
                                .lineToLinearHeading(ELEM2)
                                .lineToLinearHeading(HUMAN2)
                                .lineToLinearHeading(ELEM2)
                                .lineToLinearHeading(ELEM3)
                                .lineToLinearHeading(HUMAN3)
                                .lineToLinearHeading(SAFE3)
                                .waitSeconds(1.5)
                                .lineToLinearHeading(SPECIMEN)
                                .waitSeconds(0.1)
                                .waitSeconds(0)
                                .lineToLinearHeading(PRELOAD2)
                                .waitSeconds(0)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(SAFE3)
                                .waitSeconds(1.5)
                                .lineToLinearHeading(SPECIMEN)
                                .waitSeconds(0.1)
                                .waitSeconds(0)
                                .lineToLinearHeading(PRELOAD2)
                                .waitSeconds(0)
                                .waitSeconds(0.5)
                                .build()
                                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
