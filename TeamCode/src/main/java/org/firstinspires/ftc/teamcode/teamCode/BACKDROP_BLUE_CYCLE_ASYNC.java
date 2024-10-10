//package org.firstinspires.ftc.teamcode.teamCode;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
//
//@Autonomous
//public class BACKDROP_BLUE_CYCLE_ASYNC extends LinearOpMode {
//
//    enum State {
//        PRELOAD_LINE,
//        YELLOW_PIXEL,
//        GO_TO_SAFE,
//        GO_TO_STACK,
//        COLLECT,
//        GO_TO_BACKDROP,
//        PLACE_ON_BACKDROP,
//        PARK,
//        IDLE
//    }
//
//    State currentState = State.IDLE;
//    Pose2d startPose = new Pose2d(14, 65, Math.toRadians(-90));
//    ElapsedTime timerBabi;
//    Boolean babi;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        timerBabi = new ElapsedTime();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(startPose);
//        LiftController liftController = new LiftController(hardwareMap);
//        ClawController2Servo clawController = new ClawController2Servo(hardwareMap);
//        JointController2Servo jointController = new JointController2Servo(hardwareMap);
//        ArmController armController  =  new ArmController(hardwareMap);
//        armController.goMid();
//        jointController.goToLevel();
//        clawController.toggleLeft();
//        clawController.toggleRight();
//        liftController.update();
//        armController.update();
//
//        Trajectory preloadLineTraj = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(13, 60, Math.toRadians(-110)))
//                .build();
//        Trajectory yellowPixelTraj = drive.trajectoryBuilder(preloadLineTraj.end())
//                .lineToLinearHeading(new Pose2d(50, 27, Math.toRadians(0)))
//                .build();
//        Trajectory goToSafeTraj = drive.trajectoryBuilder(yellowPixelTraj.end())
//                .lineToLinearHeading(new Pose2d(35, 5, Math.toRadians(0)))
//                .build();
//        Trajectory goToStackTraj = drive.trajectoryBuilder(new Pose2d(goToSafeTraj.end().vec(), Math.toRadians(180)))
//                .lineToLinearHeading(new Pose2d(-46.8, 10.8, Math.toRadians(180)))
//                .build();
//        TrajectorySequence goToBackdropTraj = drive.trajectorySequenceBuilder(goToStackTraj.end())
//                .lineToLinearHeading(new Pose2d(23, 10.8, Math.toRadians(180)))
//                .lineToLinearHeading(new Pose2d(50, 32, Math.toRadians(0)))
//                .build();
//        Trajectory moveToRightCollectTraj = drive.trajectoryBuilder(goToStackTraj.end())
//                .lineToLinearHeading(new Pose2d(goToStackTraj.end().getX(), goToStackTraj.end().getY()-1, Math.toRadians(180)))
//                .build();
//        Trajectory parkTraj = drive.trajectoryBuilder(goToSafeTraj.end())
//                .lineToConstantHeading(new Vector2d(53.3, 16))
//                .build();
//        int cycleNumber = 0;
//        int stackLevel = 5;
//        ElapsedTime timeLeft = new ElapsedTime();
//        waitForStart();
//        timeLeft.reset();
//        if (isStopRequested()) return;
//        currentState = State.PRELOAD_LINE;
//        drive.followTrajectoryAsync(preloadLineTraj);
//        babi = false;
//
//        while (opModeIsActive() && !isStopRequested())
//        {
//            switch (currentState)
//            {
//                case PRELOAD_LINE:
//                {
//                    armController.goDown();
//                    liftController.goTOPos(liftController.MaxPoz);
//                    jointController.goToDown();
//                    if (!drive.isBusy())
//                    {
//                        if(!babi) timerBabi.reset();
//                        clawController.toggleRight();
//                        babi = true;
//                        if(timerBabi.milliseconds() > 200) {
//                            currentState = State.YELLOW_PIXEL;
//                            drive.followTrajectoryAsync(yellowPixelTraj);
//                            armController.goMid();
//                            liftController.goMid();
//                            jointController.goToMid();
//                            babi = false;
//                        }
//
//                    }
//                    break;
//                }
//                case YELLOW_PIXEL:
//                {
//                    if(!drive.isBusy())
//                    {
//                        if(!babi) timerBabi.reset();
//                        babi = true;
//                        clawController.toggleLeft();
//                        if(timerBabi.milliseconds() > 200) {
//                            currentState = State.GO_TO_SAFE;
//                            drive.followTrajectoryAsync(goToSafeTraj);
//                            liftController.goTOPos(0);
//                        }
//
//
//                    }
//                }
//                case GO_TO_SAFE: {
//                    if(!drive.isBusy())
//                    {
////                        if(timeLeft.seconds()>10)
////                        {
////                            currentState = State.GO_TO_STACK;
////                            drive.followTrajectoryAsync(goToStackTraj);
////                        } else {
////                            currentState = State.PARK;
////                            drive.followTrajectoryAsync(parkTraj);
////                        }
//                        drive.turnAsync(Math.toRadians(180));
//                        currentState = State.IDLE;
////                        drive.followTrajectoryAsync(goToStackTraj);
//                        break;
//                    }
//                }
//                case GO_TO_STACK:
//                {
//                    cycleNumber++;
//                    stackLevel = 5 - (2*(cycleNumber-1));
//                    if(!drive.isBusy())
//                    {
//                        clawController.toggleLeft();
//                        clawController.toggleRight();
//                        armController.goToCollect(stackLevel);
//                        currentState = State.COLLECT;
//                    }
//                }
//                case COLLECT:
//                {
//                    if(!drive.isBusy())
//                    {
//                        liftController.goToCollect();
//                        jointController.goToDown();
//                        clawController.toggleLeft();
//                        stackLevel--;
//                        drive.followTrajectoryAsync(moveToRightCollectTraj);
//                        armController.goToCollect(stackLevel);
//                        clawController.toggleRight();
//                        currentState = State.GO_TO_BACKDROP;
//                        liftController.goDown();
//                        armController.goMid();
//                        drive.followTrajectorySequenceAsync(goToBackdropTraj);
//                    }
//                }
//                case GO_TO_BACKDROP:
//                {
//                    if(!drive.isBusy())
//                    {
//                        armController.goMid();
//                        liftController.goMid();
//                        jointController.goToMid();
//                        clawController.toggleLeft();
//                        currentState = State.GO_TO_SAFE;
//                        drive.followTrajectoryAsync(goToSafeTraj);
//                    }
//                }
//                case PARK:
//                {
//                    if(!drive.isBusy())
//                    {
//                        armController.goDown();
//                        liftController.goDown();
//                        jointController.goToDown();
//                    }
//                }
//                case IDLE:
//                    break;
//            }
//
//
//            liftController.update();
//            armController.update();
//            drive.update();
//
//            Pose2d poseEstimate = drive.getPoseEstimate();
//
//
//            // Print pose to telemetry
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
////            telemetry.addData("Time Left", timeLeft.seconds());
//            telemetry.update();
//        }
//    }
//}
