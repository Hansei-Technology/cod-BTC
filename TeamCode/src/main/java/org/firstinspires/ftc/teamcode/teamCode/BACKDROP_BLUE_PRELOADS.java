package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils.CameraDetector;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
public class BACKDROP_BLUE_PRELOADS extends LinearOpMode {

    enum State {
        PRELOAD_LINE,
        YELLOW_PIXEL,
        PARK,
        IDLE
    }

    State currentState = State.IDLE;
    Pose2d startPose = new Pose2d(14, 65, Math.toRadians(180));
    ElapsedTime timerBabi;
    Boolean babi;

    @Override
    public void runOpMode() throws InterruptedException {
        CameraDetector camera = new CameraDetector(OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1")));

        timerBabi = new ElapsedTime();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        LiftController liftController = new LiftController(hardwareMap);
        ClawController clawController = new ClawController(hardwareMap);
        JointController jointController = new JointController(hardwareMap);
        ArmController armController  =  new ArmController(hardwareMap);

        Trajectory preloadLineRightTraj = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(13, 60, Math.toRadians(-110))) //right
                .build();
        Trajectory preloadLineMidTraj = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(14, 51, Math.toRadians(-78))) //mid
                .build();
        Trajectory preloadLineLeftTraj = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(16.5, 60, Math.toRadians(-100))) //left
                .build();

        Trajectory yellowPixelRightTraj = drive.trajectoryBuilder(preloadLineRightTraj.end()) // right
                .lineToLinearHeading(new Pose2d(49, 18.5, Math.toRadians(-6)))
                .build();
        Trajectory yellowPixelMidTraj = drive.trajectoryBuilder(preloadLineMidTraj.end()) //mid
                .lineToLinearHeading(new Pose2d(50, 24, Math.toRadians(0)))
                .build();
        Trajectory yellowPixelLeftTraj = drive.trajectoryBuilder(preloadLineLeftTraj.end()) //left
                .lineToLinearHeading(new Pose2d(43, 28, Math.toRadians(0)))
                .build();

        ElapsedTime timeLeft = new ElapsedTime();

        jointController.goToMid();
        clawController.toggleLeft();
        clawController.toggleRight();

        armController.goToPoz(700);
        liftController.goTOPos(-50);

        CameraDetector.Result result = CameraDetector.Result.CENTER;
        while(opModeInInit()) {
            armController.update();
            liftController.update();
            if(armController.currentPos > 550) {
                jointController.goToPoz(0.87);
                armController.goToPoz(600);
            }

            result = camera.detect();
            telemetry.addLine("Location" + result);
            telemetry.update();
        }
        camera.stop();
        if(result == CameraDetector.Result.NONE) result = CameraDetector.Result.CENTER;

        liftController.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftController.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        timeLeft.reset();
        if (isStopRequested()) return;
        currentState = State.PRELOAD_LINE;
        switch (result) {
            case LEFT:
                drive.followTrajectoryAsync(preloadLineLeftTraj);
                break;
            case RIGHT:
                drive.followTrajectoryAsync(preloadLineRightTraj);
                break;
            case CENTER:
                drive.followTrajectoryAsync(preloadLineMidTraj);
                break;
        }

        babi = false;

        while (opModeIsActive() && !isStopRequested())
        {
            switch (currentState)
            {
                case PRELOAD_LINE:
                {
                    armController.goDown();
                    liftController.goTOPos(liftController.MaxPoz);
                    jointController.goToDown();
                    if (!drive.isBusy())
                    {
                        if(!babi) timerBabi.reset();
                        clawController.openRight();
                        babi = true;
                        if(timerBabi.milliseconds() > 200) {
                            currentState = State.YELLOW_PIXEL;
                            switch (result) {
                                case RIGHT:
                                    drive.followTrajectoryAsync(yellowPixelRightTraj);
                                    break;
                                case LEFT:
                                    drive.followTrajectoryAsync(yellowPixelLeftTraj);
                                    break;
                                case CENTER:
                                    drive.followTrajectoryAsync(yellowPixelMidTraj);
                                    break;
                            }
                            armController.goMid();
                            liftController.goMid();
                            jointController.goToMid();
                            babi = false;
                        }
                    }
                    break;
                }
                case YELLOW_PIXEL:
                {
                    if(!drive.isBusy())
                    {
                        if(!babi) timerBabi.reset();
                        babi = true;
                        clawController.openLeft();
                        if(timerBabi.milliseconds() > 200) {
                            liftController.goDown();
                            currentState = State.PARK;
                            Trajectory parkTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToConstantHeading(new Vector2d(41, 50))
                                    .build();
                            drive.followTrajectoryAsync(parkTraj);

                            liftController.goTOPos(0);
                        }
                    }
                    break;
                }
                case PARK:
                {
                    if(!drive.isBusy())
                    {
                        armController.goMid();
                        jointController.goToDown();
                    }
                    break;
                }
                case IDLE:
                    break;
            }


            liftController.update();
            armController.update();
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();


            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("Time Left", timeLeft.seconds());
            telemetry.update();
        }
    }
}
