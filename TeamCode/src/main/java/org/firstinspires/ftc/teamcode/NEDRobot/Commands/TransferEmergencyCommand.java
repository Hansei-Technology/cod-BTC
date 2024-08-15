package org.firstinspires.ftc.teamcode.NEDRobot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.ActiveStateCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.BucketPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.CapacPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.ExtendoPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.LinkagePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.PitchPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.TriggerPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.WristPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class TransferEmergencyCommand extends SequentialCommandGroup {
    public TransferEmergencyCommand(Obot obotV2){
        super(
                new SequentialCommandGroup(
                        new ExtendoPosCommand(obotV2, IntakeSubsystem.ExtendoState.HOME),
                        new WaitCommand(200),
                        new WristPosCommand(obotV2, IntakeSubsystem.WristState.TRANSFER),
                        new ActiveStateCommand(obotV2, IntakeSubsystem.ActiveState.STOP),
                        new WaitCommand(200),
                        new InstantCommand(() -> obotV2.Intake.setMotionProfileTargetPosition(-50)),
                        new WaitCommand(300),
                        new InstantCommand(() -> obotV2.Intake.setMotionProfileTargetPosition(0)),
                        new WaitCommand(200),
                        new CapacPosCommand(obotV2, IntakeSubsystem.CapacState.OPENED),
                        new WaitCommand(700),
                        new PitchPosCommand(obotV2, LiftSubsystem.PitchState.HOME),
                        new WaitCommand(700),
                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.HOME),
                        new WaitCommand(700),
                        new PitchPosCommand(obotV2, LiftSubsystem.PitchState.TRANSFER),
                        new WaitCommand(500),
                        new LinkagePosCommand(obotV2, LiftSubsystem.LinkageState.TRANSFER),
                        new WaitCommand(700),
                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.TRANSFER),
                        new WaitCommand(1000),
                        new TriggerPosCommand(obotV2, LiftSubsystem.TriggerState.CLOSE)
                )
        );
    }
}