package org.firstinspires.ftc.teamcode.autonomous.opmodes

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.trajectories.ActionFactory
import org.firstinspires.ftc.teamcode.autonomous.trajectories.TrajectoryFactory
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager
import page.j5155.expressway.actions.RaceParallelAction

@Autonomous(name = "Three Specimen Plus Park", group = "specimens")
class ThreeSpecimenPlusPark: LinearOpMode() {

    lateinit var drive: MecanumDrive

    override fun runOpMode() {
        drive = MecanumDrive(hardwareMap, TrajectoryFactory.startPosRight)
        TrajectoryFactory.createTrajectories(drive)
        SubsystemManager.initialize(hardwareMap, drive)
        
        var mainAction = SequentialAction(
            ActionFactory.rightStartToFirstSpec,
            ActionFactory.firstSpecToPushSamples,
            ActionFactory.pushToGrabSecondSpec,
            ActionFactory.secondSpec,
            ActionFactory.thirdSpec,
            ActionFactory.thirdSpecToPark,
            SleepAction(1.0)
        )
        
        runBlocking(
            ActionFactory.resetEncoders()
        )
        waitForStart()
        
        if (isStopRequested) return
        
        runBlocking(
            RaceParallelAction(
                mainAction,
                ActionFactory.motorManagementAction
            )
        )
    }
    
}