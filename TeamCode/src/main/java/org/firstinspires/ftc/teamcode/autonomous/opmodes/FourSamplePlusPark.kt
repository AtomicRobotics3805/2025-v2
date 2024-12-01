package org.firstinspires.ftc.teamcode.autonomous.opmodes

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.trajectories.ActionFactory
import org.firstinspires.ftc.teamcode.autonomous.trajectories.TrajectoryFactory
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.claw
import page.j5155.expressway.actions.RaceParallelAction

@Autonomous(name = "Four Sample Plus Park", group = "samples")
class FourSamplePlusPark: LinearOpMode() {

    lateinit var drive: MecanumDrive
    
    override fun runOpMode() {
        drive = MecanumDrive(hardwareMap, TrajectoryFactory.startPosLeft)
        TrajectoryFactory.createTrajectories(drive)
        SubsystemManager.initialize(hardwareMap, drive)

        val mainAction = SequentialAction(
            ActionFactory.preloadSample,
            ParallelAction(
                ActionFactory.scoreToIntake,
                ActionFactory.rightFloorSample
            ),
            ParallelAction(
                ActionFactory.scoreToIntake,
                ActionFactory.centerFloorSample
            ),
            ParallelAction(
                ActionFactory.scoreToIntake,
                ActionFactory.leftFloorSample
            ),
            claw.toOpen(),
            ActionFactory.highBasketScoreToAscent,
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