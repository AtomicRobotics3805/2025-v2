package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.configuration.MecanumDriveConstants.reverseStrafe
import org.firstinspires.ftc.teamcode.configuration.MecanumDriveConstants.reverseStraight
import org.firstinspires.ftc.teamcode.configuration.MecanumDriveConstants.reverseTurn
import org.firstinspires.ftc.teamcode.subsystems.ActionFactory
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.arm
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.claw
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intake
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intakeExtension
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intakePivot
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.lift
import org.firstinspires.ftc.teamcode.subsystems.TeleOpActionFactory
import org.firstinspires.ftc.teamcode.util.GamepadActionLinearOpMode
import org.firstinspires.ftc.teamcode.util.rad
import org.firstinspires.ftc.teamcode.util.toDegrees
import kotlin.math.cos
import kotlin.math.sin

@TeleOp(name = "Competition TeleOp")
class CompetitionTeleOp: GamepadActionLinearOpMode() {
    lateinit var drive: MecanumDrive

    var speed = 1.0
    
    val speedManager = DriveSpeedManager()
    
    val specimenActionAlternator = AlternateActions({ TeleOpActionFactory.toSpecimenPickup }, { TeleOpActionFactory.toSpecimenScore })
    
    val timer = ElapsedTime()
    val loopList = mutableListOf<Double>()
    
    val actionCountOverTime = mutableListOf<Int>()
    
    override fun runOpMode() {
        
        initialize()
        
        // We don't want to initialize a new MecanumDrive, since that will reset our heading. Instead, use the same one (if it exists)
        drive = if (SubsystemManager.driveIsInitialized()) {
            // Drive is already initialized. We can set our drive variable to the initialized one.
            SubsystemManager.drive
        } else {
            // Otherwise we need to initialize a new one. Let's assume it'll face forward. 
            MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 90.rad))
        }
        
        // Initialize mechanisms if needed.
        if (!SubsystemManager.mechanismsAreInitialized()) {
            SubsystemManager.initialize(hardwareMap, drive)
        }
        
        // We want to control our lift and intake extension, so run our PIDFs
        runner.runAsync(ActionFactory.motorManagementAction)
        
        // Setup telemetry
        telemetry.msTransmissionInterval = 50
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        
        // Create mappings for initialization
        createInitMappings()

        // While stop hasn't been requested, run the runner
        while (!isStopRequested && !opModeIsActive()) {
            val packet = TelemetryPacket()
            runner.update()
            runner.updateAsync(packet)

            FtcDashboard.getInstance().sendTelemetryPacket(packet)
            
            telemetry.update()
        }

        // Map Actions
        createMappings()
        
        while (opModeIsActive() && !isStopRequested) {
            val packet = TelemetryPacket()
            
            runner.update()
            runner.updateAsync(packet)
            
            // Drive control
            handleDriving()
            
            // Loop timer
            loopList.add(timer.milliseconds())
            timer.reset()
            
            while (loopList.size > 1000) {
                loopList.removeAt(0)
            }
            
            // Action counter
            actionCountOverTime.add(runner.runningActions.size)
            
            while (actionCountOverTime.size > 1000) {
                loopList.removeAt(0)
            }
            
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
            
            telemetry.addLine("-----Motors-----")
            telemetry.addData("Lift target", lift.motor1PIDF.targetPosition)
            telemetry.addData("Lift position", lift.motor1.currentPosition)
            telemetry.addData("Lift power", lift.motor1.power)
            telemetry.addLine()
            telemetry.addData("Extendo target", intakeExtension.motorPIDF.targetPosition)
            telemetry.addData("Extendo position", intakeExtension.motor.currentPosition)
            telemetry.addData("Extendo power", intakeExtension.motor.power)
            telemetry.addLine()
            telemetry.addLine("-----Servos-----")
            telemetry.addData("Arm pos", arm.servo.position)
            telemetry.addData("Claw pos", claw.servo.position)
            telemetry.addData("Intake pivot pos", intakePivot.servo.position)
            telemetry.addData("Intake power", (intake.servo1.position * 2) - 1)
            telemetry.addLine()
            telemetry.addLine("-----Drivetrain-----")
            telemetry.addData("X", drive.pose.position.x)
            telemetry.addData("Y", drive.pose.position.y)
            telemetry.addData("Heading", drive.pose.heading.toDouble().toDegrees)
            telemetry.addLine()
            telemetry.addLine("-----Loop Timer-----")
            telemetry.addData("Last loop time MS", loopList.last())
            telemetry.addData("Last loop time HZ", 1000.0 / loopList.last())
            telemetry.addData("Average loop time", loopList.average())
            telemetry.addData("Average loop time HZ", 1000.0 / loopList.average())
            telemetry.addData("# of actions running", runner.runningActions.size)
            telemetry.addData("Average # of actions running", actionCountOverTime.average())
            telemetry.addData("Average # of actions running per second", actionCountOverTime.average() * (1000.0 / loopList.average()))
            
            telemetry.update()
            timer.reset()
        }
    }
    
    fun createInitMappings() {
        runner.gamepad1.a.onPressActionMap = { ActionFactory.resetEncoders() }
        runner.gamepad1.b.onPressActionMap = { ActionFactory.disablePID }
        runner.gamepad1.b.onReleaseActionMap = { ActionFactory.enablePID }
    }
    
    fun createMappings() {
        // Reset init mappings
        runner.gamepad1.a.onPressActionMap = null
        runner.gamepad1.b.onPressActionMap = null
        runner.gamepad1.b.onReleaseActionMap = null
        
        // Slowmode
        runner.gamepad1.rightBumper.onPressActionMap = { speedManager.toggleDriveSpeed() }
        runner.gamepad1.rightBumper.onReleaseActionMap = { speedManager.toggleDriveSpeed() }
        
        // Resets
        runner.gamepad2.leftStick.button.onPressActionMap = { lift.resetEncoder() }
        runner.gamepad1.x.onPressActionMap = { resetRotation() }
        
        runner.gamepad2.y.onPressActionMap = { specimenActionAlternator.runNext() }
        
        // Sample
        runner.gamepad1.a.onPressActionMap = { TeleOpActionFactory.outToIntake }
        runner.gamepad1.leftBumper.onPressActionMap = { TeleOpActionFactory.slightlyOutToIntake }
        runner.gamepad2.b.onPressActionMap = { TeleOpActionFactory.inToTransfer }
        runner.gamepad2.x.onPressActionMap = { TeleOpActionFactory.liftUp }
        runner.gamepad2.a.onPressActionMap = { claw.toOpen() }
        
        // Intake
        runner.gamepad1.y.onPressActionMap = { intake.stop() }
        runner.gamepad1.rightTrigger.onPressActionMap = { intake.start() }
        runner.gamepad1.leftTrigger.onPressActionMap = { intake.reverse() }
        
        // Hanging
        runner.gamepad1.dpadUp.onPressActionMap = { TeleOpActionFactory.toHang }
        runner.gamepad1.dpadDown.onPressActionMap = { lift.toIntake() }
        
        // TODO manual lift controls
        // TODO manual extension control
    }
    
    fun handleDriving() {
        val x: Double = (if(reverseStraight) -gamepad1.left_stick_y else gamepad1.left_stick_y).toDouble()
        val y: Double = (if(reverseStrafe) -gamepad1.left_stick_x else gamepad1.left_stick_x).toDouble()
        val rx: Double = (if(reverseTurn) -gamepad1.right_stick_x else gamepad1.right_stick_x).toDouble()

        val heading = drive.pose.heading.toDouble()
        val rotX = x * cos(-heading) - y * sin(-heading)
        val rotY = x * sin(-heading) + y * cos(-heading)
        
        drive.setDrivePowers(PoseVelocity2d(
            Vector2d(
                rotX,
                rotY
            ),
            rx
        ))
    }
    
    inner class DriveSpeedManager {
        val driveSpeeds = listOf(1.0, 0.5)
        var index = 0
        
        fun toggleDriveSpeed(): Action {
            return ToggleDriveSpeed()
        }
        
        inner class ToggleDriveSpeed: Action {
            override fun run(p: TelemetryPacket): Boolean {
                index++
                if (index >= driveSpeeds.size) {
                    index = 0
                }
                
                speed = driveSpeeds[index]
                return false
            }
        }

    }
    
    inner class AlternateActions(private val action1: () -> Action, private val action2: () -> Action) {
        var current = 0
        
        fun runNext(): Action {
            if (current == 0) {
                current = 1
                return action1.invoke()
            } else {
                current = 0
                return action2.invoke()
            }
        }
    }
    
    fun resetRotation(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                drive.pose = Pose2d(drive.pose.position, 270.rad)
                return false
            }
        }
    }
}