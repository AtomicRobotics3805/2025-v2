package org.firstinspires.ftc.teamcode.autonomous.trajectories

import android.content.pm.ActivityInfo
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtensionSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.arm
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.claw
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intake
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intakeExtension
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intakePivot
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.lift

object ActionFactory {
    //region Base
    val autonomousWithSampleInitRoutine: Action
        get() = ParallelAction(
            intakeExtension.update(),
            
            arm.toIntake(),
            SequentialAction(
                intakePivot.toTransfer(),
                claw.toClose()
            )
        )
    
    val autonomousWithSpecInitRoutine: Action
        get() = ParallelAction(
            intakeExtension.update(),
            
            SequentialAction(
                intakePivot.toUp(),
                claw.toClose()
            )
        )
    
    val scoreToIntake: Action
        get() = SequentialAction(
            claw.toOpen(),
            ParallelAction(
                arm.toIntake(),
                lift.toALittleHigh()
            ),
            intake.start()
        )
    
    val scoreToBottom: Action
        get() = SequentialAction(
            claw.toOpen(),
            ParallelAction(
                arm.toIntake(),
                lift.toIntake()
            )
        )
    
    val scoreToSpecPickup: Action
        get() = SequentialAction(
            claw.toOpen(),
            ParallelAction(
                arm.toSpecimenPickup(),
                lift.toSpecimenPickup()
            )
        )

    /**
     * This action should be run in parallel (using a RaceAction) with the other actions.
     */
    val motorManagementAction: Action
        get() = ParallelAction(
            lift.update(),
            intakeExtension.update()
        )

    /**
     * Resets the motor encoders
     */
    fun resetEncoders(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                lift.motor1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                lift.motor2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                lift.motor1PIDF.targetPosition = 0.0
                lift.motor2PIDF.targetPosition = 0.0
                lift.motor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lift.motor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                return false
            }
        }
    }
    //endregion
    
    //region Specimens
    val rightStartToFirstSpec: Action
        get() = ParallelAction(
            SequentialAction(
                SleepAction(0.2),
                TrajectoryFactory.rightStartToHighChamber1
            ),
            SequentialAction(
                SleepAction(1.25),
                claw.toOpen()
            ),
            lift.toFirstAutonSpecimenScoreHigh(),
            arm.toSpecimenScore(),
            SequentialAction(
                intakeExtension.toMiddlePos(),
                intakeExtension.toIn()
            )
        )
    
    val firstSpecToPushSamples: Action
        get() = ParallelAction(
            SequentialAction(
                TrajectoryFactory.highChamber1ToBringLeftSampleToObservationZone,
                TrajectoryFactory.observationZoneLeftToBringCenterSampleToObservationZone
            ),
            SequentialAction(
                SleepAction(1.0),
                arm.toSpecimenPickup()
            ),
            lift.toSpecimenPickup(),
            claw.toSpecimenOpen()
        )
    
    val pushToGrabSecondSpec: Action
        get() = SequentialAction(
            TrajectoryFactory.observationZoneMiddleToSpecimenPickupPosition,
            claw.toClose()
        )
    
    val secondSpec: Action
        get() = ParallelAction(
            TrajectoryFactory.specimenPickupPositionToHighChamber2,
            SequentialAction(
                SleepAction(2.9),
                ParallelAction(
                    claw.toSpecimenOpen(),
                    arm.toSpecimenPickup()
                )
            ),
            lift.toAutonSpecimenScoreHigh(),
            arm.toSpecimenScore(),
            intakeExtension.toIn()
        )
    
    val thirdSpec: Action
        get() = ParallelAction(
            SequentialAction(
                TrajectoryFactory.highChamber2ToSpecimenPickupPosition,
                claw.toClose(),
                ParallelAction(
                    TrajectoryFactory.specimenPickupPositionToHighChamber3,
                    SequentialAction(
                        SleepAction(2.9),
                        claw.toOpen()
                    ),
                    lift.toAutonSpecimenScoreHigh(),
                    arm.toSpecimenScore()
                )
            ),
            SequentialAction(
                SleepAction(1.0),
                lift.toSpecimenPickup(),
                claw.toSpecimenOpen()
            )
        )

    val fourthSpec: Action
        get() = ParallelAction(
            SequentialAction(
                TrajectoryFactory.highChamber3ToSpecimenPickupPosition,
                claw.toClose(),
                ParallelAction(
                    TrajectoryFactory.specimenPickupPositionToHighChamber4,
                    SequentialAction(
                        SleepAction(2.9),
                        claw.toOpen()
                    ),
                    lift.toAutonSpecimenScoreHigh(),
                    arm.toSpecimenScore()
                )
            ),
            SequentialAction(
                SleepAction(1.0),
                lift.toSpecimenPickup(),
                claw.toSpecimenOpen()
            )
        )

    val thirdSpecToPark: Action
        get() = ParallelAction(
            TrajectoryFactory.highChamber3ToPark,
            intakeExtension.toIn(),
            SequentialAction(
                claw.toOpen(),
                ParallelAction(
                    arm.toIntake(),
                    lift.toIntake()
                ),
                intakeExtension.toZero(),
                SleepAction(0.5)
            )
        )
    //endregion
}