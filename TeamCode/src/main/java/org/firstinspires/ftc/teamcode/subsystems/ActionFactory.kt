package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.autonomous.trajectories.TrajectoryFactory
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.arm
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.claw
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intake
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intakeExtension
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intakePivot
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.lift
import page.j5155.expressway.actions.RaceParallelAction

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

    var usePID = true
    
    /**
     * This action should be run in parallel (using a RaceAction) with the other actions.
     */
    val motorManagementAction: Action
        get() = if(usePID) ParallelAction(
            lift.update(),
            intakeExtension.update()
        ) else ParallelAction()
    
    val disablePID: Action
        get() = Action { 
            usePID = false
            false 
        }
    
    val enablePID: Action
        get() = Action {
            usePID = true
            false
        }

    /**
     * Resets the motor encoders
     */
    fun resetEncoders(): Action {
        return ParallelAction(
            lift.resetEncoder(),
            intakeExtension.resetEncoder()
        )
    }
    //endregion
    
    //region Samples
    val preloadSample: Action
        get() = ParallelAction(
            TrajectoryFactory.leftStartToHighBasket,
            SequentialAction(
                intakeExtension.toSlightlyOut(),
                intakeExtension.toIn()
            ),
            lift.toHigh(),
            SequentialAction(
                SleepAction(1.5),
                arm.toScore()
            )
        )
    
    val highBasketScoreToAscent: Action
        get() = ParallelAction(
            TrajectoryFactory.highBasketToAscentPark,
            lift.toIntake(),
            arm.toAscentOne()
        )
    
    val inToTransfer: Action
        get() = SequentialAction(
            ParallelAction(
                intakePivot.toUp(),
                lift.toALittleHigh(),
                claw.toOpen(),
                SequentialAction(
                    SleepAction(0.5),
                    arm.toIntake()
                )
            ),
            intakeExtension.toIn(),
            intakePivot.toTransfer(),
            lift.toIntake(),
            claw.toClose()
        )
    
    val rightFloorSample: Action
        get() = SequentialAction(
            ParallelAction(
                SequentialAction(
                    intakePivot.toUp(),
                    intakeExtension.toSlightlyOut(),
                    intakePivot.toDown()
                ),
                intake.start(),

                TrajectoryFactory.highBasketToRightSample
            ),
            RaceParallelAction(
                intake.runUntilSampleInIntake(),
                TrajectoryFactory.pickupRightSample
            ),
            intake.stop(),
            ParallelAction(
                SequentialAction(
                    inToTransfer,
                    ParallelAction(
                        SequentialAction(
                            intakeExtension.toSlightlyOut(),
                            intakeExtension.toIn()
                        ),
                        lift.toHigh(),
                        SequentialAction(
                            intake.reverse(),
                            SleepAction(1.5),
                            arm.toScore()
                        )
                    )
                ),
                TrajectoryFactory.rightSampleToHighBasket
            ),
            intake.stop()
        )
    
    val centerFloorSample: Action
        get() = SequentialAction(
            ParallelAction(
                SequentialAction(
                    intakePivot.toUp(),
                    intakeExtension.toSlightlyOut(),
                    intakePivot.toDown()
                ),
                intake.start(),

                TrajectoryFactory.highBasketToCenterSample
            ),
            RaceParallelAction(
                intake.runUntilSampleInIntake(),
                TrajectoryFactory.pickupCenterSample
            ),
            intake.stop(),
            ParallelAction(
                SequentialAction(
                    inToTransfer,
                    ParallelAction(
                        SequentialAction(
                            intakeExtension.toSlightlyOut(),
                            intakeExtension.toIn()
                        ),
                        lift.toHigh(),
                        SequentialAction(
                            intake.reverse(),
                            SleepAction(1.5),
                            arm.toScore()
                        )
                    )
                ),
                TrajectoryFactory.centerSampleToHighBasket
            ),
            intake.stop()
        )
    
    val leftFloorSample: Action
        get() = SequentialAction(
            ParallelAction(
                SequentialAction(
                    intakePivot.toUp(),
                    intakeExtension.toMiddlePos(),
                    intakePivot.toDown()
                ),
                intake.start(),
                TrajectoryFactory.highBasketToLeftSample
            ),
            RaceParallelAction(
                intake.runUntilSampleInIntake(),
                TrajectoryFactory.leftSampleToLeftSamplePickup
            ),
            intake.stop(),
            ParallelAction(
                SequentialAction(
                    inToTransfer,
                    ParallelAction(
                        SequentialAction(
                            intakeExtension.toSlightlyOut(),
                            intakeExtension.toIn()
                        ),
                        lift.toHigh(),
                        SequentialAction(
                            intake.reverse(),
                            SleepAction(2.0),
                            arm.toScore()
                        )
                    )
                ),
                TrajectoryFactory.leftSampleToHighBasket
            ),
            intake.stop()
        )
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