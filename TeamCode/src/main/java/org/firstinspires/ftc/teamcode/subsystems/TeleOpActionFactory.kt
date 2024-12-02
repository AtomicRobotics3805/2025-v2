package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.arm
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.claw
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intake
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intakeExtension
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.intakePivot
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager.lift

object TeleOpActionFactory {
    val outToIntake: Action
        get() = ParallelAction(
            SequentialAction(
                ParallelAction(
                    intakePivot.toUp(),
                    intakeExtension.toOut()
                ),
                intakePivot.toDown()
            ),
            intake.runUntilSampleInIntake()
        )
    
    val slightlyOutToIntake: Action
        get() = ParallelAction(
            SequentialAction(
                ParallelAction(
                    intakePivot.toUp(),
                    intakeExtension.toSlightlyOut()
                ),
                intakePivot.toDown()
            ),
            intake.runUntilSampleInIntake()
        )
    
    val inToTransfer: Action
        get() = SequentialAction(
            ParallelAction(
                intake.stop(),
                intakePivot.toUp(),
                lift.toALittleHigh(),
                claw.toOpen(),
                SequentialAction(
                    SleepAction(0.5),
                    arm.toIntake()
                )
            ),
            ParallelAction(
                SequentialAction(
                    SleepAction(0.5),
                    lift.toIntake()
                ),
                intakeExtension.toIn(),
                intakePivot.toTransfer()
            ),
            claw.toClose()
        )
    
    val liftUp: Action
        get() = ParallelAction(
            SequentialAction(
                intakeExtension.toSlightlyOut(),
                intakeExtension.toIn()
            ),
            lift.toHigh(),
            intakePivot.toUp(),
            SequentialAction(
                SleepAction(1.25),
                arm.toScore()
            )
        )
    
    val scoreAndRepeat: Action
        get() = SequentialAction(
            claw.toOpen(),
            ParallelAction(
                arm.toIntake(),
                lift.toIntake()
            )
        )
    
    val toSpecimenScore: Action
        get() = SequentialAction(
            claw.toClose(),
            ParallelAction(
                lift.toSpecimenScoreHigh(),
                arm.toSpecimenScore()
            )
        )
    
    val toHang: Action
        get() = ParallelAction(
            lift.toHangPos(),
            arm.toSpecimenScore()
        )
    
    val toSpecimenPickup: Action
        get() = ParallelAction(
            arm.toSpecimenPickup(),
            claw.toSpecimenOpen(),
            lift.toSpecimenPickup()
        )
    
    val reset: Action
        get() = SequentialAction(
            ParallelAction(
                intakePivot.toUp(),
                arm.toIntake(),
                claw.toOpen()
            ),
            intakeExtension.toIn(),
            lift.toIntake()
        )
}