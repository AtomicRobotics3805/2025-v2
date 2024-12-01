package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem
import page.j5155.expressway.ftc.ActionLinearOpMode
import page.j5155.expressway.ftc.GamepadActionRunner

class GamepadExExampleOpMode: ActionLinearOpMode(GamepadActionRunner()) {
    
    val arm = ArmSubsystem(hardwareMap)
    
    override fun runOpMode() {
        when (val gamepadRunner = runner as GamepadActionRunner) {
            else -> {
                // MUST BE CALLED FIRST
                gamepadRunner.initialize(gamepad1, gamepad2)

                gamepadRunner.gamepad1.x.onPressActionMap = arm.toIntake()
                gamepadRunner.gamepad2.y.onPressActionMap = arm.toScore()

                waitForStart()

                while (opModeIsActive()) {
                    (runner as GamepadActionRunner).update()
                }
            }
        }
    }
}