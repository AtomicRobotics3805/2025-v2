package org.firstinspires.ftc.teamcode.util

/**
 * This is an OpMode wrapper that lets you use a org.firstinspires.ftc.teamcode.util.GamepadActionRunner (and its associated features) without dispersing
 * casts throughout your code.
 * @param runner a org.firstinspires.ftc.teamcode.util.GamepadActionRunner (you don't need to override this, it handles that automatically)
 */
abstract class GamepadActionLinearOpMode(): ActionLinearOpMode() {
    override lateinit var runner: GamepadActionRunner
    
    fun initialize() {
        runner = GamepadActionRunner(gamepad1, gamepad2)
    }
}