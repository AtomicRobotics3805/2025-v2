package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class ClawSubsystem(hardwareMap: HardwareMap) {
    
    var openPos = 0.15
    var closedPos = 0.35
    var specimenOpenPos = 0.08
    
    val servo: Servo = hardwareMap.get(Servo::class.java, "claw")

    /**
     * Moves the servo to the openPos
     */
    fun toOpen(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo.position = openPos
                return false
            }
        }
    }

    /**
     * Moves the servo to the closedPos
     */
    fun toClose(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo.position = closedPos
                return false
            }
        }
    }
    
    /**
     * Moves the servo to the specimenOpenPos
     */
    fun toSpecimenOpen(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo.position = specimenOpenPos
                return false
            }
        }
    }
}