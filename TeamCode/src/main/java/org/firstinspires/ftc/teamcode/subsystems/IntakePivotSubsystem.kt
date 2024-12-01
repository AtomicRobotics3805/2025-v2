package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class IntakePivotSubsystem(hardwareMap: HardwareMap) {
    
    var upPos: Double = 0.65
    var transferPos: Double = 0.6
    var downPos: Double = 0.9
    
    val servo: Servo = hardwareMap.get(Servo::class.java, "intake_pivot")

    /**
     * Moves the servo to the upPos
     */
    fun toUp(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo.position = upPos
                return false
            }
        }
    }
    
    /**
     * Moves the servo to the upPos
     */
    fun toTransfer(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo.position = transferPos
                return false
            }
        }
    }
    
    /**
     * Moves the servo to the upPos
     */
    fun toDown(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo.position = downPos
                return false
            }
        }
    }
}