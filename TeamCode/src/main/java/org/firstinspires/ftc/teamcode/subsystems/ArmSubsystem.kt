package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class ArmSubsystem(hardwareMap: HardwareMap) {
    
    var intakePos = 0.05
    var scorePos = 0.69
    var ascentOnePos = 0.69
    var specimenPickupPos = 0.97
    var specimenScorePos = 0.7

    val servo: Servo = hardwareMap.get(Servo::class.java, "arm")

    /**
     * Moves the servo to the intakePos
     */
    fun toIntake(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo.position = intakePos
                return false
            }
        }
    }

    /**
     * Moves the servo to the scorePos
     */
    fun toScore(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo.position = scorePos
                return false
            }
        }
    }

    /**
     * Moves the servo to the ascentOnePos
     */
    fun toAscentOne(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo.position = ascentOnePos
                return false
            }
        }
    }

    /**
     * Moves the servo to the specimenPickupPos
     */
    fun toSpecimenPickup(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo.position = specimenPickupPos
                return false
            }
        }
    }

    /**
     * Moves the servo to the specimenScorePos
     */
    fun toSpecimenScore(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo.position = specimenScorePos
                return false
            }
        }
    }

}