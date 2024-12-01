package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class IntakeSubsystem(hardwareMap: HardwareMap) {
    
    var intakeSpeed = 1.0
    
    var reverseSpeed = 0.0
    
    val servo1: Servo = hardwareMap.get(Servo::class.java, "intake")
    val servo2: Servo = hardwareMap.get(Servo::class.java, "intake2")
    
    init {
        servo1.direction = Servo.Direction.REVERSE
    }

    /**
     * Starts the intake wheels spinning
     */
    fun start(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo1.position = intakeSpeed
                servo2.position = intakeSpeed
                return false
            }
        }
    }

    /**
     * Stops the intake wheels
     */
    fun stop(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo1.position = 0.5
                servo2.position = 0.5
                return false
            }
        }
    }

    /**
     * Starts the intake wheels spinning in reverse
     */
    fun reverse(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo1.position = reverseSpeed
                servo2.position = reverseSpeed
                return false
            }
        }
    }

    /**
     * Runs the intake until it senses there is a sample
     */
    fun runUntilSampleInIntake(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                servo1.position = intakeSpeed
                servo2.position = intakeSpeed

                return SubsystemManager.intakeSensor.currentSample == IntakeSensorSubsystem.SampleColors.NONE
            }
        }
    }
}