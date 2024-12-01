package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.util.PIDFController


class IntakeExtensionSubsystem(hardwareMap: HardwareMap) {

    var inPos = 69
    var outPos = 1000 // TODO
    var slightlyOutPos = 300 // TODO
    var middlePos = 600

    val motor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "intake_extension")

    val PULLEY_RADIUS = 0.675
    val GEAR_REDUCTION = 1.0
    val TICKS_PER_REV = 384.5
    val COUNTS_PER_INCH = TICKS_PER_REV * GEAR_REDUCTION * (2 / PULLEY_RADIUS * Math.PI)
    
    val motorPIDF: PIDFController = PIDFController(motor, PIDFCoefficients(0.005, 0.0, 0.0, 0.0))
    
    init {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    /**
     * This should run in any OpMode that uses the lift. It is what controls the lift using PIDF controllers
     */
    fun update(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                motorPIDF.update()
                
                p.put("Intake extension motor power", motor.power)
                p.put("Intake extension target position", motorPIDF.targetPosition)
                
                return true
            }
        }
    }

    /**
     * Moves the intake extension to the inPos
     */
    fun toIn(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motorPIDF.targetPosition = inPos * COUNTS_PER_INCH

                // If the motor is *not* within 10 ticks, the action is not complete and should run again
                return !motorPIDF.isWithinDistance(10.0)
            }
        }
    }

    /**
     * Moves the intake extension to the outPos
     */
    fun toOut(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motorPIDF.targetPosition = outPos * COUNTS_PER_INCH

                // If the motor is *not* within 10 ticks, the action is not complete and should run again
                return !motorPIDF.isWithinDistance(10.0)
            }
        }
    }
    
    /**
     * Moves the intake extension to the slightlyOutPos
     */
    fun toSlightlyOut(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motorPIDF.targetPosition = slightlyOutPos * COUNTS_PER_INCH

                // If the motor is *not* within 10 ticks, the action is not complete and should run again
                return !motorPIDF.isWithinDistance(10.0)
            }
        }
    }

    /**
     * Moves the intake extension to the middlePos
     */
    fun toMiddlePos(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motorPIDF.targetPosition = middlePos * COUNTS_PER_INCH

                // If the motor is *not* within 10 ticks, the action is not complete and should run again
                return !motorPIDF.isWithinDistance(10.0)
            }
        }
    }

    /**
     * Zero the position
     */
    fun toZero(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                motorPIDF.targetPosition = 0.0
                
                return !motorPIDF.isWithinDistance(10.0)
            }
        }
    }

}