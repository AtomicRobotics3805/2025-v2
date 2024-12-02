package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.util.PIDFController


class LiftSubsystem(hardwareMap: HardwareMap) {

    var intakePos = -1.0
    var specimenPickupPos = 1.78
    var highPos = 23.0
    var aLittleHighPos = 3.0
    var specimenScoreHighPos = 4.6
    var specimenAutoScoreHighPos = 4.6
    var firstAutonSpecimenScoreHighPos = 4.2
    var hangPos = 12.0

    val motor1: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "lift")
    val motor2: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "lift2")

    val PULLEY_RADIUS = 0.5
    val GEAR_REDUCTION = 1.0
    val TICKS_PER_REV = 537.7
    val COUNTS_PER_INCH = TICKS_PER_REV * GEAR_REDUCTION * (2 / PULLEY_RADIUS * Math.PI)
    
    val motor1PIDF: PIDFController = PIDFController(motor1, PIDFCoefficients(0.005, 0.0, 0.0, 0.0))
    val motor2PIDF: PIDFController = PIDFController(motor2, PIDFCoefficients(0.005, 0.0, 0.0, 0.0))
    
    init {
        motor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    /**
     * This should run in any OpMode that uses the lift. It is what controls the lift using PIDF controllers
     */
    fun update(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                motor1PIDF.update()
                motor2PIDF.update()
                
                p.put("Lift motor 1 power", motor1.power)
                p.put("Lift motor 2 power", motor2.power)
                p.put("Lift target position", motor1PIDF.targetPosition)
                
                return true
            }
        }
    }

    /**
     * Moves the lift to the intakePos
     */
    fun toIntake(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motor1PIDF.targetPosition = intakePos * COUNTS_PER_INCH
                motor2PIDF.targetPosition = intakePos * COUNTS_PER_INCH

                // If either motor is *not* within 10 ticks, the action is not complete and should run again
                return (!motor1PIDF.isWithinDistance(10.0)) || (!motor2PIDF.isWithinDistance(10.0)) 
            }
        }
    }

    /**
     * Moves the lift to the specimenPickupPos
     */
    fun toSpecimenPickup(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motor1PIDF.targetPosition = specimenPickupPos * COUNTS_PER_INCH
                motor2PIDF.targetPosition = specimenPickupPos * COUNTS_PER_INCH

                // If either motor is *not* within 10 ticks, the action is not complete and should run again
                return (!motor1PIDF.isWithinDistance(10.0)) || (!motor2PIDF.isWithinDistance(10.0))
            }
        }
    }

    /**
     * Moves the lift to the highPos
     */
    fun toHigh(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motor1PIDF.targetPosition = highPos * COUNTS_PER_INCH
                motor2PIDF.targetPosition = highPos * COUNTS_PER_INCH

                // If either motor is *not* within 10 ticks, the action is not complete and should run again
                return (!motor1PIDF.isWithinDistance(10.0)) || (!motor2PIDF.isWithinDistance(10.0))
            }
        }
    }

    /**
     * Moves the lift to the aLittleHighPos
     */
    fun toALittleHigh(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motor1PIDF.targetPosition = aLittleHighPos * COUNTS_PER_INCH
                motor2PIDF.targetPosition = aLittleHighPos * COUNTS_PER_INCH

                // If either motor is *not* within 10 ticks, the action is not complete and should run again
                return (!motor1PIDF.isWithinDistance(10.0)) || (!motor2PIDF.isWithinDistance(10.0))
            }
        }
    }

    /**
     * Moves the lift to the specimenPickupPos
     */
    fun toSpecimenScoreHigh(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motor1PIDF.targetPosition = specimenScoreHighPos * COUNTS_PER_INCH
                motor2PIDF.targetPosition = specimenScoreHighPos * COUNTS_PER_INCH

                // If either motor is *not* within 10 ticks, the action is not complete and should run again
                return (!motor1PIDF.isWithinDistance(10.0)) || (!motor2PIDF.isWithinDistance(10.0))
            }
        }
    }

    /**
     * Moves the lift to the specimenAutoScoreHighPos
     */
    fun toAutonSpecimenScoreHigh(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motor1PIDF.targetPosition = specimenAutoScoreHighPos * COUNTS_PER_INCH
                motor2PIDF.targetPosition = specimenAutoScoreHighPos * COUNTS_PER_INCH

                // If either motor is *not* within 10 ticks, the action is not complete and should run again
                return (!motor1PIDF.isWithinDistance(10.0)) || (!motor2PIDF.isWithinDistance(10.0))
            }
        }
    }

    /**
     * Moves the lift to the firstAutonSpecimenScoreHighPos
     */
    fun toFirstAutonSpecimenScoreHigh(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motor1PIDF.targetPosition = firstAutonSpecimenScoreHighPos * COUNTS_PER_INCH
                motor2PIDF.targetPosition = firstAutonSpecimenScoreHighPos * COUNTS_PER_INCH

                // If either motor is *not* within 10 ticks, the action is not complete and should run again
                return (!motor1PIDF.isWithinDistance(10.0)) || (!motor2PIDF.isWithinDistance(10.0))
            }
        }
    }

    /**
     * Moves the lift to the hangPos
     */
    fun toHangPos(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                motor1PIDF.targetPosition = hangPos * COUNTS_PER_INCH
                motor2PIDF.targetPosition = hangPos * COUNTS_PER_INCH

                // If either motor is *not* within 10 ticks, the action is not complete and should run again
                return (!motor1PIDF.isWithinDistance(10.0)) || (!motor2PIDF.isWithinDistance(10.0))
            }
        }
    }
    
    fun resetEncoder(): Action {
        return object: Action {
            override fun run(p: TelemetryPacket): Boolean {
                motor1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                motor2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                
                motor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                motor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                
                return false
            }
        }
    }

}