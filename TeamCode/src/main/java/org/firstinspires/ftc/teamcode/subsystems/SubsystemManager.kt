package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

object SubsystemManager {
    lateinit var arm: ArmSubsystem
    lateinit var claw: ClawSubsystem
    lateinit var intakeExtension: IntakeExtensionSubsystem
    lateinit var intakePivot: IntakePivotSubsystem
    lateinit var intakeSensor: IntakeSensorSubsystem
    lateinit var intake: IntakeSubsystem
    lateinit var lift: LiftSubsystem
    
    lateinit var drive: MecanumDrive
    
    
    fun initialize(hardwareMap: HardwareMap, drive: MecanumDrive) {
        arm = ArmSubsystem(hardwareMap)
        claw = ClawSubsystem(hardwareMap)
        intakeExtension = IntakeExtensionSubsystem(hardwareMap)
        intakePivot = IntakePivotSubsystem(hardwareMap)
        intakeSensor = IntakeSensorSubsystem(hardwareMap)
        intake = IntakeSubsystem(hardwareMap)
        lift = LiftSubsystem(hardwareMap)
        
        this.drive = drive
    }
}