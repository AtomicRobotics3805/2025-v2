package org.firstinspires.ftc.teamcode.configuration

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction

object MecanumDriveConstants {
    // IMU orientation
    var logoFacingDirection: RevHubOrientationOnRobot.LogoFacingDirection =
        RevHubOrientationOnRobot.LogoFacingDirection.UP
    var usbFacingDirection: RevHubOrientationOnRobot.UsbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT // TODO Double check this is correct

    // drive model parameters
    @JvmField
    var inPerTick: Double = 0.0011512
    var lateralInPerTick: Double = inPerTick
    var trackWidthTicks: Double = 0.0

    // feedforward parameters (in tick units)
    @JvmField
    var kS: Double = 0.0
    @JvmField
    var kV: Double = 0.0
    @JvmField
    var kA: Double = 0.0

    // path profile parameters (in inches)
    @JvmField
    var maxWheelVel: Double = 50.0
    @JvmField
    var minProfileAccel: Double = -30.0
    @JvmField
    var maxProfileAccel: Double = 50.0

    // turn profile parameters (in radians)
    var maxAngVel: Double = Math.PI // shared with path
    var maxAngAccel: Double = Math.PI

    // path controller gains
    var axialGain: Double = 0.0
    var lateralGain: Double = 0.0
    var headingGain: Double = 0.0 // shared with turn

    var axialVelGain: Double = 0.0
    var lateralVelGain: Double = 0.0
    var headingVelGain: Double = 0.0 // shared with turn
    
    @JvmField
    var leftFrontName = "LF"
    @JvmField
    var leftBackName = "LB"
    @JvmField
    var rightFrontName = "RF"
    @JvmField
    var rightBackName = "RB"
    
    @JvmField
    var leftFrontDirection = Direction.REVERSE
    @JvmField
    var leftBackDirection = Direction.REVERSE
    @JvmField
    var rightFrontDirection = Direction.FORWARD
    @JvmField
    var rightBackDirection = Direction.FORWARD
}