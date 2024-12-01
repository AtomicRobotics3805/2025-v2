package org.firstinspires.ftc.teamcode.configuration

import com.qualcomm.robotcore.hardware.DcMotorSimple

object OdometryConstants {
    
    var parYTicks: Double = 0.0 // y position of the parallel encoder (in tick units)
    var perpXTicks: Double = 0.0 // x position of the perpendicular encoder (in tick units)
    
    var parName: String = "RB"
    var perpName: String = "LB"
    
    var parDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.REVERSE
    var perpDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.REVERSE
    
}