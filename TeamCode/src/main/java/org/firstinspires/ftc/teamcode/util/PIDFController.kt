package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime

class PIDFController(private val motor: DcMotorEx, private val coefficients: PIDFCoefficients) {
    
    private val timer = ElapsedTime()
    
    var targetPosition: Double = 0.0
    
    private var lastError = 0.0
    private var integralSum = 0.0
    
    fun update() {
        val error = targetPosition - motor.currentPosition
        
        integralSum += error * timer.seconds()
        val derivative = (error - lastError) / timer.seconds()
        
        lastError = error
        motor.power = (error * coefficients.p) + (integralSum * coefficients.i) + (derivative * coefficients.d) + coefficients.f
        
        timer.reset()
    }
    
    fun isWithinDistance(distance: Double): Boolean {
        return targetPosition - motor.currentPosition <= distance
    }
    
}