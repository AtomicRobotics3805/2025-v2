package org.firstinspires.ftc.teamcode.subsystems

import android.graphics.Color
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class IntakeSensorSubsystem(hardwareMap: HardwareMap) {
    
    val sensor: RevColorSensorV3 = hardwareMap.get(RevColorSensorV3::class.java, "intake_sensor")
    
    enum class SampleColors {
        RED,
        BLUE,
        YELLOW,
        NONE
    }
    
    var currentSample: SampleColors = SampleColors.NONE
    
    var hsv: FloatArray = FloatArray(3);
    
    fun blockUntilDetected(): Action {
        return object: Action {
            val watchdog = ElapsedTime()
            val watchdogThreshold = 2.0
            
            override fun run(p: TelemetryPacket): Boolean {
                return sensor.getDistance(DistanceUnit.CM) > 2 && watchdog.seconds() < watchdogThreshold
            }
        }
    }
    
    fun updateSampleColor() {
        Color.colorToHSV(sensor.normalizedColors.toColor(), hsv)
        currentSample = if (sensor.getDistance(DistanceUnit.CM) < 2) {
            if (hsv[0] <= 26) {
                SampleColors.RED
            } else if (hsv[0] <= 85) {
                SampleColors.YELLOW
            } else {
                SampleColors.BLUE
            }
        } else {
            SampleColors.NONE
        }
    }
}