package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d

val Double.inchesToMm get() = this * 25.4
val Double.mmToInches get() = this / 25.4
val Double.toRadians get() = (Math.toRadians(this))
val Double.rad get() = (this.toRadians)

val Int.inchesToMm get() = this * 25.4
val Int.mmToInches get() = this / 25.4
val Int.toRadians get() = (Math.toRadians(this.toDouble()))
val Int.rad get() = (this.toRadians)