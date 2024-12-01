package org.firstinspires.ftc.teamcode.autonomous.trajectories

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.TranslationalVelConstraint
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.util.rad

object TrajectoryFactory {
    // Starting positions
    val startPosY = 63.0
    val startPosLeft: Pose2d = Pose2d(36.0, startPosY, 270.rad)
    val startPosRight: Pose2d = Pose2d(-12.0, startPosY, 90.rad)
    
    //region SAMPLES
    // Score positions
    val basketHigh: Pose2d = Pose2d(53.5, 50.5, 225.rad)
    val basketLow: Pose2d = Pose2d(55.0, 55.0, 225.rad)

    // Intake positions
    val rightSample: Pose2d = Pose2d(49.0, 43.0, 270.rad)
    val centerSample: Pose2d = Pose2d(58.5, 50.5, 270.rad)
    val leftSample: Pose2d = Pose2d(60.25, 36.0, 320.rad)
    val leftSamplePickup: Pose2d = Pose2d(60.25, 34.0, 315.rad)

    // Trajectories
    // Left side
    lateinit var leftStartToHighBasket: Action

    lateinit var highBasketToAscentPark: Action
    lateinit var highBasketToZonePark: Action
    lateinit var highBasketToRightSample: Action

    lateinit var pickupRightSample: Action
    lateinit var rightSampleToHighBasket: Action
    lateinit var highBasketToCenterSample: Action


    lateinit var pickupCenterSample: Action
    lateinit var centerSampleToHighBasket: Action
    lateinit var highBasketToLeftSample: Action

    lateinit var leftSampleToHighBasket: Action
    lateinit var leftSampleToLeftSamplePickup: Action
    //endregion

    //region SPECIMENS
    val submersibleYPos = 22.0
    val submersibleYApproach = 35.0
    val highChamber1 = Pose2d(-10.0, submersibleYPos,  90.rad)
    val highChamber2 = Pose2d(-7.0, submersibleYPos, 90.rad)
    val highChamber3 = Pose2d(0.0, submersibleYPos, 90.rad)
    val highChamber4 = Pose2d(2.0, submersibleYPos, 90.rad)

    val observationZoneYPos = 50.0
    val transitPos1 = Pose2d(-36.0, 36.0, 270.rad)
    val transitPos2 = Pose2d(-36.0, 16.0, 270.rad)
    val leftColoredSample = Pose2d(-52.0, 12.0, 270.rad)
    val observationZoneLeft = Pose2d(-52.0, observationZoneYPos, 270.rad)

    val centerColoredSample = Pose2d(-61.0, 8.0, 270.rad)
    val observationZoneMiddle = Pose2d(-61.0, observationZoneYPos, 270.rad)

    val prepatorySpecimenPickupPos = Pose2d(-36.0, 50.0, 270.rad)
    val specimenPickupPosition = Pose2d(-36.0, 60.0, 270.rad)

    lateinit var rightStartToHighChamber1: Action
    lateinit var highChamber1ToBringLeftSampleToObservationZone: Action
    lateinit var observationZoneLeftToBringCenterSampleToObservationZone: Action

    lateinit var observationZoneMiddleToSpecimenPickupPosition: Action
    lateinit var specimenPickupPositionToHighChamber2: Action

    lateinit var highChamber2ToSpecimenPickupPosition: Action
    lateinit var specimenPickupPositionToHighChamber3: Action
    lateinit var highChamber3ToSpecimenPickupPosition: Action
    lateinit var specimenPickupPositionToHighChamber4: Action

    lateinit var highChamber3ToPark: Action
    lateinit var highChamber4ToPark: Action

    lateinit var highChamber1ToPark: Action

    //endregion

    // Park positions
    val ascentPark: Pose2d = Pose2d(22.0, 8.0, 0.rad)
    val observationZonePark: Pose2d = Pose2d(-50.0, 60.0, 0.rad)
    
    fun createTrajectories(drive: MecanumDrive) {
        
        //region LEFT SIDE
        leftStartToHighBasket = drive.actionBuilder(startPosLeft)
            .setTangent(270.rad)
            .splineToLinearHeading(basketHigh, 45.rad).build()
        highBasketToAscentPark = drive.actionBuilder(basketHigh)
            .splineToSplineHeading(ascentPark, 180.rad).build()

        highBasketToRightSample = drive.actionBuilder(basketHigh)
            .strafeToSplineHeading(rightSample.position, rightSample.heading).build()
        pickupRightSample = drive.actionBuilder(rightSample)
            .lineToY(rightSample.position.y + 8.0, TranslationalVelConstraint(40.0)).build()
        rightSampleToHighBasket = drive.actionBuilder(rightSample)
            .setTangent(90.rad)
            .splineToSplineHeading(basketHigh, 45.rad).build()

        highBasketToCenterSample = drive.actionBuilder(basketHigh)
            .strafeToLinearHeading(centerSample.position, centerSample.heading).build()
        pickupCenterSample = drive.actionBuilder(centerSample)
            .lineToY( centerSample.position.y + 15.0, TranslationalVelConstraint(20.0)).build()
        centerSampleToHighBasket = drive.actionBuilder(centerSample)
            .setTangent(90.rad)
            .strafeToSplineHeading(basketHigh.position, basketHigh.heading).build()

        highBasketToLeftSample = drive.actionBuilder(basketHigh)
            .splineToSplineHeading(leftSample, 315.rad).build()
        leftSampleToLeftSamplePickup = drive.actionBuilder(leftSample)
            .strafeToLinearHeading(leftSamplePickup.position, leftSamplePickup.heading, TranslationalVelConstraint(40.0)).build()
        leftSampleToHighBasket = drive.actionBuilder(leftSamplePickup)
            .setTangent(135.rad)
            .splineToSplineHeading(basketHigh, 90.rad).build()
        //endregion
        
        //region RIGHT SIDE
        rightStartToHighChamber1 = drive.actionBuilder(startPosRight)
            .setTangent(270.rad)
            .splineToLinearHeading(Pose2d(highChamber1.position.x, submersibleYApproach, highChamber1.heading.toDouble()), 270.rad)
            .lineToY(highChamber1.position.y)
            .build()
        highChamber1ToBringLeftSampleToObservationZone = drive.actionBuilder(highChamber1)
            .setTangent(90.rad)
            .lineToY(highChamber1.position.y + 4.0)
            .splineToSplineHeading(transitPos1, 270.rad)
            .splineToSplineHeading(transitPos2, 270.rad)
            .splineToConstantHeading(leftColoredSample.position, 90.rad, TranslationalVelConstraint(20.0))
            .strafeTo(observationZoneLeft.position)
            .build()
        observationZoneLeftToBringCenterSampleToObservationZone = drive.actionBuilder(observationZoneLeft)
            .setTangent(270.rad)
            .splineTo(leftColoredSample.position, 270.rad)
            .splineToConstantHeading(centerColoredSample.position, 90.rad, TranslationalVelConstraint(20.0))
            .strafeTo(observationZoneMiddle.position)
            .build()
        observationZoneMiddleToSpecimenPickupPosition = drive.actionBuilder(observationZoneMiddle)
            .setTangent(270.rad)
            .splineToConstantHeading(prepatorySpecimenPickupPos.position, 90.rad)
            .splineToConstantHeading(specimenPickupPosition.position, 90.rad)
            .build()
        specimenPickupPositionToHighChamber2 = drive.actionBuilder(specimenPickupPosition)
            .splineToSplineHeading(Pose2d(highChamber2.position.x, submersibleYApproach, highChamber2.heading.toDouble()), 270.rad)
            .splineToLinearHeading(highChamber2, 270.rad).build()


        highChamber2ToSpecimenPickupPosition = drive.actionBuilder(highChamber2)
            .setTangent(90.rad)
            .splineToSplineHeading(prepatorySpecimenPickupPos, 90.rad)
            .splineToConstantHeading(specimenPickupPosition.position, 90.rad).build()
        specimenPickupPositionToHighChamber3 = drive.actionBuilder(specimenPickupPosition)
            .splineToSplineHeading(Pose2d(highChamber3.position.x, submersibleYApproach, highChamber3.heading.toDouble()), 270.rad)
            .splineToLinearHeading(highChamber3, 270.rad).build()
        highChamber3ToSpecimenPickupPosition = drive.actionBuilder(highChamber3)
            .setTangent(90.rad)
            .splineToSplineHeading(prepatorySpecimenPickupPos, 90.rad)
            .splineToConstantHeading(specimenPickupPosition.position, 90.rad).build()
        specimenPickupPositionToHighChamber4 = drive.actionBuilder(specimenPickupPosition)
            .splineToSplineHeading(Pose2d(highChamber4.position.x, submersibleYApproach, highChamber4.heading.toDouble()), 270.rad)
            .splineToLinearHeading(highChamber4, 270.rad).build()

        highChamber1ToPark = drive.actionBuilder(highChamber1)
            .splineToSplineHeading(observationZonePark, 135.rad).build()

        highChamber3ToPark = drive.actionBuilder(highChamber3)
            .splineToSplineHeading(observationZonePark, 135.rad).build()

        highChamber4ToPark = drive.actionBuilder(highChamber4)
            .splineToSplineHeading(observationZonePark, 135.rad).build()
        //endregion
    }
}