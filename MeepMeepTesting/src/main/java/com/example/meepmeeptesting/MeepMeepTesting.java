package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 2)
                .setDimensions(18, 17.5)
                .build();


        double shotAngle = 160.5;

        Pose2d initialPose = new Pose2d(-39, 54, Math.toRadians(180));
//        HeadingStorage.zeroOffset = initialPose.heading.log() - Math.toRadians(90);


        Pose2d tagCheck = new Pose2d(new Vector2d(-36, 36), Math.toRadians(-135));
        Pose2d firstShot = new Pose2d(new Vector2d(-16, 14), Math.toRadians(142));

        Pose2d startPickup = new Pose2d(new Vector2d(-12, 32), Math.toRadians(90));
        Pose2d finishPickup = new Pose2d(new Vector2d(-12, 48), Math.toRadians(90));

        Pose2d start2ndPickup = new Pose2d(new Vector2d(12, 32), Math.toRadians(90));
        Pose2d finish2ndPickup = new Pose2d(new Vector2d(12, 48), Math.toRadians(90));

        Pose2d finalPos = new Pose2d(new Vector2d(0, 38), Math.toRadians(90));

        TranslationalVelConstraint pickupVelConstraint = new TranslationalVelConstraint(4);

//        Robot m_robot = new Robot(hardwareMap, telemetry, initialPose);
//
//        telemetry.setMsTransmissionInterval(11);
//        m_robot.limelight.pipelineSwitch(0);
//        m_robot.limelight.start();
//
//        IntakeArtifactInOrder intakeAction = new IntakeArtifactInOrder(m_robot.intake, m_robot.spindexer, true);
//        ShootAllVariant shootAction = new ShootAllVariant(m_robot.shooter, m_robot.spindexer);
//        ScanIntake scanAction = new ScanIntake(m_robot.spindexer);

        TrajectoryActionBuilder firstShotTraj = myBot.getDrive().actionBuilder(initialPose)
                .setTangent(Math.toRadians(-45))
                .splineToSplineHeading(tagCheck, Math.toRadians(-45))
                .splineToSplineHeading(firstShot, Math.toRadians(-45));

        Action firstShotAction = firstShotTraj.build();

        TrajectoryActionBuilder pickupFirst = firstShotTraj.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(startPickup, Math.toRadians(90))
                .splineToSplineHeading(finishPickup, Math.toRadians(90), pickupVelConstraint);

        Action pickupFirstAction = pickupFirst.build();

        TrajectoryActionBuilder shootSecondTraj = pickupFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstShot, Math.toRadians(-90));

        Action shootSecondAction = shootSecondTraj.build();



        TrajectoryActionBuilder pickupSecond = shootSecondTraj.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(start2ndPickup, Math.toRadians(90))
                .splineToSplineHeading(finish2ndPickup, Math.toRadians(90), pickupVelConstraint);

        Action pickupSecondAction = pickupSecond.build();

        TrajectoryActionBuilder finalPosTraj = pickupSecond.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(finalPos, Math.toRadians(179.9));

        Action finalPosAction = finalPosTraj.build();



        myBot.runAction(new SequentialAction(
                firstShotAction,
                pickupFirstAction,
                shootSecondAction,
                pickupSecondAction,
                finalPosAction ));

//        myBot.runAction(new SleepAction(10));
//        myBot.runAction(pickupFirstAction);



//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
//                .lineToX(30)
//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}