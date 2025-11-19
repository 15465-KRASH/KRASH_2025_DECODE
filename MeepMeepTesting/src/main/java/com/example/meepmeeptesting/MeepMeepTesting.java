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

//        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d initialPose = new Pose2d(64, 15, Math.toRadians(180));

        Pose2d firstShot = new Pose2d(new Vector2d(58, 15), Math.toRadians(shotAngle));

        Pose2d startPickup = new Pose2d(new Vector2d(36, 30), Math.toRadians(90));
        Pose2d finishPickup = new Pose2d(new Vector2d(36, 42), Math.toRadians(90));

        Pose2d start2ndPickup = new Pose2d(new Vector2d(11, 19), Math.toRadians(90));
        Pose2d finish2ndPickup = new Pose2d(new Vector2d(11, 39), Math.toRadians(90));

        Pose2d finalPos = new Pose2d(new Vector2d(0, 38), Math.toRadians(90));
        Pose2d parkHighPos = new Pose2d(new Vector2d(50, 26), Math.toRadians(160.5));

        TranslationalVelConstraint pickupVelConstraint = new TranslationalVelConstraint(4);
        AccelConstraint pickupAccelConstraint = new ProfileAccelConstraint(-50, 10);

        TrajectoryActionBuilder firstShotTraj = myBot.getDrive().actionBuilder(initialPose)
                .lineToXLinearHeading(firstShot.position.x, firstShot.heading);

//        TrajectoryActionBuilder firstShotTraj = myBot.getDrive().actionBuilder(initialPose)
//                .splineTo(new Vector2d(48, 48), Math.toRadians(90));

        Action firstShotAction = firstShotTraj.build();

        TrajectoryActionBuilder pickupFirst = firstShotTraj.endTrajectory().fresh()
                .setTangent(Math.toRadians(shotAngle))
                .splineTo(startPickup.position, Math.toRadians(90))
                .splineTo(new Vector2d(startPickup.position.x, startPickup.position.y + 2), Math.toRadians(90), pickupVelConstraint)
                .splineTo(finishPickup.position, Math.toRadians(90), new TranslationalVelConstraint(50), pickupAccelConstraint);

        Action pickupFirstAction = pickupFirst.build();



        myBot.runAction(new SequentialAction(
                firstShotAction,
                new SleepAction(1),
                pickupFirstAction
        ));

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