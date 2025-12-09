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

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));

        Pose2d firstShot = new Pose2d(new Vector2d(0, 24), Math.toRadians(180));

        Pose2d oneByOne = new Pose2d(new Vector2d(-24, 24), Math.toRadians(180));


//        TrajectoryActionBuilder firstShotTraj = myBot.getDrive().actionBuilder(initialPose)
//                .setTangent(Math.toRadians(180))
//                .splineToConstantHeading(oneByOne.position,Math.toRadians(90));

//        TrajectoryActionBuilder firstShotTraj = myBot.getDrive().actionBuilder(initialPose)
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(oneByOne.position,Math.toRadians(180));

        TrajectoryActionBuilder firstShotTraj = myBot.getDrive().actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .splineTo(firstShot.position, Math.toRadians(90));

        Action firstShotAction = firstShotTraj.build();



        myBot.runAction(firstShotAction);

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