package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-70, -12, Math.toRadians(180)))
                .lineToX(-34)
                //кладем пластинки
                .lineToX(-38)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)
                .lineToX(-49)
                //камера если справа
                //...
                .strafeTo(new Vector2d(-49, 3))
                .waitSeconds(1)
                //если камера по центру
                //...
                .strafeTo(new Vector2d(-49, 10))
                //камера если камера слева
                //...
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}