package org.firstinspires.ftc.teamcode.opmode.auton;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class BasketConstants{
    public static Constant FORWARD = new Constant(
            BOT_WIDTH / 2 + 0.5,
            -WALL_POS + TILE_LENGTH * 2 - BOT_LENGTH / 2 - 7,
            UP
    );
    public static Constant Go1 = new Constant(
            TILE_LENGTH + BOT_WIDTH / 2 + 1.7,
            -WALL_POS + TILE_LENGTH * 2 - BOT_LENGTH / 2 - 7,
             UP
    );
    public static Constant Gop1 = new Constant(
            TILE_LENGTH + BOT_WIDTH / 2 + 1.2,
            -WALL_POS + TILE_LENGTH * 3 - BOT_LENGTH / 2 - 3.75,
             UP
     );
    public static Constant Gop2 = new Constant(
            TILE_LENGTH * 1.5 + BOT_WIDTH / 2 + 1.2,
            -WALL_POS + TILE_LENGTH * 3 - BOT_LENGTH / 2 - 3.75,
             UP
     );
    public static Constant Push1 = new Constant(
            TILE_LENGTH * 1.5 + BOT_WIDTH / 2 + 1.2,
             -WALL_POS + BOT_LENGTH / 2 + 6.25,
             UP
     );
    public static Constant Gop3 = new Constant(
            TILE_LENGTH * 1.5 + BOT_WIDTH / 2 + 1.2,
            -WALL_POS + TILE_LENGTH * 3 - BOT_LENGTH / 2 - 3.75,
             UP
     );
    public static Constant Gop4 = new Constant(
            TILE_LENGTH * 2 + BOT_WIDTH / 2 + 1.2,
            -WALL_POS + TILE_LENGTH * 3 - BOT_LENGTH / 2 - 3.75,
             UP
     );
    public static Constant Push2 = new Constant(
            TILE_LENGTH * 2 + BOT_WIDTH / 2 + 1.2,
            -WALL_POS + BOT_LENGTH / 2 + 6.25,
             UP
     );
    public static Constant Gop5 = new Constant(
            TILE_LENGTH * 2 + BOT_WIDTH / 2 + 1.2,
            -WALL_POS + TILE_LENGTH * 3 - BOT_LENGTH / 2 - 3.75,
             UP
     );
    public static Constant Gop6 = new Constant(
            TILE_LENGTH * 2 + BOT_WIDTH / 2 + 5.45,
            -WALL_POS + TILE_LENGTH * 3 - BOT_LENGTH / 2 - 3.75,
            DOWN
    );
    public static Constant Push3 = new Constant(
            TILE_LENGTH * 2 + BOT_WIDTH / 2 + 5.45,
            -WALL_POS + BOT_LENGTH / 2 + 6.25,
            DOWN
    );
    public static Constant Get1 = new Constant(
            TILE_LENGTH * 2 + BOT_WIDTH / 2 + 5.45,
            -WALL_POS + BOT_LENGTH / 2 + 2,
            DOWN
    );
    public static Constant Put1 = new Constant(
            BOT_WIDTH / 2 - 1,
            -WALL_POS + TILE_LENGTH * 2 - BOT_LENGTH / 2 - 7,
            UP
    );
    public static Constant Get2 = new Constant(
            TILE_LENGTH * 1.5 + BOT_WIDTH / 2 + 3.7,
            -WALL_POS + BOT_LENGTH / 2 + 6.25,
            DOWN
    );
    public static Constant Put2 = new Constant(
            BOT_WIDTH / 2 - 2,
            -WALL_POS + TILE_LENGTH * 2 - BOT_LENGTH / 2 - 7,
            UP
    );
    public static Constant Get3 = new Constant(
            TILE_LENGTH * 1.5 + BOT_WIDTH / 2 + 3.7,
            -WALL_POS + BOT_LENGTH / 2 + 6.25,
            DOWN
    );
    public static Constant Put3 = new Constant(
            BOT_WIDTH / 2 - 3,
            -WALL_POS + TILE_LENGTH * 2 - BOT_LENGTH / 2 - 7,
            UP
    );
    public static Constant Get4 = new Constant(
            TILE_LENGTH * 1.5 + BOT_WIDTH / 2 + 3.7,
            -WALL_POS + BOT_LENGTH / 2 + 6.25,
            DOWN
    );
    public static Constant Put4 = new Constant(
            BOT_WIDTH / 2 - 4,
            -WALL_POS + TILE_LENGTH * 2 - BOT_LENGTH / 2 - 7,
            UP
    );
    public static Constant Park = new Constant(
            TILE_LENGTH * 2 + BOT_WIDTH / 2 + 2,
            -WALL_POS + BOT_LENGTH / 2 + 4,
            UP
    );
}