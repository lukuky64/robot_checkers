"""
------------------------------------------------------------------------------------------------------------------
Modification by: 	Lucas Moore
Date: 				November 2023
Purpose: 			To modify the checkers game to be used as a ROS node for the purpose of interacting with matlab

Modifications:
- Added user input to allow game to either be played via camera or via user input
-------------------------------------------------------------------------------------------------------------------
"""

import checkers_image_interaction
import checkers
import gamebot
from time import sleep
##COLORS##
#             R    G    B
WHITE = (255, 255, 255)
BLUE = (0,   0, 255)
RED = (255,   0,   0)
BLACK = (0,   0,   0)
GOLD = (255, 215,   0)
HIGH = (160, 190, 255)

##DIRECTIONS##
NORTHWEST = "northwest"
NORTHEAST = "northeast"
SOUTHWEST = "southwest"
SOUTHEAST = "southeast"


def main():
    while True:

        while True:
            result = int(input("Do you want to detect checkers with camera? (1-yes, 0-no): "))
            if result == 1 or result == 0:
                break        

        game = checkers_image_interaction.Game(loop_mode=False, game_mode= not result) # checkers_image_interaction
        game.setup()
        game.update()
        bot = gamebot.Bot(game, RED, mid_eval='piece_and_board',
                          end_eval='sum_of_dist', method='alpha_beta', depth=3)
        random_bot_blue = gamebot.Bot(
            game, BLUE, mid_eval='piece_and_board_pov', method='alpha_beta', depth=3, end_eval='sum_of_dist')
        while True:  # main game loop
            if game.turn == BLUE:
                 # TO start player's turn uncomment the below line and comment a couple  of line below than that
                game.player_turn()
                #count_nodes = random_bot_blue.step(game.board, True)
                #print('Total nodes explored in this step are', count_nodes)
                game.update()
            else:
                # TO start player's turn uncomment the below line and comment a couple  of line below than that
                sleep(0.5)
                game.player_turn()
                count_nodes = bot.step(game.board, True)
                #print('Total nodes explored in this step are', count_nodes)
                game.update()
            if game.endit:
                break


if __name__ == "__main__":
    main()
    pass
