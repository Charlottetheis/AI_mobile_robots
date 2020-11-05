import arcade
import math
import os


SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
SCREEN_TITLE = "Humans Example"
HUMAN_SPEED = 1



class MyGame(arcade.Window):
    """ Main application class """

    def __init__(self, width, height, title):
        super().__init__(width, height, title,antialiasing=False)

        # Set the working directory (where we expect to find files) to the same
        # directory this .py file is in. You can leave this out of your own
        # code, but it is needed to easily run the examples using "python -m"
        # as mentioned at the top of this program.
        file_path = os.path.dirname(os.path.abspath(__file__))
        os.chdir(file_path)

        arcade.set_background_color(arcade.color.WHITE)

        self.frame_count = 0

        self.human_list = None
        self.robot_list = None
        
        
    def setup(self):
        self.human_list = arcade.SpriteList()
        self.robot_list = arcade.SpriteList()

        # human
        human = arcade.Sprite("human.png", 0.5)
        human.center_x = 100
        human.center_y = 100
        self.human_list.append(human)

    def on_draw(self):
        """Render the screen. """

        arcade.start_render()

        self.human_list.draw()
        
    def on_update(self, delta_time):
        """All the logic to move, and the game logic goes here. """

        self.frame_count += 1
        
        for human in self.human_list:
            human.change_x = 20
            human.change_y = 20
            
            
        self.human_list.update()
            
            
def main():
    """ Main method """
    window = MyGame(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
    window.setup()
    arcade.run()