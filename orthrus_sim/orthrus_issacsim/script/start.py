from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core import World

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = World.instance()
        world.scene.add_default_ground_plane()
        return