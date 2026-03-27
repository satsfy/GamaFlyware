class Attitude:
    """
    Attitude (posição e orientação) do drone.
    """

    def __init__(self, t: int, x: float, y: float, z: float, heading: float):
        self.t = t
        self.x = x
        self.y = y
        self.z = z
        self.heading = heading
