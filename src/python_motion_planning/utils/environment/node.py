class Node(object):
    """
    Class for searching nodes in 2D or 3D space.

    Parameters:
        current (tuple): current coordinate (x, y) or (x, y, z)
        parent (tuple): coordinate of parent node
        g (float): path cost
        h (float): heuristic cost
    """
    def __init__(self, current: tuple, parent: tuple = None, g: float = 0, h: float = 0) -> None:
        self.current = current
        self.parent = parent
        self.g = g
        self.h = h

    def __add__(self, node):
        assert isinstance(node, Node)
        # Handle 2D or 3D
        new_coords = tuple(a + b for a, b in zip(self.current, node.current))
        return Node(new_coords, self.parent, self.g + node.g, self.h)

    def __eq__(self, node) -> bool:
        if not isinstance(node, Node):
            return False
        return self.current == node.current

    def __ne__(self, node) -> bool:
        return not self.__eq__(node)

    def __lt__(self, node) -> bool:
        assert isinstance(node, Node)
        return self.g + self.h < node.g + node.h or \
               (self.g + self.h == node.g + node.h and self.h < node.h)

    def __hash__(self) -> int:
        return hash(self.current)

    def __str__(self) -> str:
        return f"Node({self.current}, {self.parent}, {self.g}, {self.h})"

    def __repr__(self) -> str:
        return self.__str__()

    # Current coordinates
    @property
    def x(self) -> float:
        return self.current[0]

    @property
    def y(self) -> float:
        return self.current[1]

    @property
    def z(self) -> float:
        if len(self.current) > 2:
            return self.current[2]
        return None

    # Parent coordinates
    @property
    def px(self) -> float:
        if self.parent:
            return self.parent[0]
        return None

    @property
    def py(self) -> float:
        if self.parent:
            return self.parent[1]
        return None

    @property
    def pz(self) -> float:
        if self.parent and len(self.parent) > 2:
            return self.parent[2]
        return None
