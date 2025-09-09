from python_motion_planning.utils.environment.point3d import Point3D

def test_point3d_operations():
    # Create points
    p1 = Point3D(1, 2, 3)
    p2 = Point3D(4, 5, 6)
    p3 = Point3D(1, 2, 3)  # same as p1

    # Test arithmetic
    p_sum = p1 + p2
    p_diff = p2 - p1
    print("p1 + p2 =", p_sum)
    print("p2 - p1 =", p_diff)

    # Test equality
    print("p1 == p3:", p1 == p3)  # True
    print("p1 != p2:", p1 != p2)  # True

    # Test distance
    print("Distance p1 -> p2:", p1.dist(p2))

    # Test tuple conversions
    tuple_p1 = p1.to_tuple
    p_from_tuple = Point3D.from_tuple(tuple_p1)
    print("Tuple of p1:", tuple_p1)
    print("Point from tuple:", p_from_tuple)

    # Test angles
    print("Angle XY p1 -> p2:", p1.angle_xy(p2))
    print("Angle XZ p1 -> p2:", p1.angle_xz(p2))
    print("Angle YZ p1 -> p2:", p1.angle_yz(p2))

if __name__ == "__main__":
    test_point3d_operations()
