import xml.etree.ElementTree as ET
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point


def get_pixel_coordinates(path, point):
    three = ET.parse(path)
    root = three.getroot()
    px = float(root[0][3][1].text)
    py = float(root[0][3][2].text)
    u0 = float(root[0][3][3].text)
    v0 = float(root[0][3][4].text)

    x = point.x
    y = point.y
    z = point.z

    u = u0 + px * x/z
    v = v0 + py * y/z
    u_round = round(u)
    v_round = round(v)
    return u_round, v_round


def apply_transformations(center, transformations):
    do_point = PointStamped()
    do_point.point.x = center[0]
    do_point.point.y = center[1]
    do_point.point.z = center[2]
    transformed_center = {}

    for t in transformations:
        center_t = do_transform_point(do_point, t)
        band = center_t.header.frame_id[:-1]
        number = center_t.header.frame_id[-1]
        path = "/home/runtime/Documents/Diplomski_projekt/intrinsics/" + band + "_" + number + ".xml"
        transformed_center[center_t.header.frame_id[3:] + ".jpg"] = get_pixel_coordinates(path, center_t.point)
    
    return transformed_center
