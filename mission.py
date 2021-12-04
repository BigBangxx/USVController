from xml.etree import ElementTree
from SelfBuiltModul.func import pretty_xml


class Mission:
    """任务相关类"""

    def __init__(self, usv_id):
        self.file_name = f"AppData/{usv_id}.xml"

    def read(self):
        """去读xml文件，返回路点列表"""
        waypoints = []
        with open(self.file_name) as wp:
            tree = ElementTree.parse(wp)
            root = tree.getroot()
            for i in root:
                if i.tag == 'waypoint':
                    waypoints.append(
                        (float(i.attrib['latitude']), float(i.attrib['longitude']), float(i.attrib['tolerance'])))
        return waypoints

    def write(self, waypoints):
        """将路点写入XML文件"""
        root = ElementTree.Element('mission')
        for way_point in waypoints:
            sub = ElementTree.SubElement(root, 'waypoint',
                                         {'latitude': str(way_point[0]), 'longitude': str(way_point[1]),
                                          'tolerance': str(way_point[2])})
        pretty_xml(root, '\t', '\n')
        tree = ElementTree.ElementTree(root)
        tree.write(self.file_name, 'UTF-8', True)
