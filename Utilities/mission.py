from xml.etree import ElementTree


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


def pretty_xml(element, indent, newline, level=0):  # elemnt为传进来的Elment类，参数indent用于缩进，newline用于换行
    """为ElementTree处理的XML添加缩进和换行"""
    if element:  # 判断element是否有子元素
        if (element.text is None) or element.text.isspace():  # 如果element的text没有内容
            element.text = newline + indent * (level + 1)
        else:
            element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * (level + 1)
            # else:  # 此处两行如果把注释去掉，Element的text也会另起一行
            # element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * level
    temp = list(element)  # 将element转成list
    for subelement in temp:
        if temp.index(subelement) < (len(temp) - 1):  # 如果不是list的最后一个元素，说明下一个行是同级别元素的起始，缩进应一致
            subelement.tail = newline + indent * (level + 1)
        else:  # 如果是list的最后一个元素， 说明下一行是母元素的结束，缩进应该少一个
            subelement.tail = newline + indent * level
        pretty_xml(subelement, indent, newline, level=level + 1)  # 对子元素进行递归操作
