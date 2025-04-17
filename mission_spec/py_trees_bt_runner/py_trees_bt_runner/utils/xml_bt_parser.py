import xml.etree.ElementTree as ET

class XmlBTParser():

    def __init__(self, node_factory):
        self.node_factory = node_factory

    def parse_xml_bt(self, xml_tree_root):
        # Map BehaviorTree ID -> XML element
        bt_map = {}
        for bt in xml_tree_root.findall("BehaviorTree"):
            bt_id = bt.attrib["ID"]
            bt_map[bt_id] = bt

        main_tree_id = xml_tree_root.attrib.get("main_tree_to_execute")

        if main_tree_id not in bt_map:
            raise RuntimeError(f"Main tree '{main_tree_id}' not found")

        main_root_xml = bt_map[main_tree_id][0]
        root_node = self.build_bt_from_xml(main_root_xml, bt_map)
        return root_node

    def build_bt_from_xml(self, xml_element, bt_map):
        tag = xml_element.tag
        attribs = xml_element.attrib.copy()
        name = attribs.get("name", tag)

        if tag == "SubTree":
            subtree_id = attribs["ID"]
            subtree_root = bt_map[subtree_id][0]
            return self.build_bt_from_xml(subtree_root, bt_map)

        if tag not in self.node_factory:
            raise ValueError(f"Node '{tag}' non found in NODE_FACTORY")

        if tag == "Inverter":
            # Inverter must be initialized with a child node
            child_xml = xml_element[0]
            child_node = self.build_bt_from_xml(child_xml, bt_map)
            node = self.node_factory[tag](name=name, child=child_node)

        else:
            node = self.node_factory[tag](**attribs)
            for child_xml in xml_element:
                child_node = self.build_bt_from_xml(child_xml, bt_map)
                node.add_child(child_node)

        return node

    def load_tree_from_file(self, xml_path):
        xml_tree = ET.parse(xml_path)
        xml_tree_root = xml_tree.getroot()
        return self.parse_xml_bt(xml_tree_root)

    def load_tree_from_string(self, xml_string):
        xml_tree = ET.fromstring(xml_string)
        return self.parse_xml_bt(xml_tree)
