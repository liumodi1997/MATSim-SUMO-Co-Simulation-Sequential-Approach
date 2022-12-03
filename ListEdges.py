import xml.etree.ElementTree as ET
import main as mf

SUMO_EDGE = {}
file = ".\\scenario\\OlympiaZentrum\\sumo\\osm.net.xml"
SUMO_EDGE = mf.load_edge_info(file)
i = 1
for edge in SUMO_EDGE:
    print(f"{i}: {edge}, from '{SUMO_EDGE[edge][0]}' to '{SUMO_EDGE[edge][1]}', len={SUMO_EDGE[edge][2]}")
    i += 1
