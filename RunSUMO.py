import main as mf
import traci

files = mf.configs( 'Leopoldstrasse' )
traci.start(["sumo", "-c", "./scenario/" + files.name +  "/sumo/" + "osm.sumocfg", "--vehroute-output", files.vehroute_file_path])

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
traci.close()