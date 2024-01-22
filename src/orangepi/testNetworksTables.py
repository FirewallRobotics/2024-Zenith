 
from networktables import NetworkTables, NetworkTablesInstance

def main():
  #intitialize Vision Network Tables 
  #in competeion it is recommended to use static ip's 10.56.07.2 would be out team's static ip.
  #team5607_vision=team5607NetworkTables.visionTable(server='roborio-5607-frc.local', tableName="apriltag")
  NT_DEFAULT_PORT = 1735
  ntinst = NetworkTablesInstance.getDefault()
  ntinst.startClientTeam(5607, NT_DEFAULT_PORT)


if __name__ == '__main__':
    main()