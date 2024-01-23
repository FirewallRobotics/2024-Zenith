 
#from networktables import NetworkTables, NetworkTablesInstance
import ntcore
import wpilib
import time

def main():
  #intitialize Vision Network Tables 
  #in competeion it is recommended to use static ip's 10.56.07.2 would be out team's static ip.
  #team5607_vision=team5607NetworkTables.visionTable(server='roborio-5607-frc.local', tableName="apriltag")
  #NT_DEFAULT_PORT = 1735
  TEAM = 5607
  ntinst = ntcore.NetworkTableInstance.getDefault()
  #ntinst.startClientTeam(5607, NT_DEFAULT_PORT)
  #ntinst.startClient4("AprilTagsCode")

  table = ntinst.getTable("AprilTagsTable")
 
  #xPub = table.getDoubleTopic("x").publish()
  #yPub = table.getDoubleTopic("y").publish()
  myStrPub =table.getStringTopic("tag1").publish()
  ntinst.startClient4("pi1 vision client")
  #ntinst.setServerTeam(TEAM) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
  ntinst.setServer("127.0.0.1") # use local host for testing with Outlineviewer
  #ntinst.startDSClient() # recommended if running on Driver Station computer; this gets the robot IP from the DS


  #xPub.set(5)
  #yPub.set(10)
  myStrPub.set('{"myprop": 5}' )
  
 
  version =ntcore.ConnectionInfo.protocol_version
  print(" Remote ip: %s" % ntcore.ConnectionInfo.remote_ip)

  while True:
     time.sleep(1)

if __name__ == '__main__':
    main()