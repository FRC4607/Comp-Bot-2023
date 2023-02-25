# Portions copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from distutils.command.config import config
import json
import sys
import ntcore
from cscore import CameraServer, CvSource

CameraServer.enableLogging()
team = None
server = False
configFile = "/boot/frc.json"

ntinst: ntcore.NetworkTableInstance = None

csOutput: CvSource = None

started: bool = False

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

def configureNetworkTables():
    global ntinst
    global csOutput
    readConfig()

    ntinst = ntcore.NetworkTableInstance.getDefault()
    ntinst.setServerTeam(team)
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4(str(team))
    entry = ntinst.getTable("PiTable").getEntry("IsFSOpen")
    entry.setDefaultBoolean(False)
    entry = ntinst.getTable("PiTable").getEntry("RecordingEnabled")
    entry.setDefaultBoolean(False)
    entry = ntinst.getTable("PiTable").getEntry("Start")
    entry.setDefaultBoolean(False)

    global csOutput
    CameraServer.startAutomaticCapture()
    csOutput = CameraServer.putVideo("Camera Feed", 416, 416)

def getRobotEnabled() -> bool:
    return ntinst.getTable("PiTable").getBoolean("RecordingEnabled", False)

def setFSOpen(open: bool):
    entry = ntinst.getTable("PiTable").getEntry("IsFSOpen")
    entry.setBoolean(open)
    
def pushCameraServer(frame):
    global csOutput
    csOutput.putFrame(frame)

def getStarted() -> bool:
    global started
    if ntinst.getTable("PiTable").getEntry("Start").getBoolean(False):
        ntinst.getTable("PiTable").getEntry("Start").setBoolean(False)
        started = True
    return started
