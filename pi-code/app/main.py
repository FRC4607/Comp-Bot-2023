#!/usr/bin/env python3

import depthai as dai
import depthaiprocessing as daiprocessing
import networktablesprocessing as ntprocessing
import fsprocessing
import numpy as np

if __name__ == "__main__":
    print("Configuring NT...")
    ntprocessing.configureNetworkTables()
    print("Waiting for start command...")
    while not ntprocessing.getStarted():
        pass
    print("Opening DepthAI device...")
    pipeline = daiprocessing.configurePipeline()
    with dai.Device(pipeline) as device:
        hd, ld = daiprocessing.configureQueues(device)
        hd: np.ndarray = hd
        while True:
            h256data, bgrData = daiprocessing.readQueues(hd, ld)
            ntprocessing.pushCameraServer(bgrData)
            if (ntprocessing.getRobotEnabled()):
                fsprocessing.openVideoFile()
                fsprocessing.saveFrameToFile(h256data)
            else:
                fsprocessing.closeVideoFile()
            ntprocessing.setFSOpen(fsprocessing.getFsOpen())
