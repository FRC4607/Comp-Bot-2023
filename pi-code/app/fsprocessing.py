
from io import BufferedWriter
import subprocess
import os
import numpy as np

videoFile: BufferedWriter = None
isFSOpen = False

def openVideoFile():
    global videoFile, isFSOpen
    if videoFile is None:
        print("Mounting exFAT partition.")
        subprocess.run(["sudo", "mount", "-o", "rw,user,uid=1000,dmask=007,fmask=117", "/dev/mmcblk0p3", "/mnt/videos"])
        isFSOpen = True
        nextNumber = getNextNumber()
        print(f"Creating {'/mnt/videos/' + str(nextNumber) + '.h265'}")
        videoFile = open("/mnt/videos/" + str(nextNumber) + ".h265", "wb")

def closeVideoFile():
    global videoFile, isFSOpen
    if videoFile is not None:
        print("Closing video file.")
        videoFile.close()
        while not videoFile.closed:
            pass
        videoFile = None
        print("Unmounting exFAT partition.")
        subprocess.run(["sudo", "umount", "/mnt/videos"])
        isFSOpen = False

def saveFrameToFile(frame):
    frame: np.ndarray = frame
    frame.tofile(videoFile)

def getFsOpen() -> bool:
    return isFSOpen

def getNextNumber() -> int:
    files = os.listdir("/mnt/videos")
    del files[files.index("System Volume Information")]
    nums = [int(s[:-5]) for s in files]
    nums.sort()
    return len(nums)
