import ctypes

from pyueye import ueye as u

hCam = u.HIDS(0)             #0: first available camera;  1-254: The camera with the specified camera ID
sInfo = u.SENSORINFO()
cInfo = u.CAMINFO()
pcImageMemory = u.c_mem_p()
MemID = u.int()
rectAOI = u.IS_RECT()
pitch = u.INT()
nBitsPerPixel = u.INT(24)    #24: bits per pixel for color mode; take 8 bits per pixel for monochrome
channels = 3                    #3: channels for color mode(RGB); take 1 channel for monochrome
m_nColorMode = u.INT()		# Y8/RGB16/RGB24/REG32
bytes_per_pixel = int(nBitsPerPixel / 8)
#---------------------------------------------------------------------------------------------------------------------------------------
print("START")
print()


# Starts the driver and establishes the connection to the camera
nRet = u.is_InitCamera(hCam, None)
if nRet != u.IS_SUCCESS:
    print("is_InitCamera ERROR")

# Define the UINT type
UINT = ctypes.c_uint
# Define the structure for nRange
class NRangeStructure(ctypes.Structure):
    _fields_ = [("nMin", UINT), ("nMax", UINT), ("nInc", UINT)]

# Create an instance of the structure and initialize it to zeros
nRange = NRangeStructure()

# Call the is_PixelClock function
nRet = u.is_PixelClock(hCam, u.IS_PIXELCLOCK_CMD_GET_RANGE,ctypes.pointer(nRange), u.sizeof(nRange))

# Check the return value
if nRet == 0:  # Assuming IS_SUCCESS is defined as 0
    nMin = nRange.nMin
    nMax = nRange.nMax
    nInc = nRange.nInc
    print(f"nMin: {nMin}, nMax: {nMax}, nInc: {nInc}")
else:
    print(f"Error: {nRet}")
