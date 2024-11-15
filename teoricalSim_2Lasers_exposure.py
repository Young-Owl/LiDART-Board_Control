import matplotlib.pyplot as plt
import numpy as np

# Laser parameters
laser_diameter = 4.5e-3
laser_divergence = 0.2e-3
wavelength = 850e-9
laser_separation = 11.7e-3				# Separation between the two lasers in mm

# Camera parameters
focal_length = 16e-3 					# In meters

# Field of view in radians per pixel
AfovPxRad = 0.021* np.pi / 180 			# 0.021 degrees (See Obsidian)
#print(AfovPxRad)

# Parameters for Lidar equation
ar = np.pi * (11.4e-3 / 2) ** 2			# Aperture area in meters
reflectivity = 80 / 100 				# Includes object reflectivity and objective lens transmissivity

# Pulse Energy calculation
laser_power = 30e-3								# Laser power in watts (30 mW)
exposure_time = 50e-6							# Exposure time in seconds
pulse_energy = laser_power * exposure_time		# Pulse energy in joules	

# Constants
h = 6.626e-34  							# J*s
c = 299792458  							# m/s

# Parameters for noise model
quantum_efficiency = 6.25/100			# 6.25% Quantum efficiency - (Valor medido)
sensitivity = 1.923						# Sensitivity in electrons per ADU
dark_noise = 6.83						# Dark noise in electrons
bit_depth = np.log2(57218)				# 12-bit camera
baseline = 112							# Baseline ADU count
maximum_adu = int(2**bit_depth - 1)		# Maximum ADU count

# Random seed
seed = 42
rs = np.random.RandomState(seed)

def get_noise(point, side, d, aux_x, aux_y):

	# Estimate laser dot diameter
	spot_diameter = laser_diameter + 2 * d * np.tan(laser_divergence / 2)

	# Estimate pixel width
	pixel_width = 2 * d * np.tan(AfovPxRad / 2)
	#print(pixel_width)
 
	# Intrinsic Camera Matrices
	K1 = np.array([	[focal_length, 	0, 				0], 
                	[0, 			focal_length, 	0],
					[0, 			0, 				1]])

	# Extrinsic Camera Matrices
	# Translation Matrices
	T1 = np.array([[0, 0, 0]]).T
	
	# Rotation Matrices
	R1 = np.identity(3)

	# Calibration Matrices
	P1 = np.matmul(K1, np.concatenate((R1, -T1), axis=1))
	#print(P1)

	# Camera projetion coordinates
	x1 = np.matmul(P1, point)

	# Normalization
	if x1[2] != 0:
		x1 /= x1[2]


	# Total number of received photons
	free_space_path_loss = 1 / (4 * np.pi * d**2)
	energy_captured_by_camera = pulse_energy * free_space_path_loss * ar * reflectivity
	photon_energy = h * c / wavelength
	num_photons = energy_captured_by_camera / photon_energy

	# Generate the laser circumference
	# Following the "side" parameter, the laser circumference will be generated on the left or right side
	# Meaning that the circle will have either a positive or negative x offset
 
	# Generate the x values
	if side == 0:
		# Left Side (Circle CenterX = -laser_separation/2)
		x = np.linspace(-(spot_diameter / 2), +(spot_diameter / 2), 256)	# 256 points
		y = np.sqrt((spot_diameter**2 / 4) - x**2)
		x = x - laser_separation/2
	else:
		# Right Side (Circle CenterX = laser_separation/2)
		x = np.linspace(-(spot_diameter / 2), +(spot_diameter / 2), 256)	# 256 points
		y = np.sqrt((spot_diameter**2 / 4) - x**2)
		x = x + laser_separation/2
 
	x = x + 0
	y = y + 0
	x = np.hstack((x, x[::-1]))
	y = np.hstack((y, -y))

	# Identify which pixels are illuminated by the laser dots
	x1minPx = np.floor(np.min(x / pixel_width) + aux_x)
	x1maxPx = np.ceil(np.max(x / pixel_width) + aux_x)
	y1minPx = np.floor(np.min(y / pixel_width) + aux_y)
	y1maxPx = np.ceil(np.max(y / pixel_width) + aux_y)

 	# Identify the meshgrid of pixels
	xPx = np.arange(x1minPx, x1maxPx+1)
	yPx = np.arange(y1minPx, y1maxPx+1)
	#xPx = xPx.astype(dtype='uint16')
	#yPx = yPx.astype(dtype='uint16')
	
	XPx, YPx = np.meshgrid(xPx, yPx)

	# Fill the dot with dots
	Npoints = int(1e5)
	circleRadius = spot_diameter / pixel_width / 2
	circleDots = (np.random.rand(Npoints) * circleRadius * np.exp(1j * 2 * np.pi * np.random.rand(Npoints)))
	circleDotsx = np.real(circleDots) + np.mean(x) / pixel_width
	circleDotsy = np.imag(circleDots)

	# Generate laser dot
	circleDotsx += aux_x
	circleDotsy += aux_y

	# Shift x and y
	x += aux_x * pixel_width
	y += aux_y * pixel_width

	# Calculate number of photons in each pixel
	# Sweep each pixel and estimate the number of photons that fall on it
	dotsInPixel = np.zeros((len(yPx), len(xPx), Npoints), dtype=bool)
	#print(dotsInPixel)

	for a in range(len(yPx)):
		for b in range(len(xPx)):
			pixelLimits = [
				XPx[a, b] - 0.5,
				XPx[a, b] + 0.5,
				YPx[a, b] - 0.5,
				YPx[a, b] + 0.5,
			]

			# Identify the dots that fall on the pixel
			dotsInPixel[a, b, :] = np.logical_and(
				np.logical_and(
					circleDotsx >= pixelLimits[0], circleDotsx <= pixelLimits[1]
				),
				np.logical_and(
					circleDotsy >= pixelLimits[2], circleDotsy <= pixelLimits[3]
				),
			)

	# Count (and normalize) the number of dots in each pixel
	dotsInPixel = np.sum(dotsInPixel, axis=2) / Npoints
	#dotsInPixel /= Npoints
	
	# Pixelization
	# Total number of photons in each pixel
	num_photons_per_pixel = num_photons * dotsInPixel
 
	return num_photons_per_pixel, xPx, yPx

# Noise model function
def add_camera_noise(num_photons, noiseOn=True, baselineOn=True):
	if not baselineOn:
		baselineFunc = 0
	else:
		baselineFunc = baseline
 
	if noiseOn:

		# Add photon shot noise
		photon_shot_noise = rs.poisson(num_photons, size = num_photons.shape)
		
		# Get the number of photoelectrons
		num_photoelectrons = np.round(quantum_efficiency * photon_shot_noise)
		
		# Add dark noise
		electrons_out = np.round(rs.normal(scale = dark_noise, size = num_photoelectrons.shape) + num_photoelectrons)
		
		# Convert electrons to Analog-to-Digital Units (ADU) and add baseline
		adu = (electrons_out * sensitivity).astype(int) # Ensure the final ADU count is discrete
		adu += baselineFunc
		adu[adu > maximum_adu] = maximum_adu  # Models pixel saturation
	
	else:
		# Convert electrons to Analog-to-Digital Units (ADU)
		adu = (num_photons * quantum_efficiency * sensitivity).astype(int)
  
		adu[adu > maximum_adu] = maximum_adu # Models pixel saturation
	
	return adu

count = 0

if __name__ == "__main__":

	squareSize = 124
	squareDecrease = 15
 
 	# Distance sweep
	for d in [1, 2, 4, 8, 16, 24, 46]:
		print('Distance = ' + str(d) + ' m')
		point = [0, 0, d, 1]
		sum = 0

		# Get noise for left and right spots
		num_photons_per_pixel_left, x_left, y_left = get_noise(point, 0, d, 0, 0)
		num_photons_per_pixel_right, x_right, y_right = get_noise(point, 1, d, 0, 0)
  

		# Normalize x and y coordinates (to avoid negative indexes)
		x_leftNorm = x_left + abs(min(x_left))
		x_rightNorm = x_right + abs(min(x_left))
		y_leftNorm = y_left + abs(min(y_left))
		y_rightNorm = y_right + abs(min(y_right))

		# Create the final image
		Xfinal, Yfinal = np.meshgrid(np.arange(min(x_leftNorm), max(x_rightNorm)+1), np.arange(y_leftNorm[0], y_leftNorm[-1]+1))
		finalMatrixNumberOfPhotons = np.zeros_like(Xfinal)

		# Convert to numpy array
		x_leftNorm = np.array(x_leftNorm)
		x_rightNorm = np.array(x_rightNorm)
  
		# Left Spot
		for x in range(int(x_leftNorm[0]), int(x_leftNorm[-1]+1)):
			for y in range(int(y_leftNorm[0]), int(y_leftNorm[-1]+1)):
				finalMatrixNumberOfPhotons[y, x] += num_photons_per_pixel_left[y, x]
    
		# Right Spot
		for x in range(int(x_rightNorm[0]), int(x_rightNorm[-1]+1)):
			for y in range(int(y_rightNorm[0]), int(y_rightNorm[-1]+1)):
				finalMatrixNumberOfPhotons[y, x] += num_photons_per_pixel_right[y, x - int(x_rightNorm[0])]

		# Add noise to the final image
		imageOut = add_camera_noise(finalMatrixNumberOfPhotons, noiseOn=True, baselineOn=True)

		# Add noise to the left and right spots (Purely for ADU comparison)
		leftSpot = add_camera_noise(num_photons_per_pixel_left, noiseOn=True, baselineOn=True)
		rightSpot = add_camera_noise(num_photons_per_pixel_right, noiseOn=True, baselineOn=True)
		print('Max ADU: ' + str(max(imageOut.flatten())))
		print('Left Max ADU: ' + str(max(leftSpot.flatten())))
		print('Right Max ADU: ' + str(max(rightSpot.flatten())))
  
		# Count all the pixels from both spots
		for i in range(imageOut.shape[0]):
			for j in range(imageOut.shape[1]):
				if int(imageOut[i][j]) >= (baseline * 2):
					sum = sum + int(imageOut[i][j])

		print('Sum = ' + str(sum))
  
		fig, ax = plt.subplots()
  
		# Disable ticks and grid for better visibility
		noGrid = False

		if not noGrid:
			ax.set_xticks(np.arange(min(x_leftNorm) - 2.5, max(x_rightNorm) + 3.5, 3))
			ax.set_yticks(np.arange(min(y_leftNorm) - 2.5, max(y_leftNorm) + 3.5, 3))  
	
			ax.set_xticklabels(np.arange(min(x_leftNorm) - 3, max(x_rightNorm) + 3, 3))
			ax.set_yticklabels(np.arange(min(y_leftNorm) - 3, max(y_leftNorm) + 3, 3))
	
			ax.set_xticks(np.arange(min(x_leftNorm) - 2.5, max(x_rightNorm) + 3.5, 1))
			ax.set_yticks(np.arange(min(y_leftNorm) - 2.5, max(y_leftNorm) + 3.5, 1))

			# Enable grid
			ax.grid(True, which='both', linestyle='--', linewidth=0.3, color='gray')

			ax.set_xlim(x_leftNorm[0] - 2.5, x_rightNorm[-1] + 2.5)
			ax.set_ylim(x_leftNorm[0] - 2.5, x_leftNorm[-1] + 2.5)

		else:
			# Simulate the practical case window
			#squareSize = squareSize - squareDecrease
			middleX = int(x_rightNorm[-1]/2)
			middleY = int(y_rightNorm[-1]/2)
   
			ax.set_xlim(middleX - squareSize/2, middleX + squareSize/2)
			ax.set_ylim(middleY - squareSize/2, middleY + squareSize/2)
			#print(middleX - squareSize/2 , middleX + squareSize/2)
			#print(middleY - squareSize/2 , middleY + squareSize/2)

			#ax.set_xlim(0, squareSize)
			#ax.set_ylim(0, squareSize)
   
			# Fill the remaining area with black from (middleX - squareSize/2) to (middleX + squareSize/2)
			# and from (middleY - squareSize/2) to (middleY + squareSize/2)
			background = np.zeros((squareSize, squareSize))
			
			# Plot the background
			img = ax.imshow(background, interpolation='nearest')
                                  
			#print(imageOut.shape)

		img = ax.imshow(imageOut, interpolation='nearest')
		cb = plt.colorbar(img)
		cb.set_label('ADU')

		print(squareSize)
		plt.tight_layout()
		plt.show()
		