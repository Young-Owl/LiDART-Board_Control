import matplotlib.pyplot as plt
import numpy as np

# Laser parameters
laser_diameter = 4.5e-3
laser_divergence = 0.2e-3
wavelength = 850e-9

# Camera parameters
focal_length = 16e-3 					# In meters

# Field of view in radians per pixel
AfovPxRad = 0.024* np.pi / 180 			# 0.024 degrees - (Valor Renato Tese, medido)
print(AfovPxRad)

# Parameters for Lidar equation
ar = np.pi * (11.4e-3 / 2) ** 2			# Aperture area in meters
reflectivity = 80 / 100 				# Includes object reflectivity and objective lens transmissivity

# Pulse Energy calculation
laser_power = 30e-3						# Laser power in watts (30 mW)
frame_rate = 50							# Frame rate in Hz
pulse_energy = laser_power / frame_rate	# Pulse energy in joules	

# Constants
h = 6.626e-34  							# J*s
c = 299792458  							# m/s

# Parameters for noise model
quantum_efficiency = 6.25/100			# 6.25% Quantum efficiency - (Valor medido)
sensitivity = 1.923						# Sensitivity in electrons per ADU
dark_noise = 6.83						# Dark noise in electrons
bit_depth = np.log2(57218)#12			# 12-bit camera
baseline = 112							# Baseline ADU count
maximum_adu = int(2**bit_depth - 1)		# Maximum ADU count

# Random seed
seed = 42
rs = np.random.RandomState(seed)


def get_noise(point, d, aux_x, aux_y):

	# Estimate laser dot diameter
	spot_diameter = laser_diameter + 2 * d * np.tan(laser_divergence / 2)	

	# Estimate pixel width
	pixel_width = 2 * d * np.tan(AfovPxRad / 2)
 
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
	# print(num_photons)


	# Generate circle based on point
	# Generate the laser circumference
	x = np.linspace(-(spot_diameter / 2), +(spot_diameter / 2), 256)	# 256 points
	#xx = np.linspace(- (110/2), + (110/2), 256)	# 256 points
	y = np.sqrt(spot_diameter**2 / 4 - x**2)						

	# Generate laser dot
	#x = np.linspace((110/2),(110/2), 1024)	# 256 pointsxx + 0
	#y = np.sqrt((110)**2 / 4 - x**2)		

	x = np.hstack((x, x[::-1]))
	y = np.hstack((y, -y))

	# Identify which pixels are illuminated by the laser dots
	x1minPx = np.floor(np.min(x / pixel_width) + aux_x)
	x1maxPx = np.ceil(np.max(x / pixel_width) + aux_x)
	y1minPx = np.floor(np.min(y / pixel_width) + aux_y)
	y1maxPx = np.ceil(np.max(y / pixel_width) + aux_y)

	# Identify the meshgrid of pixels
	xPx = np.arange(x1minPx, x1maxPx + 1)
	yPx = np.arange(y1minPx, y1maxPx + 1)
	#xPx = xPx.astype(dtype='uint16')
	#yPx = yPx.astype(dtype='uint16')
	XPx, YPx = np.meshgrid(xPx, yPx,)

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
	
	# Pixelization
	# Total number of photons in each pixel
	num_photons_per_pixel = num_photons * dotsInPixel

	# print(num_photons_per_pixel)

	output_matrix = add_camera_noise(num_photons_per_pixel, noiseOn=True, baselineOn=True)

	return output_matrix

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
		adu[adu > maximum_adu] = maximum_adu # Models pixel saturation
	
	else:

		# Convert electrons to Analog-to-Digital Units (ADU)
		adu = (num_photons * quantum_efficiency * sensitivity).astype(int)
  
		adu[adu > maximum_adu] = maximum_adu # Models pixel saturation
	
	return adu


if __name__ == "__main__":

	for d in [1,2,4,8]:
		print('Distance = ' + str(d) + ' m')
		point = [0, 0, d, 1]
		sum = 0

		imageOut = get_noise(point, d, 0, 0)
		#Go pixel by pixel, summing its value
		for i in range(imageOut.shape[0]):
			for j in range(imageOut.shape[1]):
				if int(imageOut[i][j]) >= (baseline * 2):
					sum = sum + int(imageOut[i][j])

		print('Sum = ' + str(sum))

		# Max pixel value
		maxPixelValue = float(np.amax(imageOut))
		print('Max pixel value = ' + str(maxPixelValue))

		cnt = 0
		# Count the number of pixels that are above the baseline (112 ADU)
		for i in range(imageOut.shape[0]):
			for j in range(imageOut.shape[1]):
				if imageOut[i, j] > (2*baseline):
					cnt = cnt + 1
		print('Number of pixels above the baseline = ' + str(cnt))

		squareSize = 20

		# Plot the laser dot and fill the rest of the empty space with zeros
		# These zeros should also be color coded to represent the background noise
		x = np.linspace(-(squareSize/2), +(squareSize/2)+1, 256)	# 256 points
		y = np.linspace(-(squareSize/2), +(squareSize/2)+1, 256)
		x = np.hstack((x, x[::-1]))
		y = np.hstack((y, -y))

		# Ensure padding is correct for the image
		imageOut = np.pad(imageOut, ((int((squareSize - imageOut.shape[0]) / 2), 
									int((squareSize - imageOut.shape[0]) / 2))), 
									'constant', constant_values=(0, 0))

		fig, ax = plt.subplots()

		# Set axis limits based on the actual size of imageOut
		ax.set_xlim(0, imageOut.shape[1])
		ax.set_ylim(0, imageOut.shape[0])

		# Enable grid (both major and minor grids)
		#ax.grid(True, which='major', linestyle='--', linewidth=0.5, color='gray')

		# Optionally, if you want minor grid lines too
		#ax.minorticks_on()
		#ax.grid(True, which='minor', linestyle=':', linewidth=0.3, color='gray')

		ax.set_xticks(np.arange(0.5, squareSize - 0.5, 3))
		ax.set_yticks(np.arange(0.5, squareSize - 0.5, 3))  

		ax.set_xticklabels(np.arange(0, squareSize, 3))
		ax.set_yticklabels(np.arange(0, squareSize, 3))

		ax.set_xticks(np.arange(0.5, squareSize - 0.5, 1))
		ax.set_yticks(np.arange(0.5, squareSize - 0.5, 1))
  
		ax.grid(True, which='both', linestyle='--', linewidth=0.3, color='gray')

		# Display image
		img = ax.imshow(imageOut, interpolation='nearest')

		# Add colorbar
		cb = plt.colorbar(img)
		cb.set_label('ADU')

		# Show the plot
		#plt.savefig('Sim1Laser' + str(d) + '.png')
		plt.show()
		""" # Add to imageOut zeros to fill the squareSize matrix
		imageOut = np.pad(imageOut, ((int((squareSize - len(imageOut))/2), int((squareSize - len(imageOut))/2))), 'constant', constant_values=(0, 0))
		

		fig, ax = plt.subplots()
		ax.set_xticks([])
		ax.set_yticks([])

		#Set x axis limit to squareSize pixels
		ax.set_xlim(-(squareSize/2 - (len(imageOut)/2)), (squareSize/2 + (len(imageOut)/2)-1))
		ax.set_ylim(-(squareSize/2 - (len(imageOut)/2)), (squareSize/2 + (len(imageOut)/2)-1))

		# Enable grid
		ax.grid(True, which='both', linestyle='--', linewidth=0.3, color='gray')

		img = ax.imshow(imageOut, interpolation='nearest')
		cb = plt.colorbar(img)
		cb.set_label('ADU')
		#plt.savefig('distance_' + str(d) + '.png')
		plt.show() """

		# Plot the histogram
		""" fig, ax = plt.subplots()
		ax.hist(imageOut.flatten(), bins=256, range=(0, 220))
		ax.set_title('Histogram of ADU values')
		ax.set_xlabel('ADU')
		ax.set_ylabel('Number of pixels')
		plt.savefig('HistogramModel' + str(d) + '_histogram.png')
		plt.show() """
