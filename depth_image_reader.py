import numpy as np
import cv2

# Specify the input text file containing the BGR8 image data
input_file = "image_data.txt"

# Read image data from the file
with open(input_file, 'r') as file:
    image_data = file.read().split(',')
image_data = [int(value.strip()) for value in image_data]

# Calculate the number of channels (BGR8 has 3 channels)
num_channels = 3

# Determine the dimensions based on the number of data points
data_length = len(image_data)
num_pixels = data_length // num_channels
sqrt_pixels = int(np.sqrt(num_pixels))
height = width = sqrt_pixels

# Reshape the data to match the image dimensions
image_data = np.array(image_data).reshape(height, width, num_channels)

# Convert BGR8 to RGB
rgb_image = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)

# Display or save the RGB image
cv2.imshow('RGB Image', rgb_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# If you want to save the image as a file, use cv2.imwrite
# cv2.imwrite('output_image.jpg', rgb_image)
