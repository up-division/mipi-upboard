import argparse
import numpy as np
import cv2
import os
from PIL import Image

def read_grbg_16bit_file(file_path, width, height):
    # Read the binary file
    with open(file_path, 'rb') as f:
        data = np.fromfile(f, dtype=np.uint16)

    return data[:width * height].reshape((height, width))

def demosaic_grbg_to_rgb(grbg_data):
    return cv2.cvtColor(grbg_data, cv2.COLOR_BayerBG2BGR)
    #return cv2.cvtColor(grbg_data, cv2.COLOR_BayerGB2BGR)

def save_as_jpeg(image_data, output_file, bit_depth):
    # Normalize the image data based on its bit depth
    max_value = np.max(image_data) if bit_depth == 0 else 2 ** bit_depth - 1
    image_data_8bit = (image_data / max_value * 255).astype(np.uint8)
    image = Image.fromarray(image_data_8bit)
    image.save(output_file)

def convert_grbg_to_jpeg(input_file, width, height, output_file, bit_depth):
    grbg_data = read_grbg_16bit_file(input_file, width, height)
    rgb_image = demosaic_grbg_to_rgb(grbg_data)
    save_as_jpeg(rgb_image, output_file, bit_depth)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert GRBG 16-bit binary file to image.')
    parser.add_argument('-o', '--output', type=str, help='Path to the output image file.')
    parser.add_argument('-W', '--width', type=int, default=0, help='Width of the image.')
    parser.add_argument('-H', '--height', type=int, default=0, help='Height of the image.')
    parser.add_argument('-d', '--bit_depth', type=int, choices=[0, 10, 12], default=0, help='Bit depth of the GRBG data (10 or 12).')
    parser.add_argument('input_file', type=str, help='Path to the input binary file.')

    args = parser.parse_args()
    
    out = args.output if args.output else f'{args.input_file}.jpeg'
    width, height = args.width, args.height
    size = os.path.getsize(args.input_file) // 2  # 16bit word count
    if size == 0 or size < width or size < height:
        raise AttributeError("Bad input file")
    if width == 0 and height > 0:
        width = size // height
    elif height == 0 and width > 0:
        height = size // width
    elif height <= 0 and width <= 0:
        raise AttributeError("Bad width or height")

    convert_grbg_to_jpeg(args.input_file, width, height, out, args.bit_depth)
