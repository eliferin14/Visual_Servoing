import cv2
import sys

def downsample_image(input_image_path, output_image_path, target_size):
    # Read the input image
    img = cv2.imread(input_image_path)
    
    # Resize the image to the target size
    resized_img = cv2.resize(img, target_size)
    
    # Write the downsized image to the output path
    cv2.imwrite(output_image_path, resized_img)
    
    print(f"Image successfully downsized to {target_size} and saved to {output_image_path}")

if __name__ == "__main__":
    # Check if the correct number of arguments are provided
    if len(sys.argv) != 5:
        print("Usage: python downsample_image.py <input_image_path> <output_image_path> <target_width> <target_height>")
        sys.exit(1)
    
    # Parse command line arguments
    input_image_path = sys.argv[1]
    output_image_path = sys.argv[2]
    target_width = int(sys.argv[3])
    target_height = int(sys.argv[4])
    target_size = (target_width, target_height)
    
    # Downsample the image
    downsample_image(input_image_path, output_image_path, target_size)