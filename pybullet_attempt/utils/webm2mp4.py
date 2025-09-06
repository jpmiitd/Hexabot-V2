from moviepy.video.io.VideoFileClip import VideoFileClip

def convert_webm_to_mp4(webm_file_path, mp4_file_path):
    """
    Converts a WebM video file to MP4 format.

    Args:
        webm_file_path (str): The path to the input .webm file.
        mp4_file_path (str): The path for the output .mp4 file.
    """
    try:
        # Load the video file
        clip = VideoFileClip(webm_file_path)

        # Write the video to a new file in MP4 format
        # This will automatically handle the video and audio streams
        clip.write_videofile(mp4_file_path)
        
        print(f"Successfully converted '{webm_file_path}' to '{mp4_file_path}'")
        
    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage:
# Replace 'your_video.webm' with your input file name and path
# Replace 'output_video.mp4' with your desired output file name and path
input_file = "/home/jaypea/Videos/Screencasts/hsetup.webm"
output_file = "/home/jaypea/Videos/Screencasts/hsetup.mp4"

convert_webm_to_mp4(input_file, output_file)