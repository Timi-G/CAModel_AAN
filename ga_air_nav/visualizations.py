import os

import cv2 as cv
import matplotlib.pyplot as plt

# Get images directory path dynamically
CURRENT_DIRECTORY = os.path.abspath(os.path.dirname(__file__))
PARENT_DIRECTORY = os.path.abspath(os.path.join(CURRENT_DIRECTORY, '..'))
image_dir = os.path.join(PARENT_DIRECTORY, 'images')

# c_s is string arg for choosing color and symbol
# !! one of parameters frm_close or close must be set as True
def create_simple_plots(frames_dict, axis_ranges, image_dir=image_dir, c_s='r+', frm_close=False, close=False):
    # check and delete if images folder has any content
    try:
        # remove images from 'images folder' if folder is not empty
        for img in os.listdir(image_dir):
            img_file_path = os.path.join(image_dir,img)
            if os.path.isfile(img_file_path):
                os.remove(img_file_path)
    except:
        pass

    for m, arrays in frames_dict.items():
        for n, arr in enumerate(arrays):
            if isinstance(arr[0],int):
                x, y = arr
                plt.plot(x, y,'gD')
                plt.xlim(axis_ranges[0])
                plt.ylim(axis_ranges[1])
            elif len(arr[0]) <= 2:
                for a in arr:
                    x, y = a
                    plt.plot(x, y, c_s)
                    plt.xlim(axis_ranges[0])
                    plt.ylim(axis_ranges[1])
            else:
                x, y = arr
                plt.scatter(x, y)
                plt.xlim(axis_ranges[0])
                plt.ylim(axis_ranges[1])
            if close:
                images_file = os.path.join(image_dir, f'plot_{n}.png')
                plt.savefig(images_file)
                plt.close()
        if frm_close:
            images_file = os.path.join(image_dir, f'plot_{m}.png')
            plt.savefig(images_file)
            plt.close()

def make_video(video_name, image_dir=image_dir):
    unsorted_images = [img for img in os.listdir(image_dir)
              if img.endswith(".jpg") or img.endswith(".jpeg") or img.endswith("png")]

    images = sorted(unsorted_images, key=lambda x: int(x.split('_')[1].split('.')[0]))
    # to get video frame
    frame = cv.imread(os.path.join(image_dir,images[0]))
    height,width,layers=frame.shape

    video=cv.VideoWriter(video_name,0,1,(width,height))

    for image in images:
        frame=cv.imread(os.path.join(image_dir,image))
        video.write(frame)
    video.write(cv.imread(os.path.join(image_dir, images[-1])))

    video.release()
    # Deallocating memories taken for window creation
    cv.destroyAllWindows()