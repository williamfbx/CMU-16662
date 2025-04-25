from PIL import Image
from matplotlib import pyplot as plt
import numpy as np
import requests
from io import BytesIO
# import pytesseract


class Segment:
    def __init__(self, model_type="sam2.1_hiera_small"):
        self.model = model_type
        self.url = f"https://segment.thomaschan.dev/predict"

    def segment(
        self, image_path, prompt, viz=False, box_threshold=0.3, text_threshold=0.30
    ):

        with open(image_path, "rb") as img_file:
            files = {"image": img_file}

            data = {
                "sam_type": "sam2.1_hiera_small",
                "box_threshold": str(box_threshold),
                "text_threshold": str(text_threshold),
                "text_prompt": prompt,
                "mask": True,
            }

            try:
                response = requests.post(self.url, files=files, data=data)
            except Exception as e:
                print(f"Request failed: {e}")
                return None

        output = BytesIO(response.content)
        
        output_img = Image.open(output)

        # extracted_text = pytesseract.image_to_string(output_img).strip()

        # # Compare extracted text with prompt
        # if extracted_text.lower() != prompt.lower():
        #     print(f"Segmented text '{extracted_text}' does not match prompt '{prompt}'.")
        #     return None
    
        # convert output to numpy array
        output_np = np.array(Image.open(output))

        mask_pos = np.where(output_np == 255)

        # get the mean of the mask position
        mask_mean = np.mean(mask_pos, axis=1)

        if viz:

            # plot the output
            plt.imshow(output_np, cmap="gray")

            # plot the mask position
            plt.scatter(mask_mean[1], mask_mean[0], c="red", s=10)

            # show the coordinates
            plt.text(
                mask_mean[1] - 100,
                mask_mean[0] - 50,
                f"({int(mask_mean[1])}, {int(mask_mean[0])})",
                backgroundcolor="white",
            )

            plt.axis("off")
            plt.show()

        return output_np, mask_mean


if __name__ == "__main__":
    seg = Segment()

    image_path = "./calibration/image.png"
    prompt = "Green pear"

    image = Image.open(image_path).convert("RGB")
    image_arr = np.asarray(image)

    results, _ = seg.segment(image_path, prompt, viz=True)
