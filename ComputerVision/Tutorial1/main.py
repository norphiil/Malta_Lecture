import cv2
import numpy as np


def load_image(image_name: str, flags: int) -> np.ndarray:
    '''
    Load an image from the file system and return it as a numpy array.

    Parameters:
        image_name (str): The name of the image file.

    Returns:
        np.ndarray: The loaded image as a numpy array.
    '''
    image: np.ndarray = cv2.imread(image_name, flags)
    if len(image.shape) == 2:
        (h, w) = image.shape
    else:
        (h, w, d) = image.shape
    image: np.ndarray = cv2.resize(image, (int(h/3), int(w/3)))
    return image


def logarithmicTransformation(image: np.ndarray, scaling_factor: float) -> np.ndarray:
    '''
    Apply logarithmic transformation to the input image.

    Parameters:
        image (np.ndarray): The input image.
        scaling_factor (float): The scaling factor.

    Returns:
        np.ndarray: The transformed image.
    '''
    # Apply logarithmic transformation
    transformed_image: np.ndarray = scaling_factor * np.log(1 + image)
    # Normalize the transformed image
    transformed_image = transformed_image.astype(np.uint8)
    return transformed_image


def powerLawTransformation(image: np.ndarray, scaling_factor: float, gamma: float) -> np.ndarray:
    '''
    Apply power-law transformation to the input image.

    Parameters:
        image (np.ndarray): The input image.
        scaling_factor (float): The scaling factor.
        gamma (float): The gamma value.

    Returns:
        np.ndarray: The transformed image.
    '''
    # Apply power-law transformation
    transformed_image: np.ndarray = scaling_factor * np.power(image, gamma)
    # Normalize the transformed image
    transformed_image = transformed_image.astype(np.uint8)
    return transformed_image


def negativeTransformation(image: np.ndarray) -> np.ndarray:
    '''
    Apply negative transformation to the input image.

    Parameters:
        image (np.ndarray): The input image.

    Returns:
        np.ndarray: The transformed image.
    '''
    # Apply negative transformation
    transformed_image: np.ndarray = 255 - image
    return transformed_image


def pointPorcessing():
    # Load image
    img_original = load_image('Tutorial2/image.jpg', cv2.IMREAD_GRAYSCALE)
    cv2.imshow('Original Image', img_original)
    img_log = logarithmicTransformation(img_original, 255)
    cv2.imshow('Logarithmic Transformation', img_log)
    img_log = logarithmicTransformation(img_original, 2)
    cv2.imshow('Logarithmic Transformation2', img_log)
    img_log = logarithmicTransformation(img_original, 10)
    cv2.imshow('Logarithmic Transformation10', img_log)
    img_log = logarithmicTransformation(img_original, 50)
    cv2.imshow('Logarithmic Transformation50', img_log)
    img_log = logarithmicTransformation(img_original, 100)
    cv2.imshow('Logarithmic Transformation100', img_log)
    # img_power = powerLawTransformation(img_original, 255, 0.5)
    # cv2.imshow('Power Law Transformation', img_power)
    # img_negative = negativeTransformation(img_original)
    # cv2.imshow('Negative Transformation', img_negative)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite('Tutorial2/image_log.jpg', img_log)


pointPorcessing()
