import cv2
import numpy as np


def load_image(image_name: str) -> np.ndarray:
    '''
    Load an image from the file system and return it as a numpy array.

    Parameters:
        image_name (str): The name of the image file.

    Returns:
        np.ndarray: The loaded image as a numpy array.
    '''
    image: np.ndarray = cv2.imread(image_name)
    (h, w, d) = image.shape
    image: np.ndarray = cv2.resize(image, (int(h/3), int(w/3)))
    return image


def ex1():
    '''
    Load an image from the file system and display it in a window.
    '''
    image: np.ndarray = cv2.imread('image.jpg')
    (h, w, d) = image.shape
    # resize image
    image: np.ndarray = cv2.resize(image, (int(h/3), int(w/3)))
    # show image
    cv2.imshow('Image', image)
    # cv2.imwrite('image.png', image)
    cv2.waitKey(0)


def ex2():
    '''
    Open a video capture device and display the video in a window.
    '''
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        threshold1 = 100
        ret, thresh1 = cv2.threshold(gray, threshold1, 255, cv2.THRESH_BINARY)
        cv2.imshow('thresh1', thresh1)
        cv2.imshow('frame', frame)
        cv2.imshow('gray', gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()


def ex3():
    '''
    Load an image from the file system and display the red, green, and blue image channels in separate windows.
    '''
    img: np.ndarray = load_image('image.jpg')
    blue: np.ndarray = img.copy()
    blue[:, :, 1] = 0
    blue[:, :, 2] = 0
    green: np.ndarray = img.copy()
    green[:, :, 0] = 0
    green[:, :, 2] = 0
    red: np.ndarray = img.copy()
    red[:, :, 0] = 0
    red[:, :, 1] = 0
    cv2.imshow('Red', red)
    cv2.imshow('Green', green)
    cv2.imshow('Blue', blue)
    cv2.waitKey(0)


def ex4():
    '''
    Load an image from the file system and display the original and blurred images in separate windows.
    '''
    img: np.ndarray = load_image('image.jpg')
    blur: np.ndarray = cv2.blur(img, (5, 5))
    cv2.imshow('Original', img)
    cv2.imshow('Blurred', blur)
    cv2.waitKey(0)


def ex5():
    '''
    Load an image from the file system and display the original and edge detected images in separate windows.
    '''
    img: np.ndarray = load_image('image.jpg')
    gray: np.ndarray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edge: np.ndarray = cv2.Canny(gray, 50, 150)
    cv2.imshow('Original', img)
    cv2.imshow('Edge', edge)
    cv2.waitKey(0)
