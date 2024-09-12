def apply_histogram_equalization_rgb(image):

    Bchnl = image[:,:,0]
    Gchnl = image[:,:,1]
    Rchnl = image[:,:,2]
    equalB = cv2.equalizeHist(Bchnl)
    equalG = cv2.equalizeHist(Gchnl)
    equalR = cv2.equalizeHist(Rchnl)
    equalized_image = cv2.merge((equalB, equalG, equalR))  # Merge the equalized channels

    return equalized_image



