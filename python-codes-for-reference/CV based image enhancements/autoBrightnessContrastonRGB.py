def auto_brightness_contrast(image, clip_hist_percent=1):
    #adapted from stackoverflow https://stackoverflow.com/questions/56905592/automatic-contrast-and-brightness-adjustment-of-a-color-photo-of-a-sheet-of-pape
    Bchnl = image[:,:,0]
    Gchnl = image[:,:,1]
    Rchnl = image[:,:,2]
    
    tunedB = auto_tune_channel(Bchnl)
    tunedG = auto_tune_channel(Gchnl)
    tunedR = auto_tune_channel(Rchnl)
        
    #avg_max_gray = (maximum_B + maximum_G + maximum_R) / 3
    #avg_min_gray = (minimum_B + minimum_G + minimum_R) / 3
    # Calculate alpha and beta values
    #alpha = 255 / (avg_max_gray - avg_min_gray)
    #beta = -avg_min_gray * alpha
    #print(alpha, beta)
    #auto_result = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    #return auto_result
    
    combined_tuned_channels = cv2.merge((tunedB, tunedG, tunedR))
    return combined_tuned_channels
    
def auto_tune_channel(channel, clip_hist_percent=1): 
    # Calculate grayscale histogram
    hist = cv2.calcHist([channel],[0],None,[256],[0,256])
    hist_size = len(hist)
    
    # Calculate cumulative distribution from the histogram
    accumulator = []
    accumulator.append(float(hist[0]))
    for index in range(1, hist_size):
        accumulator.append(accumulator[index -1] + float(hist[index]))
    
    # Locate points to clip
    maximum = accumulator[-1]
    clip_hist_percent *= (maximum/100.0)
    clip_hist_percent /= 2.0
    
    # Locate left cut
    minimum_gray = 0
    while accumulator[minimum_gray] < clip_hist_percent:
        minimum_gray += 1
    
    # Locate right cut
    maximum_gray = hist_size -1
    while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
        maximum_gray -= 1
    
    # Calculate alpha and beta values
    alpha = 255 / (maximum_gray - minimum_gray)
    beta = -minimum_gray * alpha
    
    tuned_channel = cv2.convertScaleAbs(channel, alpha=alpha, beta=beta)
    return tuned_channel
