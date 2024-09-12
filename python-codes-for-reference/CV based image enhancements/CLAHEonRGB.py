def apply_clahe_rgb(image, clip_limit=2.0, tile_grid_size=(8, 8)):
    
    Bchnl = image[:,:,0]
    Gchnl = image[:,:,1]
    Rchnl = image[:,:,2]
    
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)

    # Apply CLAHE to each channel
    ClaheB = clahe.apply(Bchnl)
    ClaheG = clahe.apply(Gchnl)
    ClaheR = clahe.apply(Rchnl)
    clahe_image = cv2.merge((ClaheB, ClaheG, ClaheR))  # Merge the CLAHE-enhanced channels

    return clahe_image

