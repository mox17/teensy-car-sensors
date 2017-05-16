enum rotSide {
    ROT_LEFT,
    ROT_RIGHT
};

enum rotDirection {
    ROT_DIR_FORWARD,
    ROT_DIR_BACKWARD, 
    ROT_DIR_NONE,  
    ROT_DIR_ERROR, // this can occur if active phases from the two sensors are not overlapping
};

