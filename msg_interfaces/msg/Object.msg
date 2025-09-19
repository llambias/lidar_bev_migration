float32 x_relative 
float32 y_relative
float32 x_global
float32 y_global         
uint8 id                   # 0 - 256 
string type "white_unknown" # "white_buoy" / "yellow_landingpad" / "_dock"
bool detected              # 0: not detected, 1: not detected 
float32 certainty 0.1      # 0 - 1 / 0 - 100%
float32 diameter 0.0       # x - y = buoy, y - z = landingpad, z - w = landingpad
