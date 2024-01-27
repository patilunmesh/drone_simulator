#remote id message

std_msgs/Header header  #time and frame

##DroneID

uint8 sysid       #0 for broadcasting
uint8 compid      #0 for broadcasting
uint8 id_or_mac   #Data recieved from self VS other UAVs
uint8 idtype      # 0 to 4 based on type of droneID
uint8 uavtype      #0-15 based on the type of the UAV
uint8 uasid       #the id of the UAV

#Drone Location

uint8 status                #0 to 4 for undeclared, ground, air, emergency and failure
float32 heading           #clockwise heading from true north in centi degrees
float32 horizontal_speed      #x y horizontal_speed
float32 vertical_speed        # up is positive and down is negative
float32 lattitude           #GPS lat lon
float32 longitude
float32 altitude            #height
uint8 height_ref            #w.r.t ground or takeoff point 0 or 1
float32 height              # height above referece point
float64[4] state_accuracy   # lat lon accuracy, height and speed accuracy

#externl additional iinfor is omitted
