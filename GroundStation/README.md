# GroundStation

### Contains a single Python script used to stream joystick data to the Jetson Nano over an UDP WebSocket.

## Flow

- reads the joystick axes and buttons using the `pygame` library
- rounds the axes to 3 decimals
- sends the data using an UDP socket (`socket.SOCK_DGRAM`) 

#### The UDP send address (IP and Port) are configured to be on the local network of the Wi-Fi hotspot created by the laptop.
