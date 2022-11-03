# State Machine:
Use state machine to go between states
### States:
 * Following road (potentially for regular road and grassy, hilly road)
 * Sees centre car, avoid it
 * Sees crosswalk (red line), wait for pedestrian to cross, then move straight very fast
 * Sees parked car, get image of license plate, identify it

## Navigating Map
Use predefined directions for a good route of the map

## Following road
Use PID control on the two white outer lines, can do multiple PID control loops for linear velocity and angular velocity

## Image Recognition
Use SIFT to identify the objects we need to identify such as license plates, cars and pedestrians
Can be used to turn the license plate back into a rectangle for processing in the neural network

## License plate recognition
Train the NN on google colab with images obtained from the simulation or using generated training data after being put through some processing
