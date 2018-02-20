# FollowMe
Autonomous follow me function for a model-car.

The model-car is a regular RC-toy-car that is modified with an Arduino Mega and a Raspberry Pi. It uses a compass, GPS and camera to get its curent position.

The user can controll the car over a Blutooth connection by an Andriod App.
At the beginning the car navigates by its own GPS to the GPS-coordinates of the Person with the Android-App. Afterwards it trys to find the person in the cameras view to switch to a vision based following of the person.
