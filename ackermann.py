import numpy as np
import math

class ackermann():
    def ackermann_steer_step(x, y, heading_deg, steering_deg, wheelbase, speed, dt):

        #params definition:
        # x: x-coordinate of the track
        # y: y-coordinate of the track
        # heading_deg: current heading
        # steering_deg: Based on PID error, this is the angle it wants to turn by
        # wheelbase: Distance between from axel and rear axel wheel
        # dt time interval between each frame shot
        # If the path is straight, then its angle does not change. Only the x and y positions change
        if steering_deg == 0:
            heading_rad = math.radians(heading_deg)
            x += speed * dt * math.cos(heading_rad)
            y += speed * dt * math.sin(heading_rad)
            return x, y, heading_deg

        # Otherwise if the angle is not 0, calcualte the new position.
        steering_rad = math.radians(steering_deg)

        # This calculates the radius of the turn
        radius = wheelbase / math.tan(steering_rad)

        # Simple angular velocity translation
        angular_velocity = speed / radius

        heading_rad = math.radians(heading_deg)
        old_rad = heading_rad
        heading_rad = angular_velocity * dt
        heading_deg = math.degrees(heading_rad)

        # Using paramterisation to calcualte the new positions
        x += radius * (math.sin(heading_rad) - math.sin(old_rad))
        y += radius * (-math.cos(heading_rad) + math.cos(old_rad))
        return x, y, heading_deg