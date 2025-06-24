# Publisher
import pygame, zmq, json, time

ADDR = "tcp://127.0.1:5555"
RATE = 50

ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind(ADDR)

pygame.init()
j = pygame.joystick.Joystick(0) 
j.init()


period = 1.0 / RATE
while True:
    pygame.event.pump()
    data = {
        "axes":   [j.get_axis(i)   for i in range(j.get_numaxes())],
        "buttons":[j.get_button(i) for i in range(j.get_numbuttons())]
    }
    sock.send_json(data)
    time.sleep(period)


