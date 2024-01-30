def usr(robot):

    i = 0
    while i == 0:
        robot.set_led(100,0,0)
        robot.delay(500)
        robot.set_led(0,0,100)
        robot.delay(500)

        i = 1
    return


