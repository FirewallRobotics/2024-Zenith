import unicornhat
import time

def scroll_text(text, speed=0.1, brightness=0.5):
        unicornhat.brightness(brightness)

        for char in text + '   ':
            unicornhat.clear()

            for x in range(8):
                for y in range(4):
                    pixel = text_pixel(char, x, y)
                    unicornhat.set_pixel(x, y, *pixel)

            unicornhat.show()
            time.sleep(speed)

def text_pixel(char, x, y):
    try:
        index = ord(char) - ord(' ')
        return unicornhat.get_pixel(x, y, index)
    except IndexError:
        return (0, 0, 0)

def coolstuff(dist):
    unicornhat.clear()
    unicornhat.set_pixel(0,0,255,255,255)
    unicornhat.set_pixel(0,1,dist*10,dist*10,dist*10)
    scroll_text("Firewall - - - - - - - - 5607 - - - - - - -")