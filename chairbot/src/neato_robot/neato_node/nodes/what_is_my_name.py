"""
Gives you the chairbot number by extracting it from the hostname of the machine
e.g. if the hostname is chairbot03-desktop, it will return the string "03"
"""

import platform

def what_is_my_name():
    hostname =  platform.node()
    return hostname.split('-')[0][-2:]

def what_is_my_number():
    return what_is_my_name()

def chairbot_number():
    return what_is_my_name()

