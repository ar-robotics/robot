import time  # this should give an error

var = True

# flake8 should give an error here
if var == True:
    pass

"""this line should be too long for flake8 and should give an error because its over 88 chars"""
"""this line should not give an error since it is under 88 chars"""

# since we use "no quality assurance" on this line flake should not give an error
if var == True:  # noqa
    pass
