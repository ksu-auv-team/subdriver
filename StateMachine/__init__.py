#all needs to be declared, but what's in it should be overwritten by automodinit
__all__ = ['controllers', 'gbl', 'interact', 'machines', 'search', 'start', 'sub', 'surface', 'taskless', 'test_states', 'track']
# Don't modify the line above, or this line!
import automodinit
automodinit.automodinit(__name__, __file__, globals())
del automodinit
# Anything else you want can go after here, it won't get modified.
