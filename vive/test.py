try:
    from .models import ViveTrackerMessage
except:
    from models import ViveTrackerMessage
v = ViveTrackerMessage(x=1.231123,y=2,z=3)
print(v.x)