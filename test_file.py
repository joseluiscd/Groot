import sys
sys.path.append("build/app")
import groot

r = groot.Registry()
r.load("cylinders.ggf")
e = r.entities()[0]
c = e.cylinders()[0]
print(c.points())
u = c.points()
print(c.points())

print(c.cylinder.center.x)
print(c.cylinder.center.y)
print(c.cylinder.center.z)
print(c.cylinder.center.as_numpy())
