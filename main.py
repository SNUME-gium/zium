import add
import divide

print("Let's implement division. Type two numbers for x and y")

x = int(input("x > "))
y = int(input("y > "))
        
print("%d / %d = %0.3f" % (x, y, divide.divide(x, y)))
print("%d + %d = %d" %(x,y,add.add(x,y)))
