import matplotlib.pyplot as plt
import math

if __name__ == "__main__":
    plt.figure(figsize=(8,8))
    plt.axis("equal")
    plt.grid()
    
    plt.scatter(0.0, 0.0, color='r', s=50)
    plt.scatter(6.0, -3.0, color='r', s=50)
    
        
    # Walls
    # Outer
    plt.plot([-2.0, 8.0],[ 1.0,  1.0],'b',linewidth=1.5) 
    plt.plot([ 8.0, 8.0],[ 1.0, -6.0],'b',linewidth=1.5)

      
    # Inner
    plt.plot([-2.0, 4.0],[-1.0, -1.0],'b',linewidth=1.5)
    plt.plot([4.0, 4.0],[-1.0, -6.0],'b',linewidth=1.5)


    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()
