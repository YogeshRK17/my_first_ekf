import numpy as np
import matplotlib.pyplot as plt

class strightLinePath():
    def __init__(self):
        self.ax_limit = 50

        self.x_start = 0
        self.y_start = 0
        self.x_end   = 0
        self.y_end   = 0

        self.got_start_point = False
        self.got_end_point   = False

    def render_quadrants(self, ax_lim=50):
        fig, ax = plt.subplots()
        ax.axhline(y=0, color='r', linewidth=1)
        ax.axvline(x=0, color='r', linewidth=1)

        ax.set_xlim(-ax_lim,ax_lim)
        ax.set_ylim(-ax_lim,ax_lim)

        ax.grid(True)

        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.gca().set_aspect('equal', adjustable='box')  # Equal aspect ratio

    def get_start_points(self):
        while(not self.got_start_point):
            try:
                self.x_start = int(input("Enter x co-ordinate of start point: "))
                self.y_start = int(input("Enter y co-ordinate of start point: "))
                self.got_start_point = True
            except Exception as e:
                print(e)
                print("Encountered error while taking start points, please try again...")

    def get_end_points(self):
        while(not self.got_end_point):
            try:
                self.x_end = int(input("Enter x co-ordinate of end point: "))
                self.y_end = int(input("Enter y co-ordinate of end point: "))
                self.got_end_point = True
            except Exception as e:
                print(e)
                print("Encountered error while taking end points, please try again...")

    def plot_points(self):
        get_max_ax_lim = self.max_of([self.x_start, self.x_end, self.y_start, self.y_end])
        self.render_quadrants(ax_lim=1.5*get_max_ax_lim)
        plt.scatter(self.x_start, self.y_start, color='g')
        plt.scatter(self.x_end, self.y_end, color='r')

    def max_of(self, num_list):
        if len(num_list) == 0:
            print("No elemets in list")
            return 0
        
        self.max_num = num_list[0]
        for i in num_list:
            if i > self.max_num:
                self.max_num = i
            
        return self.max_num

    def logic_test(self):
        # D = np.sqrt((self.x_start - self.x_end)**2 + (self.y_start - self.y_end)**2)
        x1, y1 = 10, 20
        x2, y2 = 20, 30

        D = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        print(D)
        V = 1
        T = D/V
        dt = 0.1
        Dx = (x2- x1)/D
        Dy = (y2 -y1)/D

        dx, dy = 0, 0

        plt.scatter(x1, y1)
        plt.scatter(x2, y2)

        for i in range(int(T/dt)):
            dx = dx + V*dt*Dx
            dy = dy + V*dt*Dy
            plt.scatter(x1 + dx, y1 + dy)



def main():
    strightLinePath_obj = strightLinePath()
    # strightLinePath_obj.get_start_points()
    # strightLinePath_obj.get_end_points()
    # strightLinePath_obj.plot_points()

    strightLinePath_obj.logic_test()
    plt.show()

if __name__ == '__main__':
    main()