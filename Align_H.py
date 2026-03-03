import urx
import math3d as m3d


def main():
    rob = urx.Robot("192.168.8.4")

    pose = rob.getl()
    rob.movel((pose[0],pose[1],pose[2],3.14/2,0,0), acc = 0.2, vel=0.2)


if __name__ == "__main__":
    main()