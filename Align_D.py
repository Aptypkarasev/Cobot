import urx

def main():
    rob = urx.Robot("192.168.8.3")

    pose = rob.getl()
    rob.movel((pose[0],pose[1],pose[2],0,3.14,0), acc = 0.2, vel=0.2)

if __name__ == "__main__":
    main()