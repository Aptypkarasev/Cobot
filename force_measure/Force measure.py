import urx
import socket
import time
from random import randint
from tkinter import Tk, Label, mainloop
from time import sleep


# rob1 = urx.Robot("192.168.8.4", use_rt=True)
rob2 = urx.Robot("192.168.8.3", use_rt=True)
running = True

fname = ('force' + str(time.asctime(time.localtime(time.time()))))

class App(Tk):
    def __init__(self):
        super().__init__()
        self.geometry("800x400+100+80") # создание окна
        self.label1 = Label(self, font='arial 17')
        self.label2 = Label(self, font='arial 17')

        self.label1.pack()
        self.label2.pack()
        self.run()

    def run(self):

            # z1 = (rob1.get_tcp_force()[2])
            # z1 = round(z1, 1)
            z2 = (rob2.get_tcp_force()[2])
            z2 = round(z2, 1)
            #print(z2, '____')
            


            self.label1["text"] = (str('Усилие') + str(' Робот Диагност: ') + str(z2) + str(' Н ' )) # Вывод текста в окне
            # self.label2["text"] = (str('Усилие') + str(' Робот Хирург: ') + str(z1) + str(' Н ' ))
            self.after(10, self.run)
            
            # file = open((fname.replace(':','_') + ".txt"), "a")
            # file.write(str(time.asctime(time.localtime(time.time()))))
            # file.write((' Робот-диагност' + " | " + str(z2) +  " Н" + " | " + ' Робот-хирург' + " | " + str(z1) + " Н" + " | "+str('\n')))
            # file.close
		

while True:



    app = App()
    app.mainloop()
