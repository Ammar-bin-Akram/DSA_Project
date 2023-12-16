from tkinter import *
import subprocess


def dijkstra():
    subprocess.run(['python','main_dijkstra.py'])


def A_star():
    subprocess.run(['python','main_a_star.py'])


root2 = Tk()
root2.geometry("500x700")
bg = PhotoImage(file="main_picture.png")
bg_label = Label(root2, image=bg)
bg_label.place(x=0, y=0)

frame1 = Frame(root2, background="blue")
frame1.pack(side=BOTTOM)

q_label = Label(root2, text="Which algorithm you want to use to find the path?", foreground="green", width=40,
                font="Arial 14 bold")
q_label.place(x=10, y=600)

dijkstra_button = Button(root2,text="Dijkstra",bg="green",fg="white",padx=20,pady=10,command=dijkstra)
dijkstra_button.place(x=100,y=650)

a_star_button = Button(root2,text=" A star ", bg="green", fg="white", padx=20, pady=10,command=A_star)
a_star_button.place(x=320,y=650)

root2.mainloop()
