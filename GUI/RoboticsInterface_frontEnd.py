#!/usr/bin/python3
"""
creator: Isaac Lima
Date: 17/06/23

A simple GUI for user control of a real robotic arm as moviment display.

This file is Implements the GUI visualization using
Klampt and PyQt5 Modules.
Klampt is used in the Calculations and 3D renderings
PyQt5 is used to Handle user input as well as rendering the main Window.

The program shows an Empty scenario with Robot in the (0, 0, 0) Coordenate axis.
It displays two text Input boxes: Joint; Angle, and Push Button: SEND.
to move the zoom press: shift+LeftClick, to move the camera press: ctrl+LeftClick.

The user is meant to provide a desired Joint/Angle moviment, then press the SEND button.
when the button is pressed the program will show an animation of the desired moviment and
aftwards it will send a HTTP request to the comunication interface so the moviment can be processed 
and executed in the robot arm.

"""

#TODO: Implement the backend HTTP protocol. Create the option to save the current position.
#      Implement an interface that allows the execution of multiple 
#      saved positions in sequence (Control Script).
#      Randians/Degree conversions for Angle input.
#      Collision detection

"""
Used Librarys
"""
from klampt import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import time
import requests

"""
main GUI class: 
defines the functions used in the animations and user input handling
"""
class robotGUI():
    """
    Init function
    Loads the world model and creates the robot variables
    """
    def __init__(self, worldFileName) -> None:
        self.world = WorldModel()
        self.world.readFile(worldFileName)
        self.robot = self.world.robot(0)
        self.q_init = self.robot.getConfig()
        self.desired_angle = 0
        self.desired_joint = 0
        self.PendingMoviment = False
        self.PendingRequest = False
        self.Startup = True

        self.simulation = Simulator(self.world)
        self.dt = 0.01

        self.maxAngleMoviment = 6.282 #2*pi
        self.maxControllerError = 0.5

        self.i = 0 # QGrid row/column index

        self.esp_ip = "192.168.100.30"
        self.url = f"http://{self.esp_ip}/"

    """
    Screen Rendering and animating function
    Renders world
    Executes the Moviment simulation

    A simple P controller is implemented using regime Error
    this controller is used for animation purposes only

    Note: The Real control Algorithm in the robot arm has no relation to the animation display.
    """
    def show(self):
        vis.customUI(self.make_gui)
        vis.add("world", self.world)
        vis.show()
        while vis.shown():
            if self.PendingMoviment:
                joint_angle = self.simulation.controller(0).getCommandedConfig()
                joint_angle[self.desired_joint] = self.desired_angle
                self.simulation.controller(0).setMilestone(joint_angle)
                self.simulation.simulate(self.dt)
                time.sleep(0.0125)
                if self.simulation.controller(0).getCommandedVelocity()[self.desired_joint] == 0:
                    self.PendingMoviment = False
                    self.HTTP_send_comands()
                    self.PendingRequest = False
        
            time.sleep(0.0125)
            
        vis.kill()  

    """
    Qt GUI function builder
    Creates the main Window, Grid layout, QLines and the Push button.
    sets the input handlers for both QLines and Push button.

    returns the created Window
    """

    def make_gui(self, glwidget) -> QMainWindow:
        labels = ["Joint: ", "Angle: "]
        line_edits = []

        self.window = QMainWindow()
        glwidget.setMaximumSize(10000, 10000)
        area = QWidget(self.window)

        layout = QGridLayout()
        layout.addWidget(glwidget, 0 , 0, 10, 10)

        mybutton = QPushButton("SEND DATA ")
        mybutton.clicked.connect(self.clickHandler)
        layout.addWidget(mybutton, 11, 8)

        self.i=0
        for label in labels:
            line_edit = QLineEdit()
            line_edit.setFixedSize(100, 25)
            layout.addWidget(QLabel(label), 11, self.i)
            layout.addWidget(line_edit, 11, self.i+1)
            line_edits.append(line_edit)
            self.i+=2


        self.i = 0
        for line_edit in line_edits:
            line_content = [line_edit, self.i]
            line_edit.returnPressed.connect(lambda line_content = line_content: self.printInput(line_content))
            self.i+=1

        area.setLayout(layout)
        self.window.setCentralWidget(area)
        return self.window


    """
    click Handling function
    it sets the self.PendingMoviment flag 
    so the loaded moviment (Joint/Angle) is executed.
    """
    def clickHandler(self):
        print("SEND REQUEST")
        self.PendingMoviment = True
        

    """
    text loading handler
    receives the input data from the user and loads into 
    self.desired_joint: for the desired joint value
    self.desired_angle: for the desired angle value
    """
    def printInput(self, line_content):
        line_edit = line_content[0]

        # Get the text from the QLineEdit widget
        text = line_edit.text()
        print(text, line_content[1])
        if line_content[1] == 0:
            self.desired_joint = int(text)
            print('CONFIGURED')
        elif line_content[1] == 1:
            self.desired_angle = int(text)*0.01745329251
            print('CONFIGURED')
        
        line_edit.clear()

    def HTTP_send_comands(self):
        """
        This function is called to send the input user data
        to the comunication interface.
        it does so by HTTP Post method, using a JOINT/ANGLE protocol.
        """
        self.PendingRequest = True
        self.desired_angle = round(651.25 * self.desired_angle)
        print(self.desired_angle)
        print("Sending data")
        data = {"Joint": str(self.desired_joint), "Angle": str(self.desired_angle)}
        try:
            response = requests.post(self.url, data=data, timeout=10)
            print(response.text)
        except Exception as error:
            print("Communication failed\n")
            print(error)

        
if __name__ == '__main__':
    robotic_gui = robotGUI("tx90scenario0.xml")
    robotic_gui.show()
