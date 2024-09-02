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
vs
"""
from klampt import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import time
import requests
import numpy as np

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
        self.robot.setConfig([0.0, 0.0, 0.0, 0.0])
        self.robot.setJointLimits([0.0, -3.22413936106985, -3.22413936106985, -2.530727415391778], [0.0, 3.22413936106985, 3.22413936106985, 2.530727415391778])
        self.robot.setVelocity([0.0, 0.0, 0.0, 0.0])
        self.robot.setVelocityLimits([10.0, 10.0, 10.0, 10.0])
        self.robot.setAccelerationLimits([20.0, 20.0, 20.0, 20.0])
        self.end_effector = self.robot.link(3)
        self.end_effector_length = self.end_effector.geometry().getBBTight()
        self.joint_index_one = 2  # For example, the third joint (0-based index)
        self.joint_index_two = 3
        rotation_matrix, translation_vector = self.robot.link(self.joint_index_two).getTransform()
        self.axis_length = 0.1  # Length of the axes lines
        self.axis_width = 0.005  # Width of the axes lines
        self.coordinate_elements = []
        # X-axis (red)

        # Get the joint's world position
        self.joint_rotation_one = self.robot.link(self.joint_index_one).getTransform()[0]
        self.axis_one = model.coordinates.Frame("joint_one", ([self.joint_rotation_one[0]*2.0, self.joint_rotation_one[1], self.joint_rotation_one[2], self.joint_rotation_one[3], self.joint_rotation_one[4]*2.0
                                                  , self.joint_rotation_one[5], self.joint_rotation_one[6], self.joint_rotation_one[7], self.joint_rotation_one[8]*2.0], 
                                                  self.robot.link(self.joint_index_one).getWorldPosition((0, 0, 0))))
        
        self.joint_rotation_two = self.robot.link(self.joint_index_two).getTransform()[0]
        self.axis_two = model.coordinates.Frame("joint_two", ([self.joint_rotation_two[0]*2.0, self.joint_rotation_two[1], self.joint_rotation_two[2], self.joint_rotation_two[3], self.joint_rotation_two[4]*2.0
                                                  , self.joint_rotation_two[5], self.joint_rotation_two[6], self.joint_rotation_two[7], self.joint_rotation_two[8]*2.0], 
                                                  self.robot.link(self.joint_index_two).getWorldPosition((0, 0, 0))))

        self.q_init = self.robot.getConfig()
        self.desired_angle = 0
        self.desired_joint = 0
        self.desired_angles = [0, 0]
        self.desired_joints = [2, 3]
        self.iter_number = 1
        self.setpoint_steps = []
        self.step_angles_visualization =  np.array([[[0, 0], [2,3]]])
        self.step_angles = []
        self.step_angles_visualization_clear_flag = False
        self.current_step = 1
        self.PendingMoviment = False
        self.PendingRequest = False
        self.Startup = True
        self.showingSteps = False
        self.stepToShow = 0

        self.simulation = Simulator(self.world)
        self.dt = 0.01

        print(self.robot.getJointLimits())
        self.maxAngleMoviment = 6.282 #2*pi
        self.maxControllerError = 0.5

        self.i = 0 # QGrid row/column index

        self.esp_ip = "192.168.136.82"
        self.url = f"http://{self.esp_ip}/"

    """
    Screen Rendering and animating function
    Renders world
    Executes the Moviment simulation

    A simple P controller is implemented using regime Error
    this controller is used for animation purposes only

    Note: The Real control Algorithm in the robot arm has no relation to the animation display.
    """
    def update_axis(self):
        self.joint_rotation_one = self.robot.link(self.joint_index_one).getTransform()[0]
        self.axis_one = model.coordinates.Frame("x", ([self.joint_rotation_one[0]*2.0, self.joint_rotation_one[1], self.joint_rotation_one[2], self.joint_rotation_one[3], self.joint_rotation_one[4]*2.0
                                                  , self.joint_rotation_one[5], self.joint_rotation_one[6], self.joint_rotation_one[7], self.joint_rotation_one[8]*2.0], 
                                                  self.robot.link(self.joint_index_one).getWorldPosition((0, 0, 0))))
            
        self.joint_rotation_two = self.robot.link(self.joint_index_two).getTransform()[0]
        self.axis_two = model.coordinates.Frame("joint_two", ([self.joint_rotation_two[0]*2.0, self.joint_rotation_two[1], self.joint_rotation_two[2], self.joint_rotation_two[3], self.joint_rotation_two[4]*2.0
                                                  , self.joint_rotation_two[5], self.joint_rotation_two[6], self.joint_rotation_two[7], self.joint_rotation_two[8]*2.0], 
                                                  self.robot.link(self.joint_index_two).getWorldPosition((0, 0, 0))))

        self.joint_two_position =  self.robot.link(self.joint_index_two).getWorldPosition((0, 0, 0))
        end_effector_position =  [0, 
                                  self.joint_two_position[1] + 0.421*np.cos(self.robot.getConfig()[self.joint_index_one]+self.robot.getConfig()[self.joint_index_two]),
                                  self.joint_two_position[2] + 0.58*np.sin(self.robot.getConfig()[self.joint_index_one]+self.robot.getConfig()[self.joint_index_two])]

        end_effector_frame = model.coordinates.Point(end_effector_position)
        vis.add("Elo 1", self.axis_one, color=(1, 0, 0, 0.5))     
        vis.add("Elo 2", self.axis_two, color=(0, 1, 0, 0.5))
        vis.add("END EFFECTOR", end_effector_frame, color=(1,0,0,0.5), size=12)
        vis.setAttribute("Elo 1", 'hide_label', True)
        vis.setAttribute("Elo 2", 'hide_label', True)
        #vis.setAttribute("end_effector",'hide_label', True)
        vis.addText(name="coordinates", text=f"Coordenadas End Effector X: {round(125*end_effector_position[0], 0)} Y: {int(125*end_effector_position[1])} Z: {round(125*end_effector_position[2], 0)}", position=[50, 795], color=(0.2, 0.25, 0.95, 1), size=25)
        vis.addText(name="Step", text=f"Executando Passo: {self.stepToShow}", position=[25, 50], color=(0.98, 0.25, 0.2, 1), size=20)
        self.coordinate_elements.append("Coordenadas Elo 1")
        self.coordinate_elements.append("Coordenadas Elo 2") 
        self.coordinate_elements.append("end_effector")   

    def show(self):
        vis.customUI(self.make_gui)
        vis.add("world", self.world)
        vis.show()
        #self.simulation.setGravity((0, 0, -9.8))
        self.simulation.setGravity((0, 0, 0))
        print(self.q_init)
        while vis.shown():
            if(self.mycheckBox.isChecked()):
                self.update_axis()       
                vis.update()
            else:
                if("Coordenadas Elo 1" in self.coordinate_elements and "Coordenadas Elo 2" in self.coordinate_elements):
                    vis.remove("END EFFECTOR")
                    vis.remove("coordinates")
                    vis.remove("Elo 1")
                    vis.remove("Elo 2")
                    vis.remove("Step")
                    self.coordinate_elements.clear()
            if self.PendingMoviment:
                joint_angle = self.simulation.controller(0).getCommandedConfig()
                joint_angle[int(self.desired_joints[0])] = self.desired_angles[0]
                joint_angle[int(self.desired_joints[1])] = self.desired_angles[1]
                self.simulation.controller(0).setMilestone(joint_angle)
                self.simulation.simulate(self.dt)
                time.sleep(0.0125)
                #print(self.robot.getConfig()[self.joint_index_one])
                if self.simulation.controller(0).getCommandedVelocity()[int(self.desired_joints[0])] == 0 and self.simulation.controller(0).getCommandedVelocity()[int(self.desired_joints[1])] == 0:
                    self.PendingMoviment = False
                    if(self.showingSteps):
                        self.stepToShow+=1
                        time.sleep(0.75)

                if(self.showingSteps):
                    if(self.stepToShow >= len(self.step_angles_visualization)):
                        self.showingSteps = False
                        self.stepToShow = 0
                    else:
                        self.showStep()
            time.sleep(0.0125)
        vis.kill()  

    """
    Qt GUI function builder
    Creates the main Window, Grid layout, QLines and the Push button.
    sets the input handlers for both QLines and Push button.

    returns the created Window
    """
    def update_slider(self, value):
        self.slider_laber.setText(f"Número de Iterações: {value}")
        self.iter_number = value

    def showAllSteps(self):
        self.PendingMoviment = True
        self.desired_angles = self.step_angles_visualization[0][0]
        self.desired_joints = self.step_angles_visualization[0][1]
        self.showingSteps = True
    
    def showStep(self):
        self.PendingMoviment = True
        self.desired_angles = self.step_angles_visualization[self.stepToShow][0]
        self.desired_joints = self.step_angles_visualization[self.stepToShow][1]


    def make_gui(self, glwidget) -> QMainWindow:
        labels = ["Junção: ", "Ângulo: "]
        line_edits = []

        self.window = QMainWindow()
        self.window.setWindowTitle("Interface de Programação Robo")
        self.window.setGeometry(480, 120, 1165, 860)
        glwidget.setMaximumSize(10000, 10000)
        area = QWidget(self.window)
        self.window.setCentralWidget(area)

        layout = QGridLayout()
        layout.addWidget(glwidget, 0 , 0, 16, 11)

        description = QLabel("Configuração de Parâmetros: \nJunção (1,2) \nÂngulo (0, 180) \nIterações (1, 10)")
        description.setFont(QFont("Futura", 12))
        layout.addWidget(description, 0, 11)

        mybutton_record = QPushButton("Gravar Passo")
        mybutton_record.setFont(QFont("Futura", 16))
        mybutton_record.clicked.connect(self.recordStep)
        layout.addWidget(mybutton_record, 12, 11)

        mybutton_clear = QPushButton("Limpar Passos")
        mybutton_clear.setFont(QFont("Futura", 16))
        mybutton_clear.clicked.connect(self.cleanSteps)
        layout.addWidget(mybutton_clear, 13, 11)

        mybutton = QPushButton("Enviar Dados")
        mybutton.setFont(QFont("Futura", 16))
        mybutton.clicked.connect(self.clickHandler)
        layout.addWidget(mybutton, 14, 11)

        mybutton_see_steps = QPushButton("Exibir Passos")
        mybutton_see_steps.setFont(QFont("Futura", 14))
        mybutton_see_steps.clicked.connect(self.showAllSteps)
        mybutton_see_steps.setFixedSize(150, 40)
        layout.addWidget(mybutton_see_steps, 8, 11, alignment=Qt.AlignCenter)

        slider = QSlider()
        slider.setOrientation(Qt.Horizontal)
        slider.setRange(1, 10)
        slider.setSingleStep(5)
        slider.setPageStep(0)
        slider.setValue(1)
        slider.setTickPosition(QSlider.TickPosition.TicksAbove)
        slider.setFixedSize(275, 50)
        slider.setTickInterval(10)
        slider.valueChanged.connect(self.update_slider)
        self.slider_laber = QLabel(f"Número de Iterações: 1")
        self.slider_laber.setFont(QFont("Futura", 14))
        layout.addWidget(slider, 2, 11)
        layout.addWidget(self.slider_laber, 1, 11)

        self.mycheckBox = QCheckBox("Habilitar Coordenadas")
        self.mycheckBox.setFont(QFont("Futura", 15))
        layout.addWidget(self.mycheckBox, 9, 11)

        self.mycheckBox_repeat = QCheckBox("Repetir Passos")
        self.mycheckBox_repeat.setFont(QFont("Futura", 15))
        layout.addWidget(self.mycheckBox_repeat, 10, 11)

        self.i=3
        for label in labels:
            mylabel = QLabel(label)
            mylabel.setFont(QFont("Futura", 14))
            line_edit = QLineEdit()
            line_edit.setFixedSize(275, 25)
            layout.addWidget(mylabel, self.i, 11)
            layout.addWidget(line_edit,  self.i+1, 11)
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
        self.HTTP_send_commands()
        self.PendingRequest = False

    
    def cleanSteps(self):
        self.setpoint_steps.clear()
        self.step_angles_visualization_clear_flag = True

    def recordStep(self):
        self.PendingMoviment = True
        
        copy_step_angles = self.step_angles
        self.setpoint_steps.append(copy_step_angles)
        self.step_angles_visualization = np.concatenate((self.step_angles_visualization, [[self.desired_angles, self.desired_joints]]))
        print(self.setpoint_steps)
        self.step_angles = []

    """
    text loading handler
    receives the input data from the user and loads into 
    self.desired_joint: for the desired joint value
    self.desired_angle: for the desired angle value
    """
    def printInput(self, line_content):
        line_edit = line_content[0]

        # Get the text from the QLineEdit widget
        if(self.step_angles_visualization_clear_flag):
            self.step_angles_visualization =  np.array([[[0, 0], [2,3]]])            
            self.step_angles_visualization_clear_flag = False
        text = line_edit.text()
        print(text, line_content[1])
        if line_content[1] == 0:
            self.desired_joint = int(text) + 1
            self.desired_joints[int(text) - 1] = self.desired_joint
            print('CONFIGURED')
        elif line_content[1] == 1:
            self.desired_angle = int(text)*0.01745329251
            self.desired_angles[self.desired_joint - 2] = self.desired_angle

        self.desired_angle_STM = abs(round(57.29*self.desired_angle)) - 90
        if(self.desired_joint == 3):
                self.desired_angle -= 1.5707963259
                self.desired_angles[self.desired_joint - 2] = self.desired_angle

        if self.desired_joint == 2:
           self.desired_joint_STM = 1
        if self.desired_joint == 3:
            self.desired_joint_STM = 2

        iter_number_toSTM = self.iter_number
        if(line_content[1]):
            self.step_angles.append({"Joint": str(self.desired_joint_STM), "K_iter": str(iter_number_toSTM), "Angle": str(self.desired_angle_STM)})
        line_edit.clear()

    def HTTP_send_commands(self):
        """
        This function is called to send the input user data
        to the comunication interface.
        it does so by HTTP Post method, using a JOINT/ANGLE protocol.
        """
        self.PendingRequest = True
        print(self.setpoint_steps)
        while(True):
            for steps_to_target in self.setpoint_steps:
                for data in steps_to_target:
                    try:
                        response = requests.post(self.url, data=data, timeout=5)
                        print(response.text)
                    except Exception as error:
                        print("error")
                    else:
                        print("Sucess")
                time.sleep(1+self.iter_number*0.1)
            if(self.mycheckBox_repeat.isChecked()):
                time.sleep(3)
                continue
            else:
                break
        self.setpoint_steps = []
        
if __name__ == '__main__':
    robotic_gui = robotGUI("tx90scenario0.xml")
    robotic_gui.show()
