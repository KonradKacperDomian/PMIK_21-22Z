##
# @mainpage Serial reader/writer
#
# @selection description_main Description
# Python program using pyserial to read and write on serial port

##
# @file main.py
#
# @brief program using pyserial to read and write on serial port
#
# @selection libraries_main Libraries/Modules
# -Pyserial

#imports
import serial
import PySimpleGUI as sg
def Gui():
    '''!Funkcja sluzaca do wyświetlenia interfejsu graficznego

    :param NONE
    :return: NONE
    '''
    layout = [[sg.Text("Program to use 3D scanner\nType name of Scan.xyz file")], [sg.InputText()], [sg.Button("Start Scan")]]
    # Create the window
    window = sg.Window("3D scanner", layout)
    # Create an event loop
    while True:
        event, values = window.read()
        # End program if user closes window or
        # presses the OK button
        if event == "Start Scan":
            window.close()
            filename = values[0]
            sg.popup_no_wait("Scanning")
            StartScan(filename)
            sg.popup("File Saved as", filename)

        if event == sg.WIN_CLOSED:
            break
    window.close()

def SaveXYZfile(PointArray, filename):
    '''!Funkcja sluzaca do zapisu pliku xyz

    :param PointArray: Lista punktow chmury punktow, ktora chcemy zapisac
    :param filename: Nazwa pliku .xyz
    :return: NONE
    '''
    myfile= open(filename+".xyz", "w")
    for lineTxt in PointArray:
        temp = lineTxt
        myfile.write(temp)
    myfile.close()
    print("File saved as "+filename+".xyz")

def StartScan(filename):
    ser.write(b"Start\n")
    tempReadFromCom = []
    pointArray = []
    SaveData = True
    while (SaveData):
        tempReadFromCom.append(ser.read().decode("utf-8"))
        if (len(tempReadFromCom) == 0):
            SaveData = False
        if tempReadFromCom[-1] == '\n':
            comunicats = "".join(tempReadFromCom)
            # print(comunicats)
            if (comunicats == "Scanning finish\n"):
                break
            pointArray.append(comunicats)
            tempReadFromCom.clear()
    pointArray.pop(0)
    SaveXYZfile(pointArray, filename)

if __name__ == '__main__':
    global ser
    ser = serial.Serial('COM5')
    Gui()






