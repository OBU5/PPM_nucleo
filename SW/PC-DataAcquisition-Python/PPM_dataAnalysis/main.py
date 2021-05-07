import analysisController
import communicationManagement


def main():
    #message = b'<MEAS:extADC:INF>'
    #analysisController.analyseDatafthroughUSB(port = "COM9", baudrate = 2000000, messageCountPerMeasurement = 2, countOfMeasurements = -1, fileName = 'out2.txt' )
    analysisController.analyseDataFromFile(nameOfFile = 'meas1.txt' )

if __name__ == "__main__":
    main()