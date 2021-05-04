import signalAnalysis
import communicationManagement


def main():
    message = b'<MEAS:extADC:INF>'
    communicationManagement.sendMessage(port = "COM9", baudrate = 2000000, message = message)
    communicationManagement.sendMessage(port = "COM9", baudrate = 2000000, message = message)
    communicationManagement.sendMessage(port = "COM9", baudrate = 2000000, message = message)
    communicationManagement.sendMessage(port = "COM9", baudrate = 2000000, message = message)
    communicationManagement.sendMessage(port = "COM9", baudrate = 2000000, message = message)
    communicationManagement.sendMessage(port = "COM9", baudrate = 2000000, message = message)
    communicationManagement.sendMessage(port = "COM9", baudrate = 2000000, message = message)
    signalAnalysis.analyseDatafthroughUSB(port = "COM9", baudrate = 2000000, messageCountPerMeasurement = 2, countOfMeasurements = 10 )

if __name__ == "__main__":
    main()