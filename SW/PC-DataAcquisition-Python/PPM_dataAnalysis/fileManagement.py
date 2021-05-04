

import csv


def saveOneColumnToCSV(path, column):
    with open(path, 'w', newline='') as csvFile:
        csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
        for i in range(len(column)):   
            csvFileWriter.writerow([column[i]])                    
        csvFile.close()

def appendColumnsToCSV(path,time, column1, column2, column3, column4, column5):
    with open(path, 'a', newline='') as csvFile:
        csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
        csvFileWriter.writerow([time, column1,column2, column3, column4, column5])                    
        csvFile.close()


