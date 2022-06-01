import csv 
import csv
with open('/home/zcy/essay_data/1.csv', 'r') as f:
    reader = csv.reader(f)
    print(type(reader))
   
    for row in reader:
        print(float(row[0]),float(row[1]))