#_*_ encoding:utf-8 _*_
import csv
import codecs
import os

def data_write_csv(datas):#file_name为写入CSV文件的路径，datas为要写入数据列表
    for i in range(100):
        print(('./essay_data/' + str(i) + '.csv'))
        if(os.path.isfile('./essay_data/' + str(i) + '.csv')):
            pass
        else:
            file_name = './essay_data/' + str(i) + '.csv'
            print(i)
            break
    file_csv = codecs.open(file_name,'w+','utf-8')#追加
    writer = csv.writer(file_csv, delimiter=',', quotechar=' ', quoting=csv.QUOTE_MINIMAL)
    for data in datas:
        writer.writerow(data)
    print("保存文件成功，处理结束")

if __name__ == '__main__':
    data = [[1.1,2.2],[2.8,3.89],[3.56,4.23]]
    data_write_csv(data)
